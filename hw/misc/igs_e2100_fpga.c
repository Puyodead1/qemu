/*
 * IGS E2100 PCI/FPGA interface used by Fish Hunter / Harpoon.
 *
 * The vendor Linux 2.6.32 driver binds PCI 11aa:1770, maps BAR0 and
 * BAR1, and uses an I/O BAR at BAR3 for interrupt status/acknowledge.
 *
 * The protection ASIC behind the FPGA is high-level emulated:
 *   - boot handshake on the BAR1 mailbox (0x100..0x11F),
 *   - the runtime message channel (BAR1 header register + BAR0 body),
 *   - the DMA engine with the FPGA asset cipher applied in-line.
 * See IGS_E2100_PROTECTION.md and IGS_SOUND_CRYPTO.md for the reverse
 * engineered protocol.
 */

#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "migration/vmstate.h"
#include "system/dma.h"
#include "system/address-spaces.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "ui/input.h"
#include "hw/core/qdev-properties.h"

#define TYPE_IGS_E2100_FPGA "igs-e2100-fpga"
OBJECT_DECLARE_SIMPLE_TYPE(IGSE2100FPGAState, IGS_E2100_FPGA)

#define IGS_E2100_VENDOR_ID 0x11aa
#define IGS_E2100_DEVICE_ID 0x1770

#define IGS_E2100_BAR0_SIZE 0x80000
#define IGS_E2100_BAR1_SIZE 0x00800
#define IGS_E2100_BAR3_SIZE 0x00100

/* BAR3 I/O ports (kernel FPGA_iIoctl / ISR) */
#define IGS_E2100_IO_EVENT_ACK     0x03
#define IGS_E2100_IO_NOTIFY        0x04  /* ioctl 0x6601 */
#define IGS_E2100_IRQ_STATUS       0x07
#define IGS_E2100_IO_ASIC_RESET    0x08  /* ioctl 0x6600 */
#define IGS_E2100_IO_DMA_LEN       0x10
#define IGS_E2100_COMMAND          0x13
#define IGS_E2100_IRQ_STATUS_ALT   0x17
#define IGS_E2100_IO_DMA_SRAM      0x18
#define IGS_E2100_IO_DMA_DESC      0x1c
#define IGS_E2100_IO_KEY           0x20
#define IGS_E2100_IO_MODE          0x40

#define IGS_E2100_IRQ_EVENT        0x10
#define IGS_E2100_IRQ_GAME_TICK    0x40
#define IGS_E2100_DMA_BUSY         0x01

#define IGS_E2100_DMA_TO_SRAM      0x11
#define IGS_E2100_DMA_FROM_SRAM    0x51

/* BAR1 mailbox: separate host->ASIC and ASIC->host words */
#define IGS_E2100_MBOX_BASE        0x100
#define IGS_E2100_MBOX_WORDS       8

/* Boot handshake commands (XOR session key k) */
#define ASIC_CMD_HELLO             0xF0F0F0F0u
#define ASIC_CMD_DONE              0x90909090u
#define HOST_REPLY_RETRY           0x0A0A0A0Au
#define HOST_REPLY_DONE            0x0C0C0C0Cu

/* Runtime channel */
#define ASIC_RX_KEY_INIT           0x82675251u  /* ASIC -> game */
#define ASIC_TX_KEY_INIT           0x80678267u  /* game -> ASIC */
#define ASIC_MAX_WORDS             (IGS_E2100_BAR0_SIZE / 4)
#define ASIC_RUNTIME_PERIOD_MS     16
#define ASIC_REPLY_DELAY_MS        2
#define ASIC_BATCH_MAX             256  /* payload words per batch */
#define IGS_ASIC_STATIONS          6    /* message 6 has 6 coin/credit slots */
#define IGS_ASIC_PLAYERS           4

enum {
    ASIC_IDLE,          /* before the first ioctl 0x6600 */
    ASIC_HELLO_SENT,    /* waiting for the hello echo */
    ASIC_WAIT_KEYS,     /* waiting for the session keys */
    ASIC_HS_SENT,       /* sent 0xF0F0F0F0 ^ k, waiting for 0x0A0A0A0A */
    ASIC_DONE_SENT,     /* sent 0x90909090 ^ k, waiting for 0x0C0C0C0C */
    ASIC_RUNTIME_WAIT,  /* handshake done; waiting for the first batch */
    ASIC_RUNTIME,       /* message channel */
};

struct IGSE2100FPGAState {
    PCIDevice parent_obj;

    MemoryRegion bar0;
    MemoryRegion bar1;
    MemoryRegion bar3;
    QEMUTimer *irq_timer;
    QEMUTimer *asic_timer;
    uint8_t bar0_regs[IGS_E2100_BAR0_SIZE];
    uint8_t bar1_regs[IGS_E2100_BAR1_SIZE];
    uint8_t io_regs[IGS_E2100_BAR3_SIZE];
    uint32_t trace_count;
    uint32_t bar0_trace_count;
    uint32_t bar1_trace_count;
    uint8_t irq_stage;

    /* protection ASIC HLE */
    uint32_t mbox_out[IGS_E2100_MBOX_WORDS];  /* written by the host */
    uint32_t mbox_in[IGS_E2100_MBOX_WORDS];   /* read by the host */
    uint32_t asic_state;
    uint32_t session_key;
    uint32_t pending_in[2];       /* mailbox reply delivered by the timer */
    bool pending_mbox;
    uint8_t pending_status;
    uint32_t rx_key, tx_key;      /* ASIC->game, game->ASIC XOR keys */
    uint32_t rx_idx, tx_idx;      /* BAR1 header register indices */
    uint32_t running_sum;         /* shared rolling checksum */
    uint32_t batches_rx, batches_tx;
    uint32_t asic_log_count;
    uint32_t dma_log_count;

    /* cabinet I/O behind the ASIC */
    QemuInputHandlerState *input;
    bool grab_keyboard;
    uint8_t keys[10];             /* 80-bit button bitmap (message 2) */
    uint32_t coins[IGS_ASIC_STATIONS];    /* coins toward the next credit */
    uint32_t credits[IGS_ASIC_STATIONS];  /* games available */
    uint32_t coin_events[IGS_ASIC_STATIONS]; /* message-59 pop-ups owed */
    int64_t fps_mark;             /* virtual ms at the last 100-frame mark */
    bool credits_dirty;           /* message 6 owed */
    bool status_dirty;            /* message 52 owed */
};

/* ---- FPGA asset cipher (IGS_SOUND_CRYPTO.md) ---------------------------- */

static const uint8_t igs_cipher_mask[16] = {
    0xee, 0xa8, 0x54, 0x3c, 0x6a, 0x16, 0x83, 0x5d,
    0x09, 0x00, 0xd7, 0x73, 0x9f, 0x6b, 0xf7, 0x7f,
};
/* plaintext byte i comes from unrotated byte perm[i] */
static const uint8_t igs_cipher_perm[16] = {
    14, 10, 8, 1, 0, 12, 9, 4, 3, 6, 11, 7, 2, 13, 5, 15,
};
/* unrotated slots whose bit 7 is inverted: 1,4,6,7,10,12,13,15 */
static const uint16_t igs_cipher_flip = 0xb4d2;

static void igs_cipher_decrypt_block(uint8_t *blk)
{
    uint8_t d[16], u[16];
    uint64_t lo, hi;
    int i;

    for (i = 0; i < 16; i++) {
        d[i] = blk[i] ^ igs_cipher_mask[i];
    }
    lo = ror64(ldq_le_p(d), 1);
    hi = ror64(ldq_le_p(d + 8), 3);
    stq_le_p(u, lo);
    stq_le_p(u + 8, hi);
    for (i = 0; i < 16; i++) {
        if (igs_cipher_flip & (1u << i)) {
            u[i] ^= 0x80;
        }
    }
    for (i = 0; i < 16; i++) {
        blk[i] = u[igs_cipher_perm[i]];
    }
}

static void igs_cipher_encrypt_block(uint8_t *blk)
{
    uint8_t u[16];
    uint64_t lo, hi;
    int i;

    for (i = 0; i < 16; i++) {
        u[igs_cipher_perm[i]] = blk[i];
    }
    for (i = 0; i < 16; i++) {
        if (igs_cipher_flip & (1u << i)) {
            u[i] ^= 0x80;
        }
    }
    lo = rol64(ldq_le_p(u), 1);
    hi = rol64(ldq_le_p(u + 8), 3);
    stq_le_p(u, lo);
    stq_le_p(u + 8, hi);
    for (i = 0; i < 16; i++) {
        blk[i] = u[i] ^ igs_cipher_mask[i];
    }
}

/* ---- interrupts ---------------------------------------------------------- */

static void igs_e2100_update_irq(IGSE2100FPGAState *s)
{
    bool level = (s->io_regs[IGS_E2100_IRQ_STATUS] & 0x70) ||
                 (s->io_regs[IGS_E2100_IRQ_STATUS_ALT] & IGS_E2100_IRQ_EVENT);

    pci_set_irq(&s->parent_obj, level);
}

static void igs_e2100_schedule_irq(IGSE2100FPGAState *s)
{
    timer_mod(s->irq_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 16);
}

static void igs_e2100_raise_irq(IGSE2100FPGAState *s,
                                unsigned status_reg,
                                uint8_t bits)
{
    s->io_regs[status_reg] |= bits;
    igs_e2100_update_irq(s);
}

static void igs_e2100_irq_timer(void *opaque)
{
    IGSE2100FPGAState *s = opaque;

    /*
     * Power-on event: the driver's startup handshake needs both causes once.
     * The game's polling thread clears the event counter when it starts, so
     * this does not disturb the protection handshake that follows.
     */
    if (s->irq_stage == 0) {
        igs_e2100_raise_irq(s, IGS_E2100_IRQ_STATUS,
                            IGS_E2100_IRQ_EVENT | IGS_E2100_IRQ_GAME_TICK);
        s->irq_stage = 1;
    }
}

/* ---- protection ASIC HLE ------------------------------------------------- */

#define asic_log(s, fmt, ...) do {                                  \
        if ((s)->asic_log_count++ < 512) {                          \
            qemu_log("igs-e2100-asic: " fmt "\n", ## __VA_ARGS__);  \
        }                                                           \
    } while (0)

/*
 * Exactly one 0x40 event per host action: the game's polling thread only
 * dispatches when the kernel's event counter is exactly 1, so a second
 * event before it has been consumed would stall it forever.
 */
static void igs_asic_post(IGSE2100FPGAState *s, uint8_t status,
                          bool mbox, uint32_t w0, uint32_t w1,
                          int64_t delay_ms)
{
    s->pending_status = status;
    s->pending_mbox = mbox;
    s->pending_in[0] = w0;
    s->pending_in[1] = w1;
    timer_mod(s->asic_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + delay_ms);
}

static uint32_t igs_bar0_ld(IGSE2100FPGAState *s, uint32_t word)
{
    return ldl_le_p(s->bar0_regs + 4 * word);
}

static void igs_bar0_st(IGSE2100FPGAState *s, uint32_t word, uint32_t v)
{
    stl_le_p(s->bar0_regs + 4 * word, v);
}

static uint32_t igs_bar1_reg(IGSE2100FPGAState *s, uint32_t idx)
{
    return ldl_le_p(s->bar1_regs + 4 * (idx & 0x3f));
}

static void igs_bar1_set_reg(IGSE2100FPGAState *s, uint32_t idx, uint32_t v)
{
    stl_le_p(s->bar1_regs + 4 * (idx & 0x3f), v);
}

/*
 * Game -> ASIC batch (fish sub_8066450): header (payload << 7 | nmsg) in
 * BAR1 register tx_idx, body at BAR0 offset 0 XOR tx_key:
 * [checksum, payload..., msg headers...], checksum = running + sum(rest).
 */
static void igs_asic_receive_batch(IGSE2100FPGAState *s)
{
    uint32_t hdr = igs_bar1_reg(s, s->tx_idx);
    uint32_t nmsg = hdr & 0x7f;
    uint32_t payload = hdr >> 7;
    uint32_t total = nmsg + payload + 1;
    uint32_t words[ASIC_MAX_WORDS];
    uint32_t sum = 0, pos = 0, i;

    if (nmsg == 0 && payload == 0) {
        return;
    }
    if (total > ASIC_MAX_WORDS) {
        asic_log(s, "rx batch too large hdr=0x%08x", hdr);
        return;
    }
    for (i = 0; i < total; i++) {
        words[i] = igs_bar0_ld(s, i) ^ s->tx_key;
    }
    for (i = 1; i < total; i++) {
        sum += words[i];
    }
    if (words[0] != s->running_sum + sum) {
        asic_log(s, "rx checksum mismatch: got 0x%08x want 0x%08x "
                 "(idx=%u key=0x%08x hdr=0x%08x)", words[0],
                 s->running_sum + sum, s->tx_idx, s->tx_key, hdr);
    }
    s->running_sum = words[0];

    for (i = 0; i < nmsg; i++) {
        uint32_t mh = words[1 + payload + i];
        uint32_t type = mh & 0x7f, len = mh >> 7;
        uint32_t *p = &words[1 + pos];

        if (pos + len > payload) {
            asic_log(s, "rx message %u overruns payload", i);
            break;
        }
        if (s->batches_rx < 8 || type != 5) {
            asic_log(s, "rx msg type=%u len=%u w0=0x%08x", type, len,
                     len ? p[0] : 0);
        }
        /* type 5: words 11/12 announce the next header index and key */
        if (type == 5 && len >= 13) {
            s->tx_idx = p[11] & 0x3f;
            s->tx_key = p[12];
        }
        /* type 50: station p starts a game (sub_8072DF0) -> spend a credit */
        if (type == 50 && len >= 1 && p[0] < IGS_ASIC_STATIONS) {
            if (s->credits[p[0]]) {
                s->credits[p[0]]--;
            }
            s->credits_dirty = true;
            s->status_dirty = true;
            asic_log(s, "station %u starts a game, %u credits left",
                     p[0], s->credits[p[0]]);
        }
        /* type 58: game events [station, action, a, b]: 1 game over, 3 tickets */
        if (type == 58 && len >= 4 && p[1] != 0) {
            asic_log(s, "station %u event %u (%u, %u)", p[0], p[1], p[2], p[3]);
        }
        pos += len;
    }
    s->batches_rx++;
}

/*
 * ASIC -> game batch (fish sub_8066590).  An empty batch (header 0) is
 * valid and skips the checksum; it is what the HLE sends until the meaning
 * of the ASIC message types (inputs, settings) is mapped.
 */
static void igs_asic_send_batch(IGSE2100FPGAState *s,
                                const uint32_t *payload, uint32_t npayload,
                                const uint32_t *msg_hdrs, uint32_t nmsg)
{
    uint32_t sum = 0, i;

    igs_bar1_set_reg(s, s->rx_idx, (npayload << 7) | nmsg);
    if (nmsg == 0 && npayload == 0) {
        return;
    }
    for (i = 0; i < npayload; i++) {
        sum += payload[i];
    }
    for (i = 0; i < nmsg; i++) {
        sum += msg_hdrs[i];
    }
    s->running_sum += sum;
    igs_bar0_st(s, 0, s->running_sum ^ s->rx_key);
    for (i = 0; i < npayload; i++) {
        igs_bar0_st(s, 1 + i, payload[i] ^ s->rx_key);
    }
    for (i = 0; i < nmsg; i++) {
        igs_bar0_st(s, 1 + npayload + i, msg_hdrs[i] ^ s->rx_key);
    }
}

/*
 * Operator settings the real ASIC keeps in its NVRAM (message 55 ->
 * 0x8405A00).  Word 10 is a gameplay divisor (coin rate) and must not be 0;
 * words 7/0 are shown as a "%d ~ %d" range, word 8 as "%d tickets", words
 * 11..14 are per player.  Values are conservative defaults, not dumped ones.
 */
static const uint32_t igs_asic_settings[15] = {
    [0] = 10, [7] = 1, [8] = 1, [9] = 1, [10] = 1,
    [11] = 1, [12] = 1, [13] = 1, [14] = 1,
};

/*
 * Second operator settings block (message 10 -> dword_8406620, 50 words; the
 * game edits it in the setup menu and sends it back as message 9).  Only
 * word 17 is set:
 *   [17] Attract Sound (dword_8406664, sub_807CDA0): 0 = attract mode
 *        silent (the scene BGM plays at volume 0), 1..3 = sound every Nth
 *        attract cycle, >3 = always.
 * Everything else stays 0, as before this block was sent.
 */
static const uint32_t igs_asic_settings2[50] = {
    [17] = 4,
};

typedef struct IGSAsicBatch {
    uint32_t payload[ASIC_BATCH_MAX];
    uint32_t hdrs[16];
    uint32_t npayload, nhdrs;
} IGSAsicBatch;

static void igs_batch_add(IGSAsicBatch *b, uint32_t type,
                          const uint32_t *words, uint32_t len)
{
    if (b->nhdrs >= ARRAY_SIZE(b->hdrs) ||
        b->npayload + len > ARRAY_SIZE(b->payload)) {
        return;
    }
    memcpy(b->payload + b->npayload, words, len * 4);
    b->npayload += len;
    b->hdrs[b->nhdrs++] = (len << 7) | type;
}

static void igs_asic_timer(void *opaque)
{
    IGSE2100FPGAState *s = opaque;

    if (s->asic_state == ASIC_RUNTIME) {
        IGSAsicBatch b = { .npayload = 0 };
        uint32_t w[16], i;

        if (s->batches_tx == 0) {
            igs_batch_add(&b, 55, igs_asic_settings, ARRAY_SIZE(igs_asic_settings));
            igs_batch_add(&b, 10, igs_asic_settings2, ARRAY_SIZE(igs_asic_settings2));
            s->credits_dirty = s->status_dirty = true;
            asic_log(s, "tx settings (types 55, 10)");
        }

        /* message 2: the 80-bit button bitmap, every batch */
        memset(w, 0, sizeof(w));
        for (i = 0; i < ARRAY_SIZE(s->keys); i++) {
            w[i / 4] |= (uint32_t)s->keys[i] << (8 * (i % 4));
        }
        igs_batch_add(&b, 2, w, 3);

        /* message 59: one coin pop-up per station per batch */
        for (i = 0; i < IGS_ASIC_STATIONS; i++) {
            if (s->coin_events[i]) {
                s->coin_events[i]--;
                w[0] = i;
                igs_batch_add(&b, 59, w, 1);
            }
        }

        /* message 6: coins [0..5], credits [6..11], next header index [12] */
        if (s->credits_dirty) {
            for (i = 0; i < IGS_ASIC_STATIONS; i++) {
                w[i] = s->coins[i];
                w[6 + i] = s->credits[i];
            }
            w[12] = s->rx_idx;
            igs_batch_add(&b, 6, w, 13);
            s->credits_dirty = false;
        }

        /* message 52: per player [amount, status (0 = normal), value] */
        if (s->status_dirty) {
            for (i = 0; i < IGS_ASIC_PLAYERS; i++) {
                w[3 * i] = 0;
                w[3 * i + 1] = 0;
                w[3 * i + 2] = 1;
            }
            igs_batch_add(&b, 52, w, 3 * IGS_ASIC_PLAYERS);
            s->status_dirty = false;
        }

        igs_asic_send_batch(s, b.payload, b.npayload, b.hdrs, b.nhdrs);
        s->batches_tx++;
    }
    if (s->pending_mbox) {
        s->mbox_in[0] = s->pending_in[0];
        s->mbox_in[1] = s->pending_in[1];
    }
    igs_e2100_raise_irq(s, IGS_E2100_IRQ_STATUS, s->pending_status);
}

static void igs_asic_reset(IGSE2100FPGAState *s)
{
    asic_log(s, "reset (ioctl 0x6600)");
    timer_del(s->asic_timer);
    s->asic_state = ASIC_HELLO_SENT;
    s->session_key = 0;
    s->rx_key = ASIC_RX_KEY_INIT;
    s->tx_key = ASIC_TX_KEY_INIT;
    s->rx_idx = 0;
    s->tx_idx = 0;
    s->running_sum = 0;
    s->batches_rx = s->batches_tx = 0;
    /* ASIC boot hello; the game echoes it without checking the content. */
    igs_asic_post(s, IGS_E2100_IRQ_EVENT | IGS_E2100_IRQ_GAME_TICK,
                  true, 0x1ec2e100, 0, 50);
}

static void igs_asic_notify(IGSE2100FPGAState *s)
{
    uint32_t k = s->session_key;
    uint32_t cmd = s->mbox_out[0];

    switch (s->asic_state) {
    case ASIC_IDLE:
        asic_log(s, "notify before reset ignored");
        break;
    case ASIC_HELLO_SENT:
        asic_log(s, "hello echo 0x%08x/0x%08x", s->mbox_out[0],
                 s->mbox_out[1]);
        s->asic_state = ASIC_WAIT_KEYS;
        igs_asic_post(s, IGS_E2100_IRQ_GAME_TICK, false, 0, 0,
                      ASIC_REPLY_DELAY_MS);
        break;
    case ASIC_WAIT_KEYS:
        /* host wrote (r ^ 0x08080808, r) and four session keys at 0x108 */
        s->session_key = k = s->mbox_out[2];
        asic_log(s, "session keys %08x %08x %08x %08x, using 0x%08x",
                 s->mbox_out[2], s->mbox_out[3], s->mbox_out[4],
                 s->mbox_out[5], k);
        s->asic_state = ASIC_HS_SENT;
        igs_asic_post(s, IGS_E2100_IRQ_GAME_TICK, true,
                      ASIC_CMD_HELLO ^ k, g_random_int() ^ k,
                      ASIC_REPLY_DELAY_MS);
        break;
    case ASIC_HS_SENT:
        if ((cmd ^ k) != HOST_REPLY_RETRY) {
            asic_log(s, "unexpected handshake reply 0x%08x", cmd ^ k);
        }
        /* skip the asic36.bin upload: report done straight away */
        s->asic_state = ASIC_DONE_SENT;
        igs_asic_post(s, IGS_E2100_IRQ_GAME_TICK, true,
                      ASIC_CMD_DONE ^ k, g_random_int() ^ k,
                      ASIC_REPLY_DELAY_MS);
        break;
    case ASIC_DONE_SENT:
        if ((cmd ^ k) != HOST_REPLY_DONE) {
            asic_log(s, "unexpected done reply 0x%08x", cmd ^ k);
        }
        asic_log(s, "handshake complete, runtime channel up");
        /*
         * This event only satisfies the game's wait(1) after the handshake.
         * No batch yet: the game has not installed its runtime callback and
         * its next sub_8066450 call overwrites the header register.
         */
        s->asic_state = ASIC_RUNTIME_WAIT;
        igs_asic_post(s, IGS_E2100_IRQ_GAME_TICK, false, 0, 0,
                      ASIC_REPLY_DELAY_MS);
        break;
    case ASIC_RUNTIME_WAIT:
        s->asic_state = ASIC_RUNTIME;
        /* fall through: this is the game's first (empty) runtime batch */
    case ASIC_RUNTIME:
        igs_asic_receive_batch(s);
        /* the game posts exactly one batch per frame: report its frame rate */
        if (s->batches_rx % 25 == 0) {
            int64_t now = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);

            if (s->fps_mark) {
                qemu_log("igs-e2100-fps: t=%" PRId64 " frames=%u fps=%.2f\n",
                         now, s->batches_rx, 25000.0 / (now - s->fps_mark));
            }
            s->fps_mark = now;
        }
        igs_asic_post(s, IGS_E2100_IRQ_GAME_TICK, false, 0, 0,
                      ASIC_RUNTIME_PERIOD_MS);
        break;
    }
}

/* ---- cabinet controls (host keyboard) ------------------------------------ */

/*
 * Button bitmap (message 2, decoded by sub_8065C00): station p owns bits
 * 8p+0..5 = up, down, right, left, bet, fire (per-player key table at
 * 0x82274E0 = [left, right, up, down, fire, bet]).  Bit 78 is the operator
 * Test button.  Coins are not bits: the ASIC counts them itself.
 */
#define IGS_KEY_UP      0
#define IGS_KEY_DOWN    1
#define IGS_KEY_RIGHT   2
#define IGS_KEY_LEFT    3
#define IGS_KEY_BET     4
#define IGS_KEY_FIRE    5
#define IGS_KEY_TEST    78
#define IGS_COIN        0x80    /* .bit flag: coin for station (bit & 7) */

static const struct {
    QKeyCode code;
    uint8_t bit;
} igs_keymap[] = {
    /* station 1: arrows, Ctrl fire, Alt bet, 5 coin */
    { Q_KEY_CODE_UP, IGS_KEY_UP },
    { Q_KEY_CODE_DOWN, IGS_KEY_DOWN },
    { Q_KEY_CODE_RIGHT, IGS_KEY_RIGHT },
    { Q_KEY_CODE_LEFT, IGS_KEY_LEFT },
    { Q_KEY_CODE_ALT, IGS_KEY_BET },
    { Q_KEY_CODE_CTRL, IGS_KEY_FIRE },
    { Q_KEY_CODE_5, IGS_COIN | 0 },
    /* station 2: R F G D, A fire, S bet, 6 coin */
    { Q_KEY_CODE_R, 8 + IGS_KEY_UP },
    { Q_KEY_CODE_F, 8 + IGS_KEY_DOWN },
    { Q_KEY_CODE_G, 8 + IGS_KEY_RIGHT },
    { Q_KEY_CODE_D, 8 + IGS_KEY_LEFT },
    { Q_KEY_CODE_S, 8 + IGS_KEY_BET },
    { Q_KEY_CODE_A, 8 + IGS_KEY_FIRE },
    { Q_KEY_CODE_6, IGS_COIN | 1 },
    /* station 3: I K L J, B fire, V bet, 7 coin */
    { Q_KEY_CODE_I, 16 + IGS_KEY_UP },
    { Q_KEY_CODE_K, 16 + IGS_KEY_DOWN },
    { Q_KEY_CODE_L, 16 + IGS_KEY_RIGHT },
    { Q_KEY_CODE_J, 16 + IGS_KEY_LEFT },
    { Q_KEY_CODE_V, 16 + IGS_KEY_BET },
    { Q_KEY_CODE_B, 16 + IGS_KEY_FIRE },
    { Q_KEY_CODE_7, IGS_COIN | 2 },
    /* station 4: keypad 8 2 6 4, keypad 0 fire, keypad . bet, 8 coin */
    { Q_KEY_CODE_KP_8, 24 + IGS_KEY_UP },
    { Q_KEY_CODE_KP_2, 24 + IGS_KEY_DOWN },
    { Q_KEY_CODE_KP_6, 24 + IGS_KEY_RIGHT },
    { Q_KEY_CODE_KP_4, 24 + IGS_KEY_LEFT },
    { Q_KEY_CODE_KP_DECIMAL, 24 + IGS_KEY_BET },
    { Q_KEY_CODE_KP_0, 24 + IGS_KEY_FIRE },
    { Q_KEY_CODE_8, IGS_COIN | 3 },
    /* operator */
    { Q_KEY_CODE_F2, IGS_KEY_TEST },
};

static void igs_asic_coin(IGSE2100FPGAState *s, unsigned station)
{
    /* coins to play = settings block 10, word 1, + 1 ("%da%d" display) */
    uint32_t per_credit = igs_asic_settings2[1] + 1;

    s->coin_events[station]++;
    if (++s->coins[station] >= per_credit) {
        s->coins[station] = 0;
        s->credits[station]++;
    }
    s->credits_dirty = true;
    asic_log(s, "coin at station %u: %u credits", station, s->credits[station]);
}

static void igs_input_event(DeviceState *dev, QemuConsole *src, InputEvent *evt)
{
    IGSE2100FPGAState *s = IGS_E2100_FPGA(dev);
    InputKeyEvent *key = evt->u.key.data;
    int qcode = qemu_input_key_value_to_qcode(key->key);
    unsigned i;

    for (i = 0; i < ARRAY_SIZE(igs_keymap); i++) {
        uint8_t bit = igs_keymap[i].bit;

        if (igs_keymap[i].code != qcode) {
            continue;
        }
        asic_log(s, "key qcode %d %s -> bit %u", qcode, key->down ? "down" : "up",
                 bit);
        if (bit & IGS_COIN) {
            if (key->down) {
                igs_asic_coin(s, bit & 7);
            }
        } else if (key->down) {
            s->keys[bit >> 3] |= 1 << (bit & 7);
        } else {
            s->keys[bit >> 3] &= ~(1 << (bit & 7));
        }
    }
}

static const QemuInputHandler igs_input_handler = {
    .name  = "IGS E2100 cabinet controls",
    .mask  = INPUT_EVENT_MASK_KEY,
    .event = igs_input_event,
};

/* ---- DMA engine ---------------------------------------------------------- */

static void igs_e2100_dma(IGSE2100FPGAState *s, uint8_t cmd)
{
    /* the command byte at +0x13 is the top byte of the length register */
    uint32_t len = ldl_le_p(s->io_regs + IGS_E2100_IO_DMA_LEN) & 0xffffff;
    uint32_t sram = ldl_le_p(s->io_regs + IGS_E2100_IO_DMA_SRAM);
    uint32_t desc = ldl_le_p(s->io_regs + IGS_E2100_IO_DMA_DESC);
    uint8_t mode = s->io_regs[IGS_E2100_IO_MODE];
    uint32_t d[2];
    hwaddr buf;
    uint32_t i;

    /*
     * io+0x1C points at a coherent descriptor: [0] buffer bus address,
     * [1] length | 0x80000000.  Refuse anything else rather than guess:
     * a wrong address here overwrites guest kernel memory.
     */
    dma_memory_read(&address_space_memory, desc, d, sizeof(d),
                    MEMTXATTRS_UNSPECIFIED);
    d[0] = le32_to_cpu(d[0]);
    d[1] = le32_to_cpu(d[1]);
    buf = d[0];
    if ((d[1] & 0x7fffffff) != len) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "igs-e2100-fpga: DMA descriptor mismatch desc=0x%x "
                      "[0x%08x 0x%08x] len=0x%x\n", desc, d[0], d[1], len);
        igs_e2100_raise_irq(s, IGS_E2100_IRQ_STATUS_ALT, IGS_E2100_IRQ_EVENT);
        return;
    }

    if (sram >= IGS_E2100_BAR0_SIZE) {
        sram = 0;
    }
    if (len > IGS_E2100_BAR0_SIZE - sram) {
        len = IGS_E2100_BAR0_SIZE - sram;
    }

    if (cmd == IGS_E2100_DMA_TO_SRAM) {
        dma_memory_read(&address_space_memory, buf, s->bar0_regs + sram, len,
                        MEMTXATTRS_UNSPECIFIED);
        /* crypto mode: 0x84/0x04 enable, low bits 1 = encrypt, 2 = decrypt */
        if ((mode & 0x84) && (mode & 3)) {
            for (i = 0; i + 16 <= len; i += 16) {
                if ((mode & 3) == 2) {
                    igs_cipher_decrypt_block(s->bar0_regs + sram + i);
                } else {
                    igs_cipher_encrypt_block(s->bar0_regs + sram + i);
                }
            }
        }
    } else if (cmd == IGS_E2100_DMA_FROM_SRAM) {
        dma_memory_write(&address_space_memory, buf, s->bar0_regs + sram, len,
                         MEMTXATTRS_UNSPECIFIED);
    } else {
        qemu_log_mask(LOG_UNIMP, "igs-e2100-fpga: DMA command 0x%02x\n", cmd);
    }

    if (s->dma_log_count++ < 64) {
        qemu_log("igs-e2100-fpga: dma cmd=0x%02x mode=0x%02x len=0x%x "
                 "sram=0x%x desc=0x%x buf=0x%" HWADDR_PRIx "\n",
                 cmd, mode, len, sram, desc, buf);
    }
    igs_e2100_raise_irq(s, IGS_E2100_IRQ_STATUS_ALT, IGS_E2100_IRQ_EVENT);
}

/* ---- BAR3 I/O ------------------------------------------------------------ */

static uint64_t igs_e2100_bar3_read(void *opaque,
                                    hwaddr addr,
                                    unsigned size)
{
    IGSE2100FPGAState *s = opaque;
    uint64_t value = 0;
    unsigned i;

    if (addr + size > IGS_E2100_BAR3_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "igs-e2100-fpga: invalid BAR3 read addr=0x%"
                      HWADDR_PRIx " size=%u\n", addr, size);
        return 0;
    }

    for (i = 0; i < size; i++) {
        value |= (uint64_t)s->io_regs[addr + i] << (i * 8);
    }
    if (s->trace_count++ < 128) {
        qemu_log("igs-e2100-fpga: io read +0x%02" HWADDR_PRIx
                 " size=%u -> 0x%" PRIx64 "\n", addr, size, value);
    }
    return value;
}

static void igs_e2100_bar3_write(void *opaque,
                                 hwaddr addr,
                                 uint64_t value,
                                 unsigned size)
{
    IGSE2100FPGAState *s = opaque;
    unsigned i;

    if (addr + size > IGS_E2100_BAR3_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "igs-e2100-fpga: invalid BAR3 write addr=0x%"
                      HWADDR_PRIx " size=%u value=0x%" PRIx64 "\n",
                      addr, size, value);
        return;
    }

    if (s->trace_count++ < 128) {
        qemu_log("igs-e2100-fpga: io write +0x%02" HWADDR_PRIx
                 " size=%u value=0x%" PRIx64 "\n", addr, size, value);
    }

    /* The driver acknowledges interrupt causes by writing them back. */
    if (addr == IGS_E2100_IRQ_STATUS ||
        addr == IGS_E2100_IRQ_STATUS_ALT) {
        s->io_regs[addr] &= ~(uint8_t)value;
        igs_e2100_update_irq(s);
        return;
    }

    if (addr == IGS_E2100_IO_ASIC_RESET && size == 1) {
        if (value & 1) {
            igs_asic_reset(s);
        }
        return;
    }
    if (addr == IGS_E2100_IO_NOTIFY && size == 1) {
        if (value & 1) {
            igs_asic_notify(s);
        }
        return;
    }
    if (addr == IGS_E2100_IO_EVENT_ACK && size == 1) {
        return;
    }

    for (i = 0; i < size; i++) {
        s->io_regs[addr + i] = value >> (i * 8);
    }

    /* only the byte write of the command starts a transfer, not outl(len) */
    if (addr == IGS_E2100_COMMAND && size == 1) {
        igs_e2100_dma(s, s->io_regs[IGS_E2100_COMMAND]);
    }
}

static const MemoryRegionOps igs_e2100_bar3_ops = {
    .read = igs_e2100_bar3_read,
    .write = igs_e2100_bar3_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
        .unaligned = true,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
        .unaligned = true,
    },
};

/* ---- BAR0: FPGA SRAM / message buffer ------------------------------------ */

static uint64_t igs_e2100_bar0_read(void *opaque,
                                    hwaddr addr,
                                    unsigned size)
{
    IGSE2100FPGAState *s = opaque;
    uint64_t value = 0;
    unsigned i;

    if (addr + size > IGS_E2100_BAR0_SIZE) {
        return 0;
    }
    for (i = 0; i < size; i++) {
        value |= (uint64_t)s->bar0_regs[addr + i] << (i * 8);
    }
    if (s->bar0_trace_count++ < 128) {
        qemu_log("igs-e2100-fpga: bar0 read +0x%05" HWADDR_PRIx
                 " size=%u -> 0x%" PRIx64 "\n", addr, size, value);
    }
    return value;
}

static void igs_e2100_bar0_write(void *opaque, hwaddr addr,
                                  uint64_t value, unsigned size)
{
    IGSE2100FPGAState *s = opaque;
    unsigned i;

    if (addr + size > IGS_E2100_BAR0_SIZE) {
        return;
    }
    for (i = 0; i < size; i++) {
        s->bar0_regs[addr + i] = value >> (i * 8);
    }
    if (s->bar0_trace_count++ < 128) {
        qemu_log("igs-e2100-fpga: bar0 write +0x%05" HWADDR_PRIx
                 " size=%u value=0x%" PRIx64 "\n", addr, size, value);
    }
}

static const MemoryRegionOps igs_e2100_bar0_ops = {
    .read = igs_e2100_bar0_read,
    .write = igs_e2100_bar0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4, .unaligned = true },
    .impl = { .min_access_size = 1, .max_access_size = 4, .unaligned = true },
};

/* ---- BAR1: header registers + mailbox ------------------------------------ */

static bool igs_e2100_is_mbox(hwaddr addr, unsigned size)
{
    return addr >= IGS_E2100_MBOX_BASE &&
           addr + size <= IGS_E2100_MBOX_BASE + 4 * IGS_E2100_MBOX_WORDS &&
           size == 4 && !(addr & 3);
}

static uint64_t igs_e2100_bar1_read(void *opaque,
                                    hwaddr addr,
                                    unsigned size)
{
    IGSE2100FPGAState *s = opaque;
    uint64_t value = 0;
    unsigned i;

    if (addr + size > IGS_E2100_BAR1_SIZE) {
        return 0;
    }
    if (igs_e2100_is_mbox(addr, size)) {
        value = s->mbox_in[(addr - IGS_E2100_MBOX_BASE) / 4];
    } else {
        for (i = 0; i < size; i++) {
            value |= (uint64_t)s->bar1_regs[addr + i] << (i * 8);
        }
    }
    if (s->bar1_trace_count++ < 256) {
        qemu_log("igs-e2100-fpga: bar1 read +0x%03" HWADDR_PRIx
                 " size=%u -> 0x%" PRIx64 "\n", addr, size, value);
    }
    return value;
}

static void igs_e2100_bar1_write(void *opaque,
                                 hwaddr addr,
                                 uint64_t value,
                                 unsigned size)
{
    IGSE2100FPGAState *s = opaque;
    unsigned i;

    if (addr + size > IGS_E2100_BAR1_SIZE) {
        return;
    }
    if (igs_e2100_is_mbox(addr, size)) {
        s->mbox_out[(addr - IGS_E2100_MBOX_BASE) / 4] = value;
    } else {
        for (i = 0; i < size; i++) {
            s->bar1_regs[addr + i] = value >> (i * 8);
        }
    }
    if (s->bar1_trace_count++ < 256) {
        qemu_log("igs-e2100-fpga: bar1 write +0x%03" HWADDR_PRIx
                 " size=%u value=0x%" PRIx64 "\n", addr, size, value);
    }
}

static const MemoryRegionOps igs_e2100_bar1_ops = {
    .read = igs_e2100_bar1_read,
    .write = igs_e2100_bar1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4, .unaligned = true },
    .impl = { .min_access_size = 1, .max_access_size = 4, .unaligned = true },
};

/* ---- device -------------------------------------------------------------- */

static void igs_e2100_fpga_reset(DeviceState *dev)
{
    IGSE2100FPGAState *s = IGS_E2100_FPGA(dev);

    memset(s->io_regs, 0, sizeof(s->io_regs));
    memset(s->bar0_regs, 0, sizeof(s->bar0_regs));
    memset(s->bar1_regs, 0, sizeof(s->bar1_regs));
    memset(s->mbox_in, 0, sizeof(s->mbox_in));
    memset(s->mbox_out, 0, sizeof(s->mbox_out));
    s->irq_stage = 0;
    s->asic_state = ASIC_IDLE;
    memset(s->keys, 0, sizeof(s->keys));
    memset(s->coins, 0, sizeof(s->coins));
    memset(s->credits, 0, sizeof(s->credits));
    memset(s->coin_events, 0, sizeof(s->coin_events));
    timer_del(s->asic_timer);
    pci_set_irq(&s->parent_obj, 0);
    igs_e2100_schedule_irq(s);
}

static void igs_e2100_fpga_realize(PCIDevice *pdev, Error **errp)
{
    IGSE2100FPGAState *s = IGS_E2100_FPGA(pdev);

    /* INTA#: pci_set_irq() indexes this one-based configuration field. */
    pdev->config[PCI_INTERRUPT_PIN] = 1;

    memory_region_init_io(&s->bar0, OBJECT(s), &igs_e2100_bar0_ops, s,
                          "igs-e2100-fpga-bar0", IGS_E2100_BAR0_SIZE);

    memory_region_init_io(&s->bar1, OBJECT(s), &igs_e2100_bar1_ops, s,
                          "igs-e2100-fpga-bar1", IGS_E2100_BAR1_SIZE);

    memory_region_init_io(&s->bar3, OBJECT(s), &igs_e2100_bar3_ops,
                          s, "igs-e2100-fpga-bar3",
                          IGS_E2100_BAR3_SIZE);

    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar0);
    pci_register_bar(pdev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar1);
    pci_register_bar(pdev, 3, PCI_BASE_ADDRESS_SPACE_IO, &s->bar3);

    s->irq_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                igs_e2100_irq_timer, s);
    s->asic_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, igs_asic_timer, s);
    igs_e2100_schedule_irq(s);

    /* the cabinet has no PC keyboard: route host keys to the controls */
    if (s->grab_keyboard) {
        s->input = qemu_input_handler_register(DEVICE(s), &igs_input_handler);
        qemu_input_handler_activate(s->input);
    }

    qemu_log("igs-e2100-fpga: realized PCI device 11aa:1770\n");
}

static void igs_e2100_fpga_exit(PCIDevice *pdev)
{
    IGSE2100FPGAState *s = IGS_E2100_FPGA(pdev);

    timer_free(s->irq_timer);
    timer_free(s->asic_timer);
    if (s->input) {
        qemu_input_handler_unregister(s->input);
    }
}

static const Property igs_e2100_fpga_properties[] = {
    DEFINE_PROP_BOOL("keyboard", IGSE2100FPGAState, grab_keyboard, true),
};

static const VMStateDescription vmstate_igs_e2100_fpga = {
    .name = TYPE_IGS_E2100_FPGA,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, IGSE2100FPGAState),
        VMSTATE_UINT8_ARRAY(io_regs, IGSE2100FPGAState,
                            IGS_E2100_BAR3_SIZE),
        VMSTATE_UINT8_ARRAY(bar0_regs, IGSE2100FPGAState,
                            IGS_E2100_BAR0_SIZE),
        VMSTATE_UINT8_ARRAY(bar1_regs, IGSE2100FPGAState,
                            IGS_E2100_BAR1_SIZE),
        VMSTATE_UINT32_ARRAY(mbox_out, IGSE2100FPGAState,
                             IGS_E2100_MBOX_WORDS),
        VMSTATE_UINT32_ARRAY(mbox_in, IGSE2100FPGAState,
                             IGS_E2100_MBOX_WORDS),
        VMSTATE_UINT32(asic_state, IGSE2100FPGAState),
        VMSTATE_UINT32(session_key, IGSE2100FPGAState),
        VMSTATE_UINT32(rx_key, IGSE2100FPGAState),
        VMSTATE_UINT32(tx_key, IGSE2100FPGAState),
        VMSTATE_UINT32(rx_idx, IGSE2100FPGAState),
        VMSTATE_UINT32(tx_idx, IGSE2100FPGAState),
        VMSTATE_UINT32(running_sum, IGSE2100FPGAState),
        VMSTATE_TIMER_PTR(asic_timer, IGSE2100FPGAState),
        VMSTATE_END_OF_LIST()
    },
};

static void igs_e2100_fpga_class_init(ObjectClass *klass,
                                      const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);

    pc->realize = igs_e2100_fpga_realize;
    pc->exit = igs_e2100_fpga_exit;
    pc->vendor_id = IGS_E2100_VENDOR_ID;
    pc->device_id = IGS_E2100_DEVICE_ID;
    pc->revision = 0x01;
    pc->class_id = PCI_CLASS_OTHERS;

    dc->desc = "IGS E2100 PCI/FPGA interface";
    dc->vmsd = &vmstate_igs_e2100_fpga;
    device_class_set_props(dc, igs_e2100_fpga_properties);
    device_class_set_legacy_reset(dc, igs_e2100_fpga_reset);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo igs_e2100_fpga_info = {
    .name = TYPE_IGS_E2100_FPGA,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(IGSE2100FPGAState),
    .class_init = igs_e2100_fpga_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { }
    },
};

static void igs_e2100_fpga_register_types(void)
{
    type_register_static(&igs_e2100_fpga_info);
}

type_init(igs_e2100_fpga_register_types)
