/*
 * LeapFrog LF1000 (MagicEyes Pollux) SoC emulation
 *
 * Based on platform.h, ic.h, and board sources from the LF1000 Linux 2.6.20
 * kernel tree released by LeapFrog under GPL.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "chardev/char-fe.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/sysbus.h"
#include "hw/core/loader.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "system/system.h"
#include "system/reset.h"
#include "target/arm/cpu-qom.h"
#include "target/arm/internals.h"
#include "target/arm/cpu.h"
#include "ui/console.h"
#include "qom/object.h"
#include <stdio.h>

/* -----------------------------------------------------------------------
 * Physical memory map (platform.h)
 * All peripherals live in a 0x20000-byte block starting at 0xC0000000.
 * The kernel accesses them via IO_ADDRESS() which maps to 0xFC000000.
 * We create a container at 0xC0000000 and alias it at 0xFC000000.
 * --------------------------------------------------------------------- */
#define LF1000_SDRAM_BASE 0x00000000
#define LF1000_SDRAM_SIZE (64 * MiB)

/* Peripheral container */
#define LF1000_PERIPH_BASE 0xC0000000
#define LF1000_PERIPH_SIZE 0x20000
#define LF1000_PERIPH_VIRT 0xFC000000 /* IO_ADDRESS() result */

/* Offsets within the peripheral container */
#define LF1000_DMA_OFF 0x0000
#define LF1000_IC_OFF 0x0800
#define LF1000_TIMER0_OFF 0x1800
#define LF1000_TIMER1_OFF 0x1880
#define LF1000_TIMER2_OFF 0x1900
#define LF1000_TIMER3_OFF 0x1980
#define LF1000_TIMER4_OFF 0x1A00
#define LF1000_DPC_OFF 0x3000
#define LF1000_MLC_OFF 0x4000
#define LF1000_ADC_OFF 0x5000
#define LF1000_SPI0_OFF 0x7800
#define LF1000_SDIO0_OFF 0x9800
#define LF1000_GPIO_OFF 0xA000
#define LF1000_PWM_OFF 0xC000
#define LF1000_AUDIO_OFF 0xD800
#define LF1000_I2C0_OFF 0xE000
#define LF1000_CLKPWR_OFF 0xF000
#define LF1000_RTC_OFF 0xF080
#define LF1000_GPIOCURRENT_OFF 0xF100
#define LF1000_UART0_OFF 0x16000
#define LF1000_UART1_OFF 0x16080
#define LF1000_UART2_OFF 0x16800
#define LF1000_UART3_OFF 0x16880 /* SYS_UART = UART3 */
#define LF1000_UDC_OFF 0x18000
#define LF1000_ALIVE_OFF 0x19000
#define LF1000_NAND_OFF 0x19800
#define LF1000_GA3D_OFF 0x1A000
#define LF1000_MCU_A_OFF 0x14800
#define LF1000_MCU_S_OFF 0x15000
#define LF1000_I2C1_OFF 0xE800
#define LF1000_IDCT_OFF 0xF800

/* 3DGE is separate at 0xE0000000 / virt 0xFE000000 */
#define LF1000_3DGE_BASE 0xE0000000
#define LF1000_3DGE_VIRT 0xFE000000
#define LF1000_3DGE_SIZE (8 * KiB)

/* IC register offsets from IC_OFF (ic.h) */
#define IC_INTMODEL 0x08
#define IC_INTMODEH 0x0C
#define IC_INTMASKL 0x10
#define IC_INTMASKH 0x14
#define IC_PRIORDER 0x18
#define IC_PRIMODE 0x1C
#define IC_INTPENDL 0x20
#define IC_INTPENDH 0x24

/* CLKPWR register offsets */
#define CLKPWR_CLKMODEREG 0x00
#define CLKPWR_PLLSETREG0 0x04
#define CLKPWR_PLLSETREG1 0x08

/* Precomputed PLL register values:
 * PLL0: p=1 m=233 s=4 -> ~393MHz
 * PLL1: p=9 m=49  s=0 -> 147MHz */
#define LF1000_PLL0_REG 0x0004e904
#define LF1000_PLL1_REG 0x00243100
/* CLKMODEREG: CPU uses PLL0 (bits 4:5 = 0), BCLK/PCLK use PLL0 */
#define LF1000_CLKMODE_REG 0x00000000

/* -----------------------------------------------------------------------
 * Interrupt Controller
 * --------------------------------------------------------------------- */
#define TYPE_LF1000_IC "lf1000-ic"
OBJECT_DECLARE_SIMPLE_TYPE(LF1000ICState, LF1000_IC)

struct LF1000ICState
{
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t intmodel;
    uint32_t intmodeh;
    uint32_t intmaskl;
    uint32_t intmaskh;
    uint32_t priorder;
    uint32_t primode;
    uint32_t intpendl;
    uint32_t intpendh;

    qemu_irq irq;
    qemu_irq fiq;
};

static void lf1000_ic_update(LF1000ICState *s)
{
    uint32_t activel = s->intpendl & ~s->intmaskl;
    uint32_t activeh = s->intpendh & ~s->intmaskh;

    qemu_set_irq(s->fiq, !!((activel & s->intmodel) |
                            (activeh & s->intmodeh)));
    qemu_set_irq(s->irq, !!((activel & ~s->intmodel) |
                            (activeh & ~s->intmodeh)));
}

static void lf1000_ic_set_irq(void *opaque, int irq, int level)
{
    LF1000ICState *s = opaque;
    uint32_t *pending = (irq < 32) ? &s->intpendl : &s->intpendh;
    uint32_t bit = 1u << (irq & 31);

    if (level)
    {
        *pending |= bit;
    }
    else
    {
        *pending &= ~bit;
    }
    lf1000_ic_update(s);
}

static uint64_t lf1000_ic_read(void *opaque, hwaddr offset, unsigned size)
{
    LF1000ICState *s = opaque;
    switch (offset)
    {
    case IC_INTMODEL:
        return s->intmodel;
    case IC_INTMODEH:
        return s->intmodeh;
    case IC_INTMASKL:
        return s->intmaskl;
    case IC_INTMASKH:
        return s->intmaskh;
    case IC_PRIORDER:
        return s->priorder;
    case IC_PRIMODE:
        return s->primode;
    case IC_INTPENDL:
        /* Hardware reports *masked* status here (pending AND enabled); the
         * kernel's get_irqnr demux relies on this so it never dispatches a
         * masked-but-pending source. */
        return s->intpendl & ~s->intmaskl;
    case IC_INTPENDH:
        return s->intpendh & ~s->intmaskh;
    default:
        return 0;
    }
}

static void lf1000_ic_write(void *opaque, hwaddr offset,
                            uint64_t val, unsigned size)
{
    LF1000ICState *s = opaque;
    switch (offset)
    {
    case IC_INTMODEL:
        s->intmodel = val;
        break;
    case IC_INTMODEH:
        s->intmodeh = val;
        break;
    case IC_INTMASKL:
        s->intmaskl = val;
        break;
    case IC_INTMASKH:
        s->intmaskh = val;
        break;
    case IC_PRIORDER:
        s->priorder = val;
        break;
    case IC_PRIMODE:
        s->primode = val;
        break;
    case IC_INTPENDL: /* read-only */
        break;
    case IC_INTPENDH: /* read-only */
        break;
    default:
        break;
    }
    lf1000_ic_update(s);
}

static const MemoryRegionOps lf1000_ic_ops = {
    .read = lf1000_ic_read,
    .write = lf1000_ic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void lf1000_ic_realize(DeviceState *dev, Error **errp)
{
    LF1000ICState *s = LF1000_IC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &lf1000_ic_ops, s,
                          "lf1000-ic", 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->fiq);
    qdev_init_gpio_in(dev, lf1000_ic_set_irq, 64);

    s->intmaskl = 0xFFFFFFFF;
    s->intmaskh = 0xFFFFFFFF;
}

static void lf1000_ic_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->realize = lf1000_ic_realize;
}

static const TypeInfo lf1000_ic_info = {
    .name = TYPE_LF1000_IC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LF1000ICState),
    .class_init = lf1000_ic_class_init,
};

/* -----------------------------------------------------------------------
 * UART — TX-only stub
 * --------------------------------------------------------------------- */
#define TYPE_LF1000_UART "lf1000-uart"
OBJECT_DECLARE_SIMPLE_TYPE(LF1000UARTState, LF1000_UART)

#define LF1000_UART_TX 0x00
#define LF1000_UART_TRSR 0x14
#define LF1000_TRSR_THRE (1 << 1)
#define LF1000_TRSR_TEMT (1 << 2)

struct LF1000UARTState
{
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    CharFrontend chr;
    qemu_irq irq;
    bool txint_armed;  /* TX-holding-empty interrupt enabled (IER bit 0x10) */
    bool rxint_armed;  /* RX data interrupt enabled (IER bit 0x20) */
    bool tx_pending;   /* latched TX-empty interrupt cause (cleared on IIR read) */
    uint8_t rx_byte;   /* last received byte */
    bool rx_pending;   /* RX data available */
};

/*
 * Native LF1000 UART register layout (see serial_reg-lf1000.h).  Both
 * DEBUG_LL / lightning-boot and the Linux 8250-lf1000 ttyS driver use these
 * offsets (the driver redefines the 16550 register names to them, accessed
 * byte-wide).  Key registers:
 *   0x08 TRSR  — line status: DR(0x01) THRE(0x02) TEMT(0x04)
 *   0x0c FSR   — FIFO status (count); 0 = empty
 *   0x10 THB   — transmit holding (TX)
 *   0x12 RHB   — receive holding (RX)
 *   0x18 IER/IIR (native intstatusreg): write = enables, read = pending.
 *        enables: THRI 0x10, RDI 0x20, MSI 0x80
 *        pending: THRI 0x01, RDI 0x02, ERR 0x04, MSI 0x08  (MASK_INT 0x0F)
 *
 * The ttyS driver is interrupt-driven.  Because we transmit instantly, the
 * TX-holding-empty condition is always true; to avoid an interrupt storm we
 * model the TX interrupt as a latched cause (`tx_pending`) that is *cleared on
 * IIR read* (acknowledge) — effectively edge-triggered.  It is re-latched when
 * the TX interrupt is enabled and after each byte is transmitted, so the driver
 * still gets an interrupt per drain cycle until it disables THRI (stop_tx).
 */
#define LF1000_UART_TRSR_REG 0x08
#define LF1000_UART_FSR      0x0c
#define LF1000_UART_THB      0x10
#define LF1000_UART_RHB      0x12
#define LF1000_UART_INTREG   0x18
#define LF1000_UART_TRSR     0x14 /* legacy alias; also reports THRE/TEMT */

#define UTRSR_DR   0x01
#define UTRSR_THRE 0x02
#define UTRSR_TEMT 0x04

#define UIER_THRI  0x10
#define UIER_RDI   0x20
#define UIER_MSI   0x80
#define UIIR_THRI  0x01
#define UIIR_RDI   0x02
#define UIIR_MASK  0x0F

static void lf1000_uart_update_irq(LF1000UARTState *s)
{
    int level = (s->tx_pending && s->txint_armed) ||
                (s->rx_pending && s->rxint_armed);
    qemu_set_irq(s->irq, level);
}

static uint64_t lf1000_uart_read(void *opaque, hwaddr offset, unsigned size)
{
    LF1000UARTState *s = opaque;

    switch (offset)
    {
    case LF1000_UART_TRSR_REG: /* 0x08 line status */
        return UTRSR_THRE | UTRSR_TEMT | (s->rx_pending ? UTRSR_DR : 0);
    case LF1000_UART_FSR: /* 0x0c FIFO status: empty */
        return 0;
    case LF1000_UART_RHB: /* 0x12 receive holding */
        s->rx_pending = false;
        lf1000_uart_update_irq(s);
        return s->rx_byte;
    case LF1000_UART_TRSR: /* 0x14 legacy status alias */
        return UTRSR_THRE | UTRSR_TEMT;
    case LF1000_UART_INTREG: /* 0x18 read = pending/enable bits; reading
                              * acknowledges (clears) the latched TX cause */
    {
        uint64_t r = (s->txint_armed ? UIER_THRI : 0) |
                     (s->rxint_armed ? UIER_RDI : 0) |
                     ((s->tx_pending && s->txint_armed) ? UIIR_THRI : 0) |
                     ((s->rx_pending && s->rxint_armed) ? UIIR_RDI : 0);
        s->tx_pending = false;
        lf1000_uart_update_irq(s);
        return r;
    }
    default:
        return 0;
    }
}

static void lf1000_uart_write(void *opaque, hwaddr offset,
                              uint64_t val, unsigned size)
{
    LF1000UARTState *s = opaque;

    switch (offset)
    {
    case LF1000_UART_THB: /* 0x10 transmit */
    {
        uint8_t ch = val & 0xFF;
        qemu_chr_fe_write_all(&s->chr, &ch, 1);
        /* The byte left instantly → TX holding empty again: re-latch the cause
         * so the driver gets another interrupt to send the next byte. */
        s->tx_pending = true;
        lf1000_uart_update_irq(s);
        return;
    }
    case LF1000_UART_INTREG: /* 0x18 write = interrupt enables (read-modify-write
                              * preserves the enable bits, so parse them here) */
        s->txint_armed = !!(val & UIER_THRI);
        s->rxint_armed = !!(val & UIER_RDI);
        if (s->txint_armed)
            s->tx_pending = true; /* THRE is empty as soon as TX int is enabled */
        lf1000_uart_update_irq(s);
        return;
    default:
        /* LCR(0)/UCON(2)/FCR(4)/MCR(6) config and SYNC_PENDCLR: ignore */
        return;
    }
}

static void lf1000_uart_rx(void *opaque, const uint8_t *buf, int size)
{
    LF1000UARTState *s = opaque;
    if (size > 0) {
        s->rx_byte = buf[0];
        s->rx_pending = true;
        lf1000_uart_update_irq(s);
    }
}

static int lf1000_uart_can_rx(void *opaque)
{
    LF1000UARTState *s = opaque;
    return s->rx_pending ? 0 : 1;
}

static const MemoryRegionOps lf1000_uart_ops = {
    .read = lf1000_uart_read,
    .write = lf1000_uart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void lf1000_uart_realize(DeviceState *dev, Error **errp)
{
    LF1000UARTState *s = LF1000_UART(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &lf1000_uart_ops, s,
                          "lf1000-uart", 0x80);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qemu_chr_fe_set_handlers(&s->chr, lf1000_uart_can_rx, lf1000_uart_rx,
                             NULL, NULL, s, NULL, true);
}

static Property lf1000_uart_properties[] = {
    DEFINE_PROP_CHR("chardev", LF1000UARTState, chr),
};

static void lf1000_uart_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->realize = lf1000_uart_realize;
    device_class_set_props_n(dc, lf1000_uart_properties,
                             ARRAY_SIZE(lf1000_uart_properties));
}

static const TypeInfo lf1000_uart_info = {
    .name = TYPE_LF1000_UART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LF1000UARTState),
    .class_init = lf1000_uart_class_init,
};

/* -----------------------------------------------------------------------
 * Peripheral stub — returns 0, eats writes
 * --------------------------------------------------------------------- */
static uint64_t stub_read(void *opaque, hwaddr offset, unsigned size)
{
    return 0;
}

static void stub_write(void *opaque, hwaddr offset,
                       uint64_t val, unsigned size)
{
}

static const MemoryRegionOps stub_ops = {
    .read = stub_read,
    .write = stub_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void add_stub(MemoryRegion *parent, const char *name,
                     hwaddr offset, uint64_t size)
{
    MemoryRegion *mr = g_new0(MemoryRegion, 1);
    memory_region_init_io(mr, NULL, &stub_ops, NULL, name, size);
    memory_region_add_subregion(parent, offset, mr);
}

/* -----------------------------------------------------------------------
 * MLC (Multi-Layer Controller) — RGB layer-0 framebuffer scan-out
 *
 * The MLC composites display layers from framebuffers in SDRAM.  We model
 * RGB layer 0 (the primary layer used by lightning-boot's bootsplash and the
 * Linux fb/app): MLCADDRESS0 = framebuffer base, MLCHSTRIDE0/VSTRIDE0 =
 * byte strides, MLCCONTROL0 = format (bits 31:16) + LAYERENB (bit 5).
 * Screen is 320x240.  Supported pixel formats: R5G6B5/B5G6R5 (2 bpp),
 * R8G8B8/B8G8R8 (3 bpp, e.g. bootsplash 0xC653), and X8R8G8B8/X8B8G8R8 (4 bpp).
 * --------------------------------------------------------------------- */
#define MLC_W 320
#define MLC_H 240

#define MLC_CONTROLT   0x00
#define MLC_SCREENSIZE 0x04
#define MLC_CONTROL0   0x24
#define MLC_HSTRIDE0   0x28
#define MLC_VSTRIDE0   0x2C
#define MLC_ADDRESS0   0x38
#define MLC_LAYERENB   (1u << 5)

typedef struct
{
    MemoryRegion iomem;
    QemuConsole *con;
    uint8_t *ram_ptr;
    uint64_t ram_size;
    uint32_t regs[0x400 / 4];
} LF1000MLCState;

static LF1000MLCState mlc_state;

static uint64_t mlc_read(void *opaque, hwaddr offset, unsigned size)
{
    LF1000MLCState *s = opaque;
    if (offset / 4 >= ARRAY_SIZE(s->regs))
        return 0;
    uint32_t v = s->regs[offset / 4];
    /* The DIRTYFLAG (bit 4) of the control registers is set by software to
     * request that new layer settings be latched; the hardware clears it once
     * applied (at vsync).  Report it clear so display update loops don't spin
     * or block waiting for the apply to complete. */
    if (offset == MLC_CONTROLT || offset == MLC_CONTROL0 ||
        offset == 0x58 /* MLCCONTROL1 */ || offset == 0x7C /* MLCCONTROL2 */)
        v &= ~(1u << 4);
    return v;
}

static void mlc_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    LF1000MLCState *s = opaque;
    if (offset / 4 < ARRAY_SIZE(s->regs))
        s->regs[offset / 4] = val;
}

static const MemoryRegionOps mlc_ops = {
    .read = mlc_read,
    .write = mlc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static inline uint32_t mlc_pixel(const uint8_t *p, int bpp, uint32_t fmt)
{
    uint8_t r, g, b;
    /* In the format code, the top nibble's high bit (0x8000) selects BGR order. */
    bool bgr = (fmt & 0x8000) != 0;
    if (bpp == 2) {
        uint16_t v = p[0] | (p[1] << 8); /* little-endian R5G6B5 */
        uint8_t c5a = (v >> 11) & 0x1f, c6 = (v >> 5) & 0x3f, c5b = v & 0x1f;
        uint8_t hi = (c5a << 3) | (c5a >> 2);
        uint8_t mid = (c6 << 2) | (c6 >> 4);
        uint8_t lo = (c5b << 3) | (c5b >> 2);
        if (bgr) { b = hi; g = mid; r = lo; } else { r = hi; g = mid; b = lo; }
    } else { /* 3 or 4 bytes per pixel */
        if (bgr) { b = p[0]; g = p[1]; r = p[2]; }
        else     { r = p[0]; g = p[1]; b = p[2]; }
    }
    return 0xff000000u | (r << 16) | (g << 8) | b;
}

static bool mlc_gfx_update(void *opaque)
{
    LF1000MLCState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    uint32_t ctrl = s->regs[MLC_CONTROL0 / 4];
    uint32_t addr = s->regs[MLC_ADDRESS0 / 4];
    int hstride = s->regs[MLC_HSTRIDE0 / 4];
    int vstride = s->regs[MLC_VSTRIDE0 / 4];
    uint32_t fmt = (ctrl >> 16) & 0xffff;
    uint8_t *dst = surface_data(surface);
    int dst_stride = surface_stride(surface);
    int x, y;

    if (hstride < 2 || hstride > 4)
        hstride = 2;
    if (vstride <= 0)
        vstride = hstride * MLC_W;

    for (y = 0; y < MLC_H; y++) {
        uint32_t *drow = (uint32_t *)(dst + y * dst_stride);
        for (x = 0; x < MLC_W; x++) {
            uint32_t px;
            if (!(ctrl & MLC_LAYERENB)) {
                px = 0xff000000u; /* layer off → black */
            } else {
                uint64_t off = (uint64_t)addr + (uint64_t)y * vstride +
                               (uint64_t)x * hstride;
                if (off + hstride <= s->ram_size)
                    px = mlc_pixel(s->ram_ptr + off, hstride, fmt);
                else
                    px = 0xff000000u;
            }
            drow[x] = px;
        }
    }
    qemu_console_update_full(s->con);
    return true;
}

static const GraphicHwOps mlc_gfx_ops = {
    .gfx_update = mlc_gfx_update,
};

/* -----------------------------------------------------------------------
 * DPC (display controller) — register file + periodic vertical-sync.
 *
 * The Brio render loop waits for vsync before drawing each frame by polling
 * DPCCTRL0._INTPEND (bit 10).  Our stub never set it, so the loop blocked
 * after the first frame (the H&S splash) and the menu was never drawn.  We
 * raise _INTPEND at ~60 Hz (real time); the driver/app acknowledges it by
 * writing the bit back (write-1-to-clear).
 * --------------------------------------------------------------------- */
#define DPC_CTRL0   0x08C
#define DPC_INTPEND (1u << 10)

typedef struct
{
    MemoryRegion iomem;
    QEMUTimer *timer;
    bool vsync;
    uint32_t regs[0x200 / 4];
} LF1000DPCState;

static LF1000DPCState dpc_state;

static uint64_t dpc_read(void *opaque, hwaddr offset, unsigned size)
{
    LF1000DPCState *s = opaque;
    uint32_t v = (offset / 4 < ARRAY_SIZE(s->regs)) ? s->regs[offset / 4] : 0;
    if (offset == DPC_CTRL0 && s->vsync)
        v |= DPC_INTPEND;
    return v;
}

static void dpc_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    LF1000DPCState *s = opaque;
    if (offset == DPC_CTRL0 && (val & DPC_INTPEND))
        s->vsync = false;               /* write-1-to-clear vsync ack */
    if (offset / 4 < ARRAY_SIZE(s->regs))
        s->regs[offset / 4] = (uint32_t)val & ~DPC_INTPEND;
}

static const MemoryRegionOps dpc_ops = {
    .read = dpc_read,
    .write = dpc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void dpc_vsync_tick(void *opaque)
{
    LF1000DPCState *s = opaque;
    s->vsync = true;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 16666666LL);
}

/* -----------------------------------------------------------------------
 * DMA controller (8 channels, periph 0x0000, IRQ 3)
 *
 * The Brio display/audio stack blits through this engine.  Per channel
 * (stride 0x80): SRCADDR@0x0, DSTADDR@0x4, LENGTH@0x8 (bytes-1, 16-bit),
 * MODE@0xC.  MODE bits: RUN(1<<19) INTENB(1<<18) INTPEND(1<<17, W1C) BUSY(1<<16);
 * source/dest IO+increment flags in the low bits.  CMDSPACE@0x2C reports FIFO
 * space (must be non-zero or the driver errors).  We complete transfers
 * instantly: on a RUN write we perform the memory-to-memory copy (the screen
 * blit), set INTPEND and raise IRQ 3; the driver's handler clears INTPEND via
 * a write with the INTPEND bit set.
 * --------------------------------------------------------------------- */
#define DMA_CHANNELS    8
#define DMA_CH_STRIDE   0x80
#define DMA_SRCADDR     0x00
#define DMA_DSTADDR     0x04
#define DMA_LENGTH      0x08
#define DMA_MODE        0x0C
#define DMA_CMDBUSY     0x28
#define DMA_CMDSPACE    0x2C
#define DMA_RUN         (1u << 19)
#define DMA_INTENB      (1u << 18)
#define DMA_INTPEND     (1u << 17)
#define DMA_BUSY        (1u << 16)

typedef struct
{
    uint32_t srcaddr, dstaddr, mode;
    uint16_t length;
    bool intpend;
    bool intenb;
} LF1000DMAChan;

typedef struct
{
    MemoryRegion iomem;
    qemu_irq irq;
    uint8_t *ram_ptr;
    uint64_t ram_size;
    LF1000DMAChan ch[DMA_CHANNELS];
} LF1000DMAState;

static LF1000DMAState dma_state;

static void dma_update_irq(LF1000DMAState *s)
{
    int level = 0, i;
    for (i = 0; i < DMA_CHANNELS; i++)
        if (s->ch[i].intpend && s->ch[i].intenb)
            level = 1;
    qemu_set_irq(s->irq, level);
}

static uint64_t dma_read(void *opaque, hwaddr offset, unsigned size)
{
    LF1000DMAState *s = opaque;
    int chan = offset / DMA_CH_STRIDE;
    uint32_t reg = offset % DMA_CH_STRIDE;
    if (chan >= DMA_CHANNELS)
        return 0;
    LF1000DMAChan *c = &s->ch[chan];
    switch (reg)
    {
    case DMA_SRCADDR: return c->srcaddr;
    case DMA_DSTADDR: return c->dstaddr;
    case DMA_LENGTH:  return c->length;
    case DMA_MODE:    /* BUSY always clear (instant); reflect INTPEND/INTENB */
        return (c->mode & ~(DMA_RUN | DMA_BUSY | DMA_INTPEND | DMA_INTENB)) |
               (c->intenb ? DMA_INTENB : 0) | (c->intpend ? DMA_INTPEND : 0);
    case DMA_CMDBUSY:  return 0;   /* command FIFO not busy */
    case DMA_CMDSPACE: return 7;   /* FIFO space (low 3 bits must be non-zero) */
    default: return 0;
    }
}

static void dma_run_transfer(LF1000DMAState *s, LF1000DMAChan *c)
{
    uint32_t len = (uint32_t)c->length + 1;
    uint64_t src = c->srcaddr, dst = c->dstaddr;
    /* Only memory-to-memory transfers (both ends in SDRAM) move data we can
     * scan out — that's the screen blit.  Peripheral transfers (audio FIFO,
     * etc.) target MMIO outside SDRAM and are harmlessly ignored. */
    if (src + len <= s->ram_size && dst + len <= s->ram_size)
        memmove(s->ram_ptr + dst, s->ram_ptr + src, len);
    c->intpend = true;
}

static void dma_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    LF1000DMAState *s = opaque;
    int chan = offset / DMA_CH_STRIDE;
    uint32_t reg = offset % DMA_CH_STRIDE;
    if (chan >= DMA_CHANNELS)
        return;
    LF1000DMAChan *c = &s->ch[chan];
    switch (reg)
    {
    case DMA_SRCADDR: c->srcaddr = val; break;
    case DMA_DSTADDR: c->dstaddr = val; break;
    case DMA_LENGTH:  c->length = val & 0xFFFF; break;
    case DMA_MODE:
        if (val & DMA_INTPEND)        /* write-1-to-clear acknowledge */
            c->intpend = false;
        c->intenb = !!(val & DMA_INTENB);
        c->mode = val & ~(DMA_INTPEND | DMA_BUSY);
        if (val & DMA_RUN)
            dma_run_transfer(s, c);
        dma_update_irq(s);
        break;
    default: break;
    }
}

static const MemoryRegionOps dma_ops = {
    .read = dma_read,
    .write = dma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* GPIO emulation — returns NAND_CART_DETECT_PIN (port A, pin 18) high so
 * the kernel's lf1000 NAND driver skips the cartridge NAND scan. */
#define GPIO_PORT_A_PAD_OFF 0x18 /* GPIOAPAD — input pad values for port A */
static uint64_t gpio_read(void *opaque, hwaddr offset, unsigned size)
{
    if (offset == GPIO_PORT_A_PAD_OFF)
        return (1u << 18); /* NAND_CART_DETECT_PIN=18, high = not detected */
    return 0;
}
static void gpio_write(void *opaque, hwaddr offset, uint64_t val, unsigned size) {}
static const MemoryRegionOps gpio_ops = {
    .read = gpio_read,
    .write = gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static uint64_t clkpwr_read(void *opaque, hwaddr offset, unsigned size)
{
    switch (offset)
    {
    case CLKPWR_CLKMODEREG:
        return LF1000_CLKMODE_REG;
    case CLKPWR_PLLSETREG0:
        return LF1000_PLL0_REG;
    case CLKPWR_PLLSETREG1:
        return LF1000_PLL1_REG;
    default:
        return 0;
    }
}

static void clkpwr_write(void *opaque, hwaddr offset,
                         uint64_t val, unsigned size)
{
    /* ignore writes — we provide fixed PLL values */
}

static const MemoryRegionOps clkpwr_ops = {
    .read = clkpwr_read,
    .write = clkpwr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* 100 Hz system timer — fires IRQ 4 (LF1000_TIMER0_IRQ) to advance jiffies */
static qemu_irq lf1000_timer0_irq;
static QEMUTimer *lf1000_sys_timer_qemu;

/*
 * LF1000 hardware timer emulation (5 timers @ 0x1800..0x1A80).
 *
 * Each timer is an up-counter clocked at TIMER_PLL(PLL1) / TIMER_DIV.
 * With PLL1 = 147 MHz, CLKDIV = 32 and SELTCLK = TCLK/1, TIMER_DIV = 32,
 * so the counter runs at 147 MHz / 32 = 4,593,750 Hz.  The counter auto-zeros
 * when it reaches tmrmatch (hardware behaviour described in core_func.c).
 *
 * This is essential: the kernel's system-tick handler (lf1000_timer_interrupt)
 * uses TIMER2 as an "intertick" timer.  At every tick IRQ it reads TIMER2's
 * count (≈ one TIMER_SYS_TICK = 45930), runs timer_tick() that many times, then
 * resets TIMER2 by writing TIMER0's count back into it.  A counter that ignores
 * the reset write (or free-runs off host time) yields a huge intertick_count and
 * tens of thousands of timer_tick() calls per IRQ — a livelock.  Honouring the
 * reset write and counting at the real rate keeps intertick_count ≈ 1 tick.
 */
#define LF1000_TIMER_RATE_HZ 4593750ULL /* PLL1(147MHz) / TIMER_DIV(32) */
#define TMR_CTRL_RUN     (1u << 3)
#define TMR_CTRL_INTPEND (1u << 5)
#define TMR_CTRL_LDCNT   (1u << 6)

typedef struct
{
    uint32_t count_base; /* counter value latched at base_ns */
    int64_t base_ns;     /* virtual time of last (re)base */
    uint32_t match;      /* tmrmatch — counter auto-zeros at this value */
    uint32_t control;    /* tmrcontrol */
    uint32_t clkenb;     /* tmrclkenb */
    uint32_t clkgen;     /* tmrclkgen */
} LF1000TimerState;

static LF1000TimerState lf_timers[5];

static uint32_t lf1000_timer_count(LF1000TimerState *t)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t elapsed = (now > t->base_ns) ? (uint64_t)(now - t->base_ns) : 0;
    uint64_t ticks = (elapsed * LF1000_TIMER_RATE_HZ) / 1000000000ULL;
    uint64_t c = (uint64_t)t->count_base + ticks;

    /* Hardware auto-zeros at match (period = match + 1).  A match of 0 or the
     * free-run sentinel 0xffffffff means "no wrap" (natural 32-bit rollover). */
    if (t->match != 0 && t->match != 0xffffffff)
        c %= ((uint64_t)t->match + 1);
    return (uint32_t)c;
}

static void lf1000_timer_rebase(LF1000TimerState *t, uint32_t value)
{
    t->count_base = value;
    t->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void lf1000_sys_timer_tick(void *opaque)
{
    qemu_irq_raise(lf1000_timer0_irq);
    timer_mod(lf1000_sys_timer_qemu,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 10000000LL);
}

static uint64_t timer_read(void *opaque, hwaddr offset, unsigned size)
{
    LF1000TimerState *t = &lf_timers[(uintptr_t)opaque];

    switch (offset)
    {
    case 0x00: /* TMRCOUNT — live counter (LDCNT latch is a no-op for us) */
        return lf1000_timer_count(t);
    case 0x04: /* TMRMATCH */
        return t->match;
    case 0x08: /* TMRCONTROL */
        return t->control;
    case 0x40: /* TMRCLKENB */
        return t->clkenb;
    case 0x44: /* TMRCLKGEN */
        return t->clkgen;
    default:
        return 0;
    }
}

static void timer_write(void *opaque, hwaddr offset,
                        uint64_t val, unsigned size)
{
    uintptr_t idx = (uintptr_t)opaque;
    LF1000TimerState *t = &lf_timers[idx];

    switch (offset)
    {
    case 0x00: /* TMRCOUNT — writing reloads the counter base */
        lf1000_timer_rebase(t, (uint32_t)val);
        break;
    case 0x04: /* TMRMATCH */
        /* Preserve the current count across a match change to avoid a jump. */
        lf1000_timer_rebase(t, lf1000_timer_count(t));
        t->match = (uint32_t)val;
        break;
    case 0x08: /* TMRCONTROL */
        t->control = (uint32_t)val;
        /* Writing INTPEND acknowledges the system-tick IRQ (clear_timer_irq). */
        if (idx == 0 && (val & TMR_CTRL_INTPEND) && lf1000_timer0_irq)
            qemu_irq_lower(lf1000_timer0_irq);
        break;
    case 0x40: /* TMRCLKENB */
        t->clkenb = (uint32_t)val;
        break;
    case 0x44: /* TMRCLKGEN */
        t->clkgen = (uint32_t)val;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps timer_ops = {
    .read = timer_read,
    .write = timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};
/*
 * LF1000 I2C controller emulation (channel 0 only).
 *
 * The driver is entirely interrupt-driven: every transaction step writes
 * OP_HOLD (bit 1) to IRQ_PEND and then blocks in wait_event_interruptible_timeout
 * waiting for IRQ 32.  Without a real device on the bus, the IRQ never fires
 * and every step times out (10 jiffies = 100 ms each).
 *
 * We fire IRQ 32 immediately whenever OP_HOLD is written, making all I2C
 * transactions complete without any delay.  IDSR reads return a plausible
 * CS42L52 chip-ID byte (0xE3) so the audio-codec probe succeeds.
 *
 * Register offsets (from include/asm-arm/arch-lf1000/i2c.h):
 *   0x000 ICCR   0x004 ICSR   0x008 IAR    0x00C IDSR
 *   0x010 QCNT   0x014 BURST  0x018 BCNT   0x01C BSTAT
 *   0x020 BINT   0x024 IRQ_PEND               0x100 CLKENB
 *
 * IRQ_PEND bits:  bit 1 = OP_HOLD (software sets to generate START/STOP)
 *                 bit 0 = PEND    (hardware sets when done; write-1-to-clear)
 */

#define I2C_ICSR 0x004
#define I2C_IDSR 0x00C
#define I2C_IRQ_PEND 0x024
#define I2C_OP_HOLD 1 /* bit index */
#define I2C_PEND 0    /* bit index */

typedef struct
{
    uint32_t regs[0x104 / 4];
    qemu_irq irq;
} LF1000I2CState;

static LF1000I2CState i2c0_state;

static uint64_t i2c_read(void *opaque, hwaddr offset, unsigned size)
{
    LF1000I2CState *s = opaque;
    /* IDSR reads return CS42L52 chip-ID — any non-negative byte satisfies the probe */
    if (offset == I2C_IDSR)
        return 0xE3;
    if (offset < sizeof(s->regs))
        return s->regs[offset / 4];
    return 0;
}

static void i2c_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    LF1000I2CState *s = opaque;
    if (offset < sizeof(s->regs))
        s->regs[offset / 4] = val;

    if (offset == I2C_IRQ_PEND)
    {
        if (val & (1u << I2C_OP_HOLD))
        {
            /*
             * start_stop_condition() wrote OP_HOLD to trigger a START/STOP.
             * OP_HOLD is self-clearing in hardware — consume it, set PEND,
             * and raise the IRQ.
             *
             * Important: use else-if for the PEND clear below.
             * start_stop_condition() does a read-modify-write; if PEND happened
             * to be set in the stored register when it read, the write value has
             * both OP_HOLD=1 and PEND=1.  We must NOT lower the IRQ in that
             * case — the IRQ handler's acknowledge write is distinguished by
             * having OP_HOLD=0 (we clear OP_HOLD from stored regs here, so the
             * handler reads OP_HOLD=0 and its write-back also has OP_HOLD=0).
             */
            s->regs[I2C_IRQ_PEND / 4] &= ~(1u << I2C_OP_HOLD);
            s->regs[I2C_IRQ_PEND / 4] |= (1u << I2C_PEND);
            qemu_irq_raise(s->irq);
        }
        else if (val & (1u << I2C_PEND))
        {
            /* IRQ handler write-1-to-clear PEND (OP_HOLD is 0) */
            s->regs[I2C_IRQ_PEND / 4] &= ~(1u << I2C_PEND);
            qemu_irq_lower(s->irq);
        }
    }
}

static const MemoryRegionOps i2c_ops = {
    .read = i2c_read,
    .write = i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * LF1000 ADC emulation
 * ADC IRQ 25, base 0xC0005000.  The audio driver calls adc_GetReading()
 * which starts a conversion and waits (no timeout) for IRQ 25.
 * We fire the IRQ immediately when ADCCON.ADEN (bit 0) is written.
 */
#define ADC_ADCCON 0x00
#define ADC_ADCDAT 0x04
#define ADC_ADCINTENB 0x08
#define ADC_ADCINTCLR 0x0C
#define ADC_ADCCLKENB 0x40
#define ADC_SIZE 0x44

typedef struct
{
    uint8_t mem[ADC_SIZE];
    qemu_irq irq;
} LF1000ADCState;

static LF1000ADCState adc_state;

static uint64_t adc_read(void *opaque, hwaddr offset, unsigned size)
{
    LF1000ADCState *s = opaque;
    if (offset == ADC_ADCDAT)
        return 512; /* mid-range value for volume/battery reads */
    uint64_t val = 0;
    if (offset + size <= ADC_SIZE)
        memcpy(&val, s->mem + offset, size);
    return val;
}

static void adc_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    LF1000ADCState *s = opaque;
    if (offset + size <= ADC_SIZE)
        memcpy(s->mem + offset, &val, size);
    if (offset == ADC_ADCCON && (val & 1u))
        qemu_irq_raise(s->irq); /* conversion done immediately */
    if (offset == ADC_ADCINTCLR)
        qemu_irq_lower(s->irq);
}

static const MemoryRegionOps adc_ops = {
    .read = adc_read,
    .write = adc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
};

/*
 * LF1000 SPI emulation (SPI0, IRQ 12, base 0xC0007800)
 * The DPC driver calls lf1000_spi_xfer() for LCD init; the SPI driver uses
 * wait_event_interruptible (no timeout) on intqueue, so we must fire IRQ 12
 * when SSPSPIDATA is written.
 *
 * SSPSPISTATE (offset 0x06) bit layout:
 *   [15]=IRQEENB  [14]=IRQWENB  [13]=IRQRENB  (r/w enable bits)
 *   [6]=IRQE  [5]=IRQW  [4]=IRQR               (w1c status bits)
 *   [2]=WFFEMPTY  [0]=RFFEMPTY                  (hw status, read-only)
 */
#define SPI_SSPSPIDATA 0x04
#define SPI_SSPSPISTATE 0x06
#define SPI_SIZE 0x50
#define SPI_WFFEMPTY (1u << 2)
#define SPI_RFFEMPTY (1u << 0)
#define SPI_IRQE (1u << 6)
#define SPI_IRQEENB (1u << 15)
#define SPI_IRQWENB (1u << 14)
#define SPI_IRQRENB (1u << 13)

typedef struct
{
    uint8_t mem[SPI_SIZE];
    uint16_t state; /* SSPSPISTATE — maintained separately for clarity */
    qemu_irq irq;
} LF1000SPIState;

static LF1000SPIState spi0_state;

static void spi_update_irq(LF1000SPIState *s)
{
    bool active = ((s->state & SPI_IRQEENB) && (s->state & SPI_IRQE)) ||
                  ((s->state & SPI_IRQWENB) && (s->state & (1u << 5))) ||
                  ((s->state & SPI_IRQRENB) && (s->state & (1u << 4)));
    if (active)
        qemu_irq_raise(s->irq);
    else
        qemu_irq_lower(s->irq);
}

static uint64_t spi_read(void *opaque, hwaddr offset, unsigned size)
{
    LF1000SPIState *s = opaque;
    if (offset == SPI_SSPSPISTATE)
        return s->state;
    if (offset == SPI_SSPSPIDATA)
    {
        s->state |= SPI_RFFEMPTY; /* RX FIFO empty after read */
        spi_update_irq(s);
        return 0; /* dummy received byte */
    }
    uint64_t val = 0;
    if (offset + size <= SPI_SIZE)
        memcpy(&val, s->mem + offset, size);
    return val;
}

static void spi_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    LF1000SPIState *s = opaque;
    if (offset + size <= SPI_SIZE)
        memcpy(s->mem + offset, &val, size);

    if (offset == SPI_SSPSPISTATE)
    {
        uint16_t w = (uint16_t)val;
        /* Enable bits [15:13]: normal r/w */
        s->state = (s->state & ~0xE000u) | (w & 0xE000u);
        /* Status bits [6:4]: write-1-to-clear */
        s->state &= ~(w & 0x0070u);
        /* WFFEMPTY and RFFEMPTY are read-only hardware status */
        spi_update_irq(s);
    }
    else if (offset == SPI_SSPSPIDATA)
    {
        /* Data written → transfer complete → RX has data, IRQE fires */
        s->state &= ~SPI_RFFEMPTY; /* RX FIFO has data */
        s->state |= SPI_IRQE;      /* transfer complete */
        spi_update_irq(s);
    }
}

static const MemoryRegionOps spi_ops = {
    .read = spi_read,
    .write = spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
};

/*
 * LF1000 NAND Controller emulation
 * Samsung K9F2G08U0A: 256MB, 2048+64 page, 128KB erase block, 8-bit bus
 *
 * Register map (at IO_ADDRESS(LF1000_NAND_BASE) = 0xFAC00000):
 *   0x00: NFDATA  - data register
 *   0x10: NFCMD   - command register
 *   0x18: NFADDR  - address register
 *
 * NFCONTROL at MCU_S_BASE + 0x74:
 *   bit 9: RnB (1=ready, 0=busy)
 *   bit 8: IRQENB
 *   bit 15: IRQPEND
 */

/* NAND geometry */
#define NAND_PAGE_SIZE 2048
#define NAND_OOB_SIZE 64
#define NAND_PAGE_TOTAL (NAND_PAGE_SIZE + NAND_OOB_SIZE)
#define NAND_PAGES_PER_BLK 64
#define NAND_NUM_BLOCKS 2048
#define NAND_TOTAL_PAGES (NAND_NUM_BLOCKS * NAND_PAGES_PER_BLK)

/* NAND commands */
#define NAND_CMD_READ0 0x00
#define NAND_CMD_READ1 0x30
#define NAND_CMD_READID 0x90
#define NAND_CMD_RESET 0xFF
#define NAND_CMD_ERASE1 0x60
#define NAND_CMD_ERASE2 0xD0
#define NAND_CMD_SEQIN 0x80
#define NAND_CMD_PAGEPROG 0x10
#define NAND_CMD_STATUS 0x70

/* Samsung K9F2G08U0A ID */
static const uint8_t nand_id_bytes[] = {0xEC, 0xDA, 0x10, 0x95, 0x44};

/*
 * Software Hamming ECC — matches the kernel's nand_ecc.c (no ECC_SMC).
 * Computes 3 ECC bytes for 256 bytes of data.
 * Kernel places 24 ECC bytes (8 sectors × 3) at OOB positions 40–63
 * (nand_oob_64 layout).
 */
static const uint8_t nand_ecc_precalc_table[] = {
    0x00,
    0x55,
    0x56,
    0x03,
    0x59,
    0x0c,
    0x0f,
    0x5a,
    0x5a,
    0x0f,
    0x0c,
    0x59,
    0x03,
    0x56,
    0x55,
    0x00,
    0x65,
    0x30,
    0x33,
    0x66,
    0x3c,
    0x69,
    0x6a,
    0x3f,
    0x3f,
    0x6a,
    0x69,
    0x3c,
    0x66,
    0x33,
    0x30,
    0x65,
    0x66,
    0x33,
    0x30,
    0x65,
    0x3f,
    0x6a,
    0x69,
    0x3c,
    0x3c,
    0x69,
    0x6a,
    0x3f,
    0x65,
    0x30,
    0x33,
    0x66,
    0x03,
    0x56,
    0x55,
    0x00,
    0x5a,
    0x0f,
    0x0c,
    0x59,
    0x59,
    0x0c,
    0x0f,
    0x5a,
    0x00,
    0x55,
    0x56,
    0x03,
    0x69,
    0x3c,
    0x3f,
    0x6a,
    0x30,
    0x65,
    0x66,
    0x33,
    0x33,
    0x66,
    0x65,
    0x30,
    0x6a,
    0x3f,
    0x3c,
    0x69,
    0x0c,
    0x59,
    0x5a,
    0x0f,
    0x55,
    0x00,
    0x03,
    0x56,
    0x56,
    0x03,
    0x00,
    0x55,
    0x0f,
    0x5a,
    0x59,
    0x0c,
    0x0f,
    0x5a,
    0x59,
    0x0c,
    0x56,
    0x03,
    0x00,
    0x55,
    0x55,
    0x00,
    0x03,
    0x56,
    0x0c,
    0x59,
    0x5a,
    0x0f,
    0x6a,
    0x3f,
    0x3c,
    0x69,
    0x33,
    0x66,
    0x65,
    0x30,
    0x30,
    0x65,
    0x66,
    0x33,
    0x69,
    0x3c,
    0x3f,
    0x6a,
    0x6a,
    0x3f,
    0x3c,
    0x69,
    0x33,
    0x66,
    0x65,
    0x30,
    0x30,
    0x65,
    0x66,
    0x33,
    0x69,
    0x3c,
    0x3f,
    0x6a,
    0x0f,
    0x5a,
    0x59,
    0x0c,
    0x56,
    0x03,
    0x00,
    0x55,
    0x55,
    0x00,
    0x03,
    0x56,
    0x0c,
    0x59,
    0x5a,
    0x0f,
    0x0c,
    0x59,
    0x5a,
    0x0f,
    0x55,
    0x00,
    0x03,
    0x56,
    0x56,
    0x03,
    0x00,
    0x55,
    0x0f,
    0x5a,
    0x59,
    0x0c,
    0x69,
    0x3c,
    0x3f,
    0x6a,
    0x30,
    0x65,
    0x66,
    0x33,
    0x33,
    0x66,
    0x65,
    0x30,
    0x6a,
    0x3f,
    0x3c,
    0x69,
    0x03,
    0x56,
    0x55,
    0x00,
    0x5a,
    0x0f,
    0x0c,
    0x59,
    0x59,
    0x0c,
    0x0f,
    0x5a,
    0x00,
    0x55,
    0x56,
    0x03,
    0x66,
    0x33,
    0x30,
    0x65,
    0x3f,
    0x6a,
    0x69,
    0x3c,
    0x3c,
    0x69,
    0x6a,
    0x3f,
    0x65,
    0x30,
    0x33,
    0x66,
    0x65,
    0x30,
    0x33,
    0x66,
    0x3c,
    0x69,
    0x6a,
    0x3f,
    0x3f,
    0x6a,
    0x69,
    0x3c,
    0x66,
    0x33,
    0x30,
    0x65,
    0x00,
    0x55,
    0x56,
    0x03,
    0x59,
    0x0c,
    0x0f,
    0x5a,
    0x5a,
    0x0f,
    0x0c,
    0x59,
    0x03,
    0x56,
    0x55,
    0x00,
};

static void nand_sw_ecc(const uint8_t *dat, uint8_t *ecc_code)
{
    uint8_t idx, reg1, reg2, reg3, tmp1, tmp2;
    int i;
    reg1 = reg2 = reg3 = 0;
    for (i = 0; i < 256; i++)
    {
        idx = nand_ecc_precalc_table[*dat++];
        reg1 ^= (idx & 0x3f);
        if (idx & 0x40)
        {
            reg3 ^= (uint8_t)i;
            reg2 ^= ~(uint8_t)i;
        }
    }
    tmp1 = (reg3 & 0x80) >> 0;
    tmp1 |= (reg2 & 0x80) >> 1;
    tmp1 |= (reg3 & 0x40) >> 1;
    tmp1 |= (reg2 & 0x40) >> 2;
    tmp1 |= (reg3 & 0x20) >> 2;
    tmp1 |= (reg2 & 0x20) >> 3;
    tmp1 |= (reg3 & 0x10) >> 3;
    tmp1 |= (reg2 & 0x10) >> 4;
    tmp2 = (reg3 & 0x08) << 4;
    tmp2 |= (reg2 & 0x08) << 3;
    tmp2 |= (reg3 & 0x04) << 3;
    tmp2 |= (reg2 & 0x04) << 2;
    tmp2 |= (reg3 & 0x02) << 2;
    tmp2 |= (reg2 & 0x02) << 1;
    tmp2 |= (reg3 & 0x01) << 1;
    tmp2 |= (reg2 & 0x01) << 0;
    ecc_code[0] = ~tmp1;
    ecc_code[1] = ~tmp2;
    ecc_code[2] = ((~reg1) << 2) | 0x03;
}

typedef enum
{
    NAND_STATE_IDLE,
    NAND_STATE_READ_ID,
    NAND_STATE_READ_ADDR,
    NAND_STATE_READ_DATA,
    NAND_STATE_ERASE_ADDR,
    NAND_STATE_PROGRAM_ADDR,
    NAND_STATE_PROGRAM_DATA,
    NAND_STATE_STATUS,
} NANDState;

typedef struct
{
    NANDState state;
    uint8_t cmd;
    uint8_t addr_bytes[5];
    int addr_idx;
    int col_addr;
    int row_addr;
    int data_idx;
    int id_idx;
    uint8_t status;

    /* NAND image */
    uint8_t *data;
    size_t size;
} LF1000NAND;

static LF1000NAND nand_state;

/*
 * After expanding the raw NAND image (inserting 0xFF OOB), compute Hamming ECC
 * for every non-erased page and embed it at nand_oob_64 positions 40–63.
 * This lets the kernel's NAND_ECC_SOFT layer verify reads without errors.
 */
static void nand_embed_ecc(void)
{
    for (size_t pg = 0; pg < NAND_TOTAL_PAGES; pg++)
    {
        uint8_t *data = nand_state.data + pg * NAND_PAGE_TOTAL;
        uint8_t *oob = data + NAND_PAGE_SIZE;

        /* skip fully-erased pages (all-0xFF data → ECC is 0xFF, matches OOB) */
        bool erased = true;
        for (int j = 0; j < NAND_PAGE_SIZE; j++)
        {
            if (data[j] != 0xFF)
            {
                erased = false;
                break;
            }
        }
        if (erased)
            continue;

        /* 8 sectors of 256 bytes; ECC at OOB bytes 40, 41, 42 ... 61, 62, 63 */
        for (int s = 0; s < 8; s++)
        {
            uint8_t ecc[3];
            nand_sw_ecc(data + s * 256, ecc);
            oob[40 + s * 3 + 0] = ecc[0];
            oob[40 + s * 3 + 1] = ecc[1];
            oob[40 + s * 3 + 2] = ecc[2];
        }
    }
}

static void nand_load_image(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f)
    {
        /* allocate blank (0xFF) NAND */
        nand_state.size = (size_t)NAND_TOTAL_PAGES * NAND_PAGE_TOTAL;
        nand_state.data = g_malloc(nand_state.size);
        memset(nand_state.data, 0xFF, nand_state.size);
        return;
    }
    fseek(f, 0, SEEK_END);
    size_t raw_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    /*
     * The NAND image we built is a flat raw image without OOB bytes.
     * We need to expand it to include OOB (set to 0xFF) for each page.
     */
    size_t num_pages = raw_size / NAND_PAGE_SIZE;
    if (raw_size % NAND_PAGE_SIZE)
        num_pages++;

    nand_state.size = (size_t)NAND_TOTAL_PAGES * NAND_PAGE_TOTAL;
    nand_state.data = g_malloc(nand_state.size);
    memset(nand_state.data, 0xFF, nand_state.size);

    uint8_t *raw = g_malloc(raw_size);
    if (fread(raw, 1, raw_size, f) != raw_size)
    {
        fprintf(stderr, "lf1000-nand: short read from %s\n", path);
    }
    fclose(f);

    /* Copy raw data into pages, leaving OOB as 0xFF */
    for (size_t i = 0; i < num_pages && i < NAND_TOTAL_PAGES; i++)
    {
        size_t src_off = i * NAND_PAGE_SIZE;
        size_t dst_off = i * NAND_PAGE_TOTAL;
        size_t copy_sz = NAND_PAGE_SIZE;
        if (src_off + copy_sz > raw_size)
            copy_sz = raw_size - src_off;
        memcpy(nand_state.data + dst_off, raw + src_off, copy_sz);
    }

    g_free(raw);
    fprintf(stderr, "lf1000-nand: loaded %zu bytes (%zu pages) from %s\n",
            raw_size, num_pages, path);

    /*
     * Inject the stock lightning-boot binary into NAND block 0.
     *
     * The stock binary has SELF_BOOTSTRAP: on power-on the LF1000 hardware
     * loads only the first NAND page (2048 bytes) into SDRAM, then bootstrap()
     * calls nand_read() to fetch pages 1-62 of block 0 into SDRAM at 0x800.
     * QEMU loads the stock binary from the filesystem directly, but NAND block 0
     * currently holds a debug build.  SELF_BOOTSTRAP then overwrites the stock
     * binary's code at 0x800+ with debug-binary fragments, corrupting execution.
     *
     * Fix: overwrite block 0 in the NAND buffer with the stock binary so that
     * SELF_BOOTSTRAP reads the correct continuation.  ECC is (re-)computed below.
     */
    const char *lb_path =
        "/home/puyodead1/Desktop/Leapfrog/didj/Firmware/bootstrap-LF_LF1000/lightning-boot.bin";
    FILE *lb_f = fopen(lb_path, "rb");
    if (lb_f)
    {
        size_t blk0_raw = (size_t)NAND_PAGES_PER_BLK * NAND_PAGE_SIZE;
        uint8_t *lb_raw = g_malloc(blk0_raw);
        memset(lb_raw, 0xFF, blk0_raw);
        size_t lb_read = fread(lb_raw, 1, blk0_raw, lb_f);
        (void)lb_read;
        fclose(lb_f);
        for (size_t i = 0; i < NAND_PAGES_PER_BLK; i++)
        {
            size_t dst = i * NAND_PAGE_TOTAL;
            memcpy(nand_state.data + dst, lb_raw + i * NAND_PAGE_SIZE, NAND_PAGE_SIZE);
            memset(nand_state.data + dst + NAND_PAGE_SIZE, 0xFF, NAND_OOB_SIZE);
        }
        g_free(lb_raw);
        fprintf(stderr, "lf1000-nand: injected stock lightning-boot into block 0\n");
    }

    /* Compute and embed software ECC so the kernel's NAND_ECC_SOFT layer passes */
    nand_embed_ecc();
    fprintf(stderr, "lf1000-nand: ECC embedded for all non-erased pages\n");
}

static uint8_t nand_read_byte(LF1000NAND *s)
{
    switch (s->state)
    {
    case NAND_STATE_READ_ID:
        if (s->id_idx < (int)sizeof(nand_id_bytes))
            return nand_id_bytes[s->id_idx++];
        return 0xFF;

    case NAND_STATE_READ_ADDR: /* extra CMD_READ0 before data output */
    case NAND_STATE_READ_DATA:
    {
        size_t page_off = (size_t)s->row_addr * NAND_PAGE_TOTAL;
        size_t byte_off = page_off + s->col_addr + s->data_idx;
        s->data_idx++;
        if (byte_off < s->size)
            return s->data[byte_off];
        return 0xFF;
    }

    case NAND_STATE_STATUS:
        return s->status;

    default:
        return 0xFF;
    }
}

static uint64_t lf1000_nand_read(void *opaque, hwaddr offset, unsigned size)
{
    LF1000NAND *s = (LF1000NAND *)opaque;
    if (offset == 0x00)
    {
        uint64_t result;
        if (size == 1)
        {
            result = nand_read_byte(s);
        }
        else
        {
            uint32_t val = 0;
            for (int i = 0; i < 4; i++)
                val |= ((uint32_t)nand_read_byte(s)) << (i * 8);
            result = val;
        }
        return result;
    }
    return 0xFF;
}

static void lf1000_nand_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    LF1000NAND *s = (LF1000NAND *)opaque;

    switch (offset)
    {
    case 0x10: /* NFCMD */
        s->cmd = val & 0xFF;
        switch (s->cmd)
        {
        case NAND_CMD_RESET:
            s->state = NAND_STATE_IDLE;
            s->status = 0xC0; /* READY | WP# (not write-protected) */
            break;
        case NAND_CMD_READID:
            s->state = NAND_STATE_READ_ID;
            s->id_idx = 0;
            s->addr_idx = 0;
            break;
        case NAND_CMD_READ0:
            s->state = NAND_STATE_READ_ADDR;
            s->addr_idx = 0;
            s->data_idx = 0;
            /* Keep col_addr and row_addr — new address bytes will update them */
            break;
        case NAND_CMD_READ1: /* 0x30 - second cycle of read */
            s->state = NAND_STATE_READ_DATA;
            s->data_idx = 0;
            break;
        case NAND_CMD_STATUS:
            s->state = NAND_STATE_STATUS;
            break;
        case NAND_CMD_ERASE1:
            s->state = NAND_STATE_ERASE_ADDR;
            s->addr_idx = 0;
            s->row_addr = 0;
            break;
        case NAND_CMD_ERASE2:
        {
            /* erase block containing row_addr */
            int block = s->row_addr / NAND_PAGES_PER_BLK;
            size_t off = (size_t)block * NAND_PAGES_PER_BLK * NAND_PAGE_TOTAL;
            size_t len = (size_t)NAND_PAGES_PER_BLK * NAND_PAGE_TOTAL;
            if (off + len <= s->size)
                memset(s->data + off, 0xFF, len);
            s->state = NAND_STATE_IDLE;
            s->status = 0xC0; /* READY | WP# (not write-protected) */
            break;
        }
        case NAND_CMD_SEQIN:
            s->state = NAND_STATE_PROGRAM_ADDR;
            s->addr_idx = 0;
            s->col_addr = 0;
            s->row_addr = 0;
            break;
        case NAND_CMD_PAGEPROG:
        {
            /* commit program buffer — already written in PROGRAM_DATA */
            s->state = NAND_STATE_IDLE;
            s->status = 0xC0; /* READY | WP# (not write-protected) */
            break;
        }
        default:
            break;
        }
        break;

    case 0x18: /* NFADDR */
        val &= 0xFF;
        switch (s->state)
        {
        case NAND_STATE_READ_ADDR:
            /* 5 address cycles: col_lo, col_hi, row0, row1, row2 */
            if (s->addr_idx == 0)
                s->col_addr = val;
            else if (s->addr_idx == 1)
                s->col_addr |= (val << 8);
            else if (s->addr_idx == 2)
                s->row_addr = val;
            else if (s->addr_idx == 3)
                s->row_addr |= (val << 8);
            else if (s->addr_idx == 4)
                s->row_addr |= (val << 16);
            s->addr_idx++;
            break;
        case NAND_STATE_READ_ID:
            /* address 0x00 = manufacturer ID, just reset id_idx */
            s->id_idx = 0;
            break;
        case NAND_STATE_ERASE_ADDR:
            if (s->addr_idx == 0)
                s->row_addr = val;
            else if (s->addr_idx == 1)
                s->row_addr |= (val << 8);
            else if (s->addr_idx == 2)
                s->row_addr |= (val << 16);
            s->addr_idx++;
            break;
        case NAND_STATE_PROGRAM_ADDR:
            if (s->addr_idx == 0)
                s->col_addr = val;
            else if (s->addr_idx == 1)
                s->col_addr |= (val << 8);
            else if (s->addr_idx == 2)
                s->row_addr = val;
            else if (s->addr_idx == 3)
                s->row_addr |= (val << 8);
            else if (s->addr_idx == 4)
                s->row_addr |= (val << 16);
            s->addr_idx++;
            if (s->addr_idx == 5)
            {
                s->state = NAND_STATE_PROGRAM_DATA;
                s->data_idx = 0;
            }
            break;
        default:
            break;
        }
        break;

    case 0x00: /* NFDATA write */
        if (s->state == NAND_STATE_PROGRAM_DATA)
        {
            size_t page_off = (size_t)s->row_addr * NAND_PAGE_TOTAL;
            size_t byte_off = page_off + s->col_addr + s->data_idx;
            if (byte_off < s->size)
                s->data[byte_off] = val & 0xFF;
            s->data_idx++;
        }
        break;
    }
}

static const MemoryRegionOps lf1000_nand_ops = {
    .read = lf1000_nand_read,
    .write = lf1000_nand_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static uint64_t mcus_read(void *opaque, hwaddr offset, unsigned size)
{
    switch (offset)
    {
    case 0x074:          /* NFCONTROL (from base 0xC0015000) */
    case 0x874:          /* NFCONTROL (from base 0xC0015800, offset 0x74) */
        return (1 << 9); /* RnB=1 */
    case 0x08C:          /* NFECCSTATUS (from base 0xC0015000) */
    case 0x88C:          /* NFECCSTATUS (from base 0xC0015800, offset 0x8C) */
        return (1 << 1); /* NFECCDECDONE=1 */
    default:
        return 0;
    }
}

static void mcus_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    /* ignore writes */
}

static const MemoryRegionOps mcus_ops = {
    .read = mcus_read,
    .write = mcus_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

typedef struct
{
    ARMCPU *cpu;
    CPUARMState *env;
    QEMUTimer *fixup_timer;
} LF1000ResetData;

static void lf1000_sp_fixup(void *opaque)
{
    LF1000ResetData *d = opaque;
    CPUARMState *env = &d->cpu->env;
    uint32_t lb_stack = 0x00228000;
    env->banked_r13[bank_number(ARM_CPU_MODE_SVC)] = lb_stack;
    env->banked_r13[bank_number(ARM_CPU_MODE_ABT)] = lb_stack;
    env->banked_r13[bank_number(ARM_CPU_MODE_IRQ)] = lb_stack;
    env->banked_r13[bank_number(ARM_CPU_MODE_UND)] = lb_stack;
    env->banked_r13[bank_number(ARM_CPU_MODE_FIQ)] = lb_stack;
    env->banked_r13[bank_number(ARM_CPU_MODE_USR)] = lb_stack;
    env->regs[13] = lb_stack;
    fprintf(stderr, "lf1000_sp_fixup: SP fixed to 0x00228000\n");
}

static void lf1000_cpu_reset(void *opaque)
{
    LF1000ResetData *d = opaque;
    /* Schedule SP fixup to run after CPU reset completes */
    timer_mod(d->fixup_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1);
}

/* -----------------------------------------------------------------------
 * Machine init
 * --------------------------------------------------------------------- */
static void lf1000_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *ram = g_new(MemoryRegion, 1);
    qemu_irq cpu_irq, cpu_fiq;

    /* --- CPU: ARM926EJ-S -------------------------------------------- */
    Object *cpuobj = object_new(ARM_CPU_TYPE_NAME("arm926"));
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    ARMCPU *arm_cpu = ARM_CPU(cpuobj);
    CPUARMState *env = &arm_cpu->env;

    cpu_irq = qdev_get_gpio_in(DEVICE(arm_cpu), ARM_CPU_IRQ);
    cpu_fiq = qdev_get_gpio_in(DEVICE(arm_cpu), ARM_CPU_FIQ);

    /* --- SDRAM: 32 MB @ 0x00000000 ---------------------------------- */
    memory_region_init_ram(ram, NULL, "lf1000.sdram",
                           LF1000_SDRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, LF1000_SDRAM_BASE, ram);

    /* --- Peripheral container @ 0xC0000000 aliased @ 0xFC000000 ----- */
    MemoryRegion *periph = g_new0(MemoryRegion, 1);
    memory_region_init(periph, NULL, "lf1000-periph", LF1000_PERIPH_SIZE);
    memory_region_add_subregion(sysmem, LF1000_PERIPH_BASE, periph);

    MemoryRegion *periph_alias = g_new0(MemoryRegion, 1);
    memory_region_init_alias(periph_alias, NULL, "lf1000-periph-virt",
                             periph, 0, LF1000_PERIPH_SIZE);
    memory_region_add_subregion(sysmem, LF1000_PERIPH_VIRT, periph_alias);

    /* --- Interrupt controller --------------------------------------- */
    DeviceState *ic_dev = qdev_new(TYPE_LF1000_IC);
    SysBusDevice *ic_sbd = SYS_BUS_DEVICE(ic_dev);
    sysbus_realize_and_unref(ic_sbd, &error_fatal);
    memory_region_add_subregion(periph, LF1000_IC_OFF,
                                sysbus_mmio_get_region(ic_sbd, 0));
    sysbus_connect_irq(ic_sbd, 0, cpu_irq);
    sysbus_connect_irq(ic_sbd, 1, cpu_fiq);

    /* --- 100 Hz system timer → IRQ 4 (LF1000_TIMER0_IRQ) ----------- */
    lf1000_timer0_irq = qdev_get_gpio_in(ic_dev, 4);
    lf1000_sys_timer_qemu = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                         lf1000_sys_timer_tick, NULL);
    timer_mod(lf1000_sys_timer_qemu,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 10000000LL);

    /* --- UART0 @ offset 0x16000 (kernel console for CPU_LF1000) ----- */
    DeviceState *uart0_dev = qdev_new(TYPE_LF1000_UART);
    if (serial_hd(0))
    {
        qdev_prop_set_chr(uart0_dev, "chardev", serial_hd(0));
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(uart0_dev), &error_fatal);
    memory_region_add_subregion(periph, LF1000_UART0_OFF,
                                sysbus_mmio_get_region(
                                    SYS_BUS_DEVICE(uart0_dev), 0));
    /* UART0 → IRQ 10 (LF1000_UART0_IRQ): ttyS0 / userspace console TX+RX */
    sysbus_connect_irq(SYS_BUS_DEVICE(uart0_dev), 0,
                       qdev_get_gpio_in(ic_dev, 10));

    /* --- UART3 @ offset 0x16880 (no chardev needed) ----------------- */
    DeviceState *uart3_dev = qdev_new(TYPE_LF1000_UART);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(uart3_dev), &error_fatal);
    memory_region_add_subregion(periph, LF1000_UART3_OFF,
                                sysbus_mmio_get_region(
                                    SYS_BUS_DEVICE(uart3_dev), 0));

    MemoryRegion *clkpwr_mr = g_new0(MemoryRegion, 1);
    memory_region_init_io(clkpwr_mr, NULL, &clkpwr_ops, NULL,
                          "lf1000-clkpwr", 0x80);
    memory_region_add_subregion(periph, LF1000_CLKPWR_OFF, clkpwr_mr);

    for (int i = 0; i < 5; i++)
    {
        MemoryRegion *tmr = g_new0(MemoryRegion, 1);
        memory_region_init_io(tmr, NULL, &timer_ops,
                              (void *)(uintptr_t)i, "lf1000-timer", 0x80);
        memory_region_add_subregion(periph,
                                    LF1000_TIMER0_OFF + i * 0x80, tmr);
    }

    /* Load NAND image */
    memset(&nand_state, 0, sizeof(nand_state));
    nand_state.status = 0xC0; /* READY | WP# (not write-protected) */
    nand_load_image("/home/puyodead1/Desktop/Leapfrog/didj/Firmware/didj_nand.bin");

    /* MCU_S with NFCONTROL */
    MemoryRegion *mcus_mr = g_new0(MemoryRegion, 1);
    memory_region_init_io(mcus_mr, NULL, &mcus_ops, NULL, "lf1000-mcus", 0x1000);
    memory_region_add_subregion(periph, LF1000_MCU_S_OFF, mcus_mr);

    /* NAND controller @ 0xAC000000 / 0xFAC00000 */
    MemoryRegion *nand_mr = g_new0(MemoryRegion, 1);
    memory_region_init_io(nand_mr, NULL, &lf1000_nand_ops, &nand_state,
                          "lf1000-nand-ctrl", 0x20);
    memory_region_add_subregion(sysmem, 0xAC000000, nand_mr);

    MemoryRegion *nand_virt = g_new0(MemoryRegion, 1);
    memory_region_init_alias(nand_virt, NULL, "lf1000-nand-virt",
                             nand_mr, 0, 0x20);
    memory_region_add_subregion(sysmem, 0xFAC00000, nand_virt);

    /* --- Peripheral stubs (all as subregions of periph container) --- */
    /* DMA controller — performs the screen blits; IRQ 3 (LF1000_DMA_IRQ) */
    memset(&dma_state, 0, sizeof(dma_state));
    dma_state.ram_ptr = memory_region_get_ram_ptr(ram);
    dma_state.ram_size = LF1000_SDRAM_SIZE;
    dma_state.irq = qdev_get_gpio_in(ic_dev, 3);
    memory_region_init_io(&dma_state.iomem, NULL, &dma_ops, &dma_state,
                          "lf1000-dma", DMA_CHANNELS * DMA_CH_STRIDE);
    memory_region_add_subregion(periph, LF1000_DMA_OFF, &dma_state.iomem);
    /* DPC display controller with ~60 Hz vsync (DPCCTRL0._INTPEND) */
    memset(&dpc_state, 0, sizeof(dpc_state));
    memory_region_init_io(&dpc_state.iomem, NULL, &dpc_ops, &dpc_state,
                          "lf1000-dpc", 0x200);
    memory_region_add_subregion(periph, LF1000_DPC_OFF, &dpc_state.iomem);
    dpc_state.timer = timer_new_ns(QEMU_CLOCK_REALTIME, dpc_vsync_tick, &dpc_state);
    timer_mod(dpc_state.timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 16666666LL);

    /* MLC framebuffer — RGB layer 0 scanned out to a 320x240 console */
    memset(&mlc_state, 0, sizeof(mlc_state));
    mlc_state.ram_ptr = memory_region_get_ram_ptr(ram);
    mlc_state.ram_size = LF1000_SDRAM_SIZE;
    memory_region_init_io(&mlc_state.iomem, NULL, &mlc_ops, &mlc_state,
                          "lf1000-mlc", 0x400);
    memory_region_add_subregion(periph, LF1000_MLC_OFF, &mlc_state.iomem);
    mlc_state.con = qemu_graphic_console_create(NULL, 0, &mlc_gfx_ops,
                                                &mlc_state);
    qemu_console_resize(mlc_state.con, MLC_W, MLC_H);
    adc_state.irq = qdev_get_gpio_in(ic_dev, 25);
    MemoryRegion *adc_mr = g_new0(MemoryRegion, 1);
    memory_region_init_io(adc_mr, NULL, &adc_ops, &adc_state, "lf1000-adc", ADC_SIZE);
    memory_region_add_subregion(periph, LF1000_ADC_OFF, adc_mr);
    spi0_state.state = SPI_WFFEMPTY | SPI_RFFEMPTY; /* TX empty, RX empty */
    spi0_state.irq = qdev_get_gpio_in(ic_dev, 12);
    MemoryRegion *spi0_mr = g_new0(MemoryRegion, 1);
    memory_region_init_io(spi0_mr, NULL, &spi_ops, &spi0_state, "lf1000-spi0", SPI_SIZE);
    memory_region_add_subregion(periph, LF1000_SPI0_OFF, spi0_mr);
    add_stub(periph, "lf1000-sdio0", LF1000_SDIO0_OFF, 0x800);
    MemoryRegion *gpio_mr = g_new0(MemoryRegion, 1);
    memory_region_init_io(gpio_mr, NULL, &gpio_ops, NULL, "lf1000-gpio", 0x100);
    memory_region_add_subregion(periph, LF1000_GPIO_OFF, gpio_mr);
    add_stub(periph, "lf1000-pwm", LF1000_PWM_OFF, 0x50);
    add_stub(periph, "lf1000-audio", LF1000_AUDIO_OFF, 0x400);
    /* I2C0 — proper emulation so audio codec probe doesn't time out */
    i2c0_state.irq = qdev_get_gpio_in(ic_dev, 32);
    MemoryRegion *i2c0_mr = g_new0(MemoryRegion, 1);
    memory_region_init_io(i2c0_mr, NULL, &i2c_ops, &i2c0_state,
                          "lf1000-i2c0", 0x200);
    memory_region_add_subregion(periph, LF1000_I2C0_OFF, i2c0_mr);
    add_stub(periph, "lf1000-i2c1", LF1000_I2C1_OFF, 0x30);
    add_stub(periph, "lf1000-idct", LF1000_IDCT_OFF, 0x30);
    add_stub(periph, "lf1000-rtc", LF1000_RTC_OFF, 0x20);
    add_stub(periph, "lf1000-gpiocur", LF1000_GPIOCURRENT_OFF, 0x20);
    add_stub(periph, "lf1000-uart1", LF1000_UART1_OFF, 0x80);
    add_stub(periph, "lf1000-uart2", LF1000_UART2_OFF, 0x80);
    add_stub(periph, "lf1000-udc", LF1000_UDC_OFF, 0x900);
    add_stub(periph, "lf1000-alive", LF1000_ALIVE_OFF, 0x20);
    add_stub(periph, "lf1000-nand", LF1000_NAND_OFF, 0x100);
    add_stub(periph, "lf1000-ga3d", LF1000_GA3D_OFF, 0x2000);

    /* catch-all stub for entire peripheral space */
    MemoryRegion *periph_bg = g_new0(MemoryRegion, 1);
    memory_region_init_io(periph_bg, NULL, &stub_ops, NULL,
                          "lf1000-periph-bg", LF1000_PERIPH_SIZE);
    memory_region_add_subregion_overlap(periph, 0, periph_bg, -1);

    /* --- 3DGE separate block @ 0xE0000000 / virt 0xFE000000 --------- */
    MemoryRegion *tdge = g_new0(MemoryRegion, 1);
    memory_region_init_io(tdge, NULL, &stub_ops, NULL,
                          "lf1000-3dge", LF1000_3DGE_SIZE);
    memory_region_add_subregion(sysmem, LF1000_3DGE_BASE, tdge);

    MemoryRegion *tdge_alias = g_new0(MemoryRegion, 1);
    memory_region_init_alias(tdge_alias, NULL, "lf1000-3dge-virt",
                             tdge, 0, LF1000_3DGE_SIZE);
    memory_region_add_subregion(sysmem, LF1000_3DGE_VIRT, tdge_alias);

    /* --- Boot ------------------------------------------------------- */
    if (machine->kernel_filename)
    {
        /* Direct kernel boot */
        struct arm_boot_info *info = g_new0(struct arm_boot_info, 1);
        info->board_id = 1235;
        info->ram_size = LF1000_SDRAM_SIZE;
        info->loader_start = LF1000_SDRAM_BASE;
        arm_load_kernel(arm_cpu, machine, info);
    }
    else
    {
        /*
         * Boot lightning-boot loaded directly at 0x0.  This build has
         * SELF_BOOTSTRAP disabled (the source explicitly sanctions this when
         * forcing the image to address 0, as we do) but carries the original
         * cmdline including ubi.mtd=Brio ... so UBI attaches.
         */
        Error *err = NULL;
        const char *lb_path = "/home/puyodead1/Desktop/Leapfrog/didj/src-4222/lightning-boot/lightning-boot.bin";
        if (load_image_targphys(lb_path, 0x0, 0x10000, &err) < 0)
        {
            fprintf(stderr, "lf1000: failed to load lightning-boot\n");
        }
    }

    /* Register reset handler to set up banked SPs */
    LF1000ResetData *reset_data = g_new0(LF1000ResetData, 1);
    reset_data->cpu = arm_cpu;
    reset_data->env = env;
    reset_data->fixup_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, lf1000_sp_fixup, reset_data);
    qemu_register_reset(lf1000_cpu_reset, reset_data);
}

static void lf1000_machine_init(MachineClass *mc)
{
    mc->desc = "LeapFrog LF1000 (Didj / LeapPad)";
    mc->init = lf1000_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm926");
    mc->default_ram_size = LF1000_SDRAM_SIZE;
}
DEFINE_MACHINE_ARM("lf1000", lf1000_machine_init)

static void __attribute__((used)) lf1000_register_types(void)
{
    type_register_static(&lf1000_ic_info);
    type_register_static(&lf1000_uart_info);
}
type_init(lf1000_register_types)