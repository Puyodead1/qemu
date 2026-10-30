/*
 * Minimal Realtek RTL8169 gigabit Ethernet model
 *
 * Just enough of the chip for Linux's r8169 driver (as in 2.6.32): the
 * original 8169 MAC (TxConfig version bits 0), an always-up GMII PHY behind
 * PHYAR, C+ descriptor rings for TX and RX, and level INTx.  No checksum or
 * segmentation offload, VLAN, counters dump or migration.
 *
 * Written for the IGS E2100 guest, whose kernel has no other NIC driver.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/pci/pci_device.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/dma.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "net/net.h"
#include "qom/object.h"

#define TYPE_RTL8169 "rtl8169"
OBJECT_DECLARE_SIMPLE_TYPE(RTL8169State, RTL8169)

/* registers */
enum {
    MAC0            = 0x00,
    TxDescStartLo   = 0x20,
    TxDescStartHi   = 0x24,
    ChipCmd         = 0x37,
    TxPoll          = 0x38,
    IntrMask        = 0x3c,
    IntrStatus      = 0x3e,
    PHYAR           = 0x60,
    PHYstatus       = 0x6c,
    RxMaxSize       = 0xda,
    RxDescAddrLo    = 0xe4,
    RxDescAddrHi    = 0xe8,
};

/* ChipCmd */
#define CMD_RESET       0x10
#define CMD_RX_ENB      0x08
#define CMD_TX_ENB      0x04
/* TxPoll */
#define TXPOLL_NPQ      0x40
/* interrupts */
#define INT_RX_OK       0x0001
#define INT_TX_OK       0x0004
#define INT_RX_OVERFLOW 0x0010
#define INT_LINK_CHG    0x0020
/* PHYstatus */
#define PHY_FULLDUP     0x01
#define PHY_LINK        0x02
#define PHY_1000MF      0x10
/* descriptor opts1 */
#define DESC_OWN        0x80000000u
#define DESC_EOR        0x40000000u
#define DESC_FS         0x20000000u
#define DESC_LS         0x10000000u

#define RTL8169_TX_MAX  (16 * 1024)

struct RTL8169State {
    PCIDevice parent_obj;

    NICState *nic;
    NICConf conf;
    MemoryRegion bar_io;
    MemoryRegion bar_mem;

    uint8_t regs[256];
    uint16_t phy[32];
    uint32_t tx_idx, rx_idx;
    uint8_t txbuf[RTL8169_TX_MAX];
    uint32_t txlen;
};

static const uint16_t rtl8169_phy_reset_values[32] = {
    [0] = 0x1140,       /* BMCR: autoneg, full duplex, 1000 Mb/s */
    [1] = 0x796d,       /* BMSR: link up, autoneg complete */
    [2] = 0x001c,       /* PHYSID1/2: RTL8211 */
    [3] = 0xc912,
    [4] = 0x01e1,       /* ADVERTISE */
    [5] = 0x45e1,       /* LPA */
    [9] = 0x0300,       /* CTRL1000 */
    [10] = 0x3c00,      /* STAT1000 */
};

static uint16_t rtl8169_get16(RTL8169State *s, int addr)
{
    return lduw_le_p(&s->regs[addr]);
}

static uint64_t rtl8169_get64(RTL8169State *s, int lo)
{
    return ldl_le_p(&s->regs[lo]) | (uint64_t)ldl_le_p(&s->regs[lo + 4]) << 32;
}

static bool rtl8169_link_up(RTL8169State *s)
{
    return !qemu_get_queue(s->nic)->link_down;
}

static void rtl8169_update_irq(RTL8169State *s)
{
    uint16_t pending = rtl8169_get16(s, IntrStatus) & rtl8169_get16(s, IntrMask);

    pci_set_irq(PCI_DEVICE(s), pending != 0);
}

static void rtl8169_raise(RTL8169State *s, uint16_t bits)
{
    stw_le_p(&s->regs[IntrStatus], rtl8169_get16(s, IntrStatus) | bits);
    rtl8169_update_irq(s);
}

static void rtl8169_soft_reset(RTL8169State *s)
{
    uint8_t mac[6];

    /* the MAC address is reloaded from the EEPROM; everything else clears */
    memcpy(mac, &s->regs[MAC0], 6);
    memset(s->regs, 0, sizeof(s->regs));
    memcpy(&s->regs[MAC0], mac, 6);
    s->tx_idx = s->rx_idx = 0;
    s->txlen = 0;
    rtl8169_update_irq(s);
}

static void rtl8169_reset(DeviceState *dev)
{
    RTL8169State *s = RTL8169(dev);

    memcpy(&s->regs[MAC0], s->conf.macaddr.a, 6);
    rtl8169_soft_reset(s);
    memcpy(s->phy, rtl8169_phy_reset_values, sizeof(s->phy));
}

/* PHY access through PHYAR: bit 31 set = write (cleared when done),
 * clear = read (set when the data is valid). */
static void rtl8169_phyar_write(RTL8169State *s, uint32_t val)
{
    int reg = (val >> 16) & 0x1f;

    if (val & 0x80000000u) {
        uint16_t data = val & 0xffff;

        if (reg == 0 && (data & 0x8000)) {
            memcpy(s->phy, rtl8169_phy_reset_values, sizeof(s->phy));
        } else if (reg == 0) {
            s->phy[0] = data & ~0x0200;     /* autoneg restart self-clears */
        } else if (reg != 1 && reg != 2 && reg != 3) {
            s->phy[reg] = data;
        }
        stl_le_p(&s->regs[PHYAR], val & ~0x80000000u);
    } else {
        uint16_t data = s->phy[reg];

        if (reg == 1 && !rtl8169_link_up(s)) {
            data &= ~0x0004;
        }
        stl_le_p(&s->regs[PHYAR], 0x80000000u | reg << 16 | data);
    }
}

static void rtl8169_transmit(RTL8169State *s)
{
    PCIDevice *d = PCI_DEVICE(s);
    uint64_t ring = rtl8169_get64(s, TxDescStartLo);
    bool sent = false;
    int n;

    if (!(s->regs[ChipCmd] & CMD_TX_ENB)) {
        return;
    }
    for (n = 0; n < 1024; n++) {
        dma_addr_t desc = ring + s->tx_idx * 16;
        uint32_t opts1, len;
        uint64_t buf;

        pci_dma_read(d, desc, &opts1, 4);
        opts1 = le32_to_cpu(opts1);
        if (!(opts1 & DESC_OWN)) {
            break;
        }
        pci_dma_read(d, desc + 8, &buf, 8);
        buf = le64_to_cpu(buf);
        len = opts1 & 0xffff;
        if (opts1 & DESC_FS) {
            s->txlen = 0;
        }
        if (s->txlen + len <= sizeof(s->txbuf)) {
            pci_dma_read(d, buf, s->txbuf + s->txlen, len);
            s->txlen += len;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "rtl8169: oversized TX frame\n");
            s->txlen = sizeof(s->txbuf) + 1;
        }
        if ((opts1 & DESC_LS) && s->txlen <= sizeof(s->txbuf)) {
            qemu_send_packet(qemu_get_queue(s->nic), s->txbuf, s->txlen);
            s->txlen = 0;
        }
        opts1 = cpu_to_le32(opts1 & ~DESC_OWN);
        pci_dma_write(d, desc, &opts1, 4);
        s->tx_idx = (le32_to_cpu(opts1) & DESC_EOR) ? 0 : s->tx_idx + 1;
        sent = true;
    }
    if (sent) {
        rtl8169_raise(s, INT_TX_OK);
    }
}

static bool rtl8169_can_receive(NetClientState *nc)
{
    RTL8169State *s = qemu_get_nic_opaque(nc);

    return (s->regs[ChipCmd] & CMD_RX_ENB) != 0;
}

static ssize_t rtl8169_receive(NetClientState *nc, const uint8_t *buf,
                               size_t size)
{
    RTL8169State *s = qemu_get_nic_opaque(nc);
    PCIDevice *d = PCI_DEVICE(s);
    dma_addr_t desc;
    uint32_t opts1, bufsize, opts2 = 0;
    uint64_t addr;

    if (!(s->regs[ChipCmd] & CMD_RX_ENB)) {
        return size;
    }
    desc = rtl8169_get64(s, RxDescAddrLo) + s->rx_idx * 16;
    pci_dma_read(d, desc, &opts1, 4);
    opts1 = le32_to_cpu(opts1);
    if (!(opts1 & DESC_OWN)) {
        /* ring full: hold the packet until the driver re-arms IntrMask */
        rtl8169_raise(s, INT_RX_OVERFLOW);
        return 0;
    }
    bufsize = opts1 & 0x3fff;
    if (size + 4 > bufsize) {
        qemu_log_mask(LOG_GUEST_ERROR, "rtl8169: RX frame %zu > buffer %u\n",
                      size, bufsize);
        return size;
    }
    pci_dma_read(d, desc + 8, &addr, 8);
    pci_dma_write(d, le64_to_cpu(addr), buf, size);
    /* the length includes the 4-byte FCS, which r8169 strips */
    opts1 = (opts1 & DESC_EOR) | DESC_FS | DESC_LS | (uint32_t)(size + 4);
    s->rx_idx = (opts1 & DESC_EOR) ? 0 : s->rx_idx + 1;
    opts1 = cpu_to_le32(opts1);
    pci_dma_write(d, desc + 4, &opts2, 4);          /* no VLAN tag */
    pci_dma_write(d, desc, &opts1, 4);
    rtl8169_raise(s, INT_RX_OK);
    return size;
}

static void rtl8169_set_link_status(NetClientState *nc)
{
    rtl8169_raise(qemu_get_nic_opaque(nc), INT_LINK_CHG);
}

static uint8_t rtl8169_readb(RTL8169State *s, hwaddr addr)
{
    switch (addr) {
    case PHYstatus:
        return rtl8169_link_up(s) ? PHY_LINK | PHY_FULLDUP | PHY_1000MF : 0;
    case TxPoll:
        return 0;
    default:
        return s->regs[addr];
    }
}

static uint64_t rtl8169_read(void *opaque, hwaddr addr, unsigned size)
{
    RTL8169State *s = opaque;
    uint64_t val = 0;
    unsigned i;

    for (i = 0; i < size && addr + i < sizeof(s->regs); i++) {
        val |= (uint64_t)rtl8169_readb(s, addr + i) << (8 * i);
    }
    return val;
}

static void rtl8169_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size)
{
    RTL8169State *s = opaque;
    bool irq = false, flush = false;
    unsigned i;

    if (addr == PHYAR && size == 4) {
        rtl8169_phyar_write(s, val);
        return;
    }
    for (i = 0; i < size && addr + i < sizeof(s->regs); i++) {
        hwaddr a = addr + i;
        uint8_t b = val >> (8 * i);

        switch (a) {
        case ChipCmd:
            if (b & CMD_RESET) {
                rtl8169_soft_reset(s);
            } else {
                flush |= (b & ~s->regs[a] & CMD_RX_ENB) != 0;
                s->regs[a] = b & (CMD_RX_ENB | CMD_TX_ENB);
            }
            break;
        case TxPoll:
            if (b & TXPOLL_NPQ) {
                rtl8169_transmit(s);
            }
            break;
        case IntrStatus:
        case IntrStatus + 1:
            s->regs[a] &= ~b;                       /* write 1 to clear */
            irq = true;
            break;
        case IntrMask:
        case IntrMask + 1:
            s->regs[a] = b;
            irq = flush = true;
            break;
        case PHYstatus:
            break;
        case TxDescStartLo ... TxDescStartLo + 3:
            s->regs[a] = b;
            s->tx_idx = 0;
            break;
        case RxDescAddrLo ... RxDescAddrLo + 3:
            s->regs[a] = b;
            s->rx_idx = 0;
            break;
        default:
            s->regs[a] = b;
        }
    }
    if (irq) {
        rtl8169_update_irq(s);
    }
    if (flush) {
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
    }
}

static const MemoryRegionOps rtl8169_ops = {
    .read = rtl8169_read,
    .write = rtl8169_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static NetClientInfo net_rtl8169_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = rtl8169_can_receive,
    .receive = rtl8169_receive,
    .link_status_changed = rtl8169_set_link_status,
};

static void rtl8169_realize(PCIDevice *dev, Error **errp)
{
    RTL8169State *s = RTL8169(dev);
    DeviceState *d = DEVICE(dev);

    dev->config[PCI_INTERRUPT_PIN] = 1;
    memory_region_init_io(&s->bar_io, OBJECT(s), &rtl8169_ops, s,
                          "rtl8169", 0x100);
    memory_region_init_alias(&s->bar_mem, OBJECT(s), "rtl8169-mem",
                             &s->bar_io, 0, 0x100);
    /* r8169 wants the registers as MMIO in BAR 1 */
    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_IO, &s->bar_io);
    pci_register_bar(dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar_mem);

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&net_rtl8169_info, &s->conf,
                          object_get_typename(OBJECT(dev)), d->id,
                          &d->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static void rtl8169_exit(PCIDevice *dev)
{
    qemu_del_nic(RTL8169(dev)->nic);
}

static const VMStateDescription vmstate_rtl8169 = {
    .name = "rtl8169",
    .unmigratable = 1,
};

static const Property rtl8169_properties[] = {
    DEFINE_NIC_PROPERTIES(RTL8169State, conf),
};

static void rtl8169_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = rtl8169_realize;
    k->exit = rtl8169_exit;
    k->vendor_id = PCI_VENDOR_ID_REALTEK;
    k->device_id = 0x8169;
    k->revision = 0x10;
    k->class_id = PCI_CLASS_NETWORK_ETHERNET;
    device_class_set_legacy_reset(dc, rtl8169_reset);
    dc->vmsd = &vmstate_rtl8169;
    dc->desc = "Realtek RTL8169 Gigabit Ethernet (minimal)";
    device_class_set_props(dc, rtl8169_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo rtl8169_info = {
    .name          = TYPE_RTL8169,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(RTL8169State),
    .class_init    = rtl8169_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void rtl8169_register_types(void)
{
    type_register_static(&rtl8169_info);
}

type_init(rtl8169_register_types)
