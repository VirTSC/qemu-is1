/*
 * QEMU IDE Emulation: PCI PIIX3/4 support.
 *
 * Copyright (c) 2003 Fabrice Bellard
 * Copyright (c) 2006 Openedhand Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * References:
 *  [1] 82371FB (PIIX) AND 82371SB (PIIX3) PCI ISA IDE XCELERATOR,
 *      290550-002, Intel Corporation, April 1997.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/pci/pci.h"
#include "hw/ide/piix.h"
#include "hw/ide/pci.h"
#include "ide-internal.h"
#include "hw/core/qdev-properties.h"
#include "trace.h"

static uint64_t bmdma_read(void *opaque, hwaddr addr, unsigned size)
{
    BMDMAState *bm = opaque;
    uint32_t val;

    if (size != 1) {
        return ((uint64_t)1 << (size * 8)) - 1;
    }

    switch(addr & 3) {
    case 0:
        val = bm->cmd;
        break;
    case 2:
        val = bm->status;
        break;
    default:
        val = 0xff;
        break;
    }

    trace_bmdma_read(addr, val);
    return val;
}

static void bmdma_write(void *opaque, hwaddr addr,
                        uint64_t val, unsigned size)
{
    BMDMAState *bm = opaque;

    if (size != 1) {
        return;
    }

    trace_bmdma_write(addr, val);

    switch(addr & 3) {
    case 0:
        bmdma_cmd_writeb(bm, val);
        break;
    case 2:
        bmdma_status_writeb(bm, val);
        break;
    }
}

static const MemoryRegionOps piix_bmdma_ops = {
    .read = bmdma_read,
    .write = bmdma_write,
};

static void bmdma_setup_bar(PCIIDEState *d)
{
    int i;

    memory_region_init(&d->bmdma_bar, OBJECT(d), "piix-bmdma-container", 16);
    for(i = 0;i < 2; i++) {
        BMDMAState *bm = &d->bmdma[i];

        memory_region_init_io(&bm->extra_io, OBJECT(d), &piix_bmdma_ops, bm,
                              "piix-bmdma", 4);
        memory_region_add_subregion(&d->bmdma_bar, i * 8, &bm->extra_io);
        memory_region_init_io(&bm->addr_ioport, OBJECT(d),
                              &bmdma_addr_ioport_ops, bm, "bmdma", 4);
        memory_region_add_subregion(&d->bmdma_bar, i * 8 + 4, &bm->addr_ioport);
    }
}

static void piix_ide_reset(DeviceState *dev)
{
    PCIIDEState *d = PCI_IDE(dev);
    PCIDevice *pd = PCI_DEVICE(d);
    uint8_t *pci_conf = pd->config;
    int i;

    for (i = 0; i < 2; i++) {
        ide_bus_reset(&d->bus[i]);
    }

    /*
     * PCI command register default value (0000h) per [1, p.48].
     *
     * force_bus_master is deliberately NOT applied here: setting the bit at
     * reset is both too early - the firmware's own writes come later and clear
     * it - and wrong, because poking the register directly skips the
     * bus-master bookkeeping. pci_piix_ide_write_config() folds it into the
     * firmware's write instead.
     */
    pci_set_word(pci_conf + PCI_COMMAND, 0x0000);
    pci_set_word(pci_conf + PCI_STATUS,
                 PCI_STATUS_DEVSEL_MEDIUM | PCI_STATUS_FAST_BACK);
    pci_set_long(pci_conf + 0x20, 0x1);  /* BMIBA: 20-23h */
}

static bool pci_piix_init_bus(PCIIDEState *d, unsigned i, Error **errp)
{
    static const struct {
        int iobase;
        int iobase2;
        int isairq;
    } port_info[] = {
        {0x1f0, 0x3f6, 14},
        {0x170, 0x376, 15},
    };
    int ret;

    ide_bus_init(&d->bus[i], sizeof(d->bus[i]), DEVICE(d), i, 2);
    ret = ide_init_ioport(&d->bus[i], NULL, port_info[i].iobase,
                          port_info[i].iobase2);
    if (ret) {
        error_setg_errno(errp, -ret, "Failed to realize %s port %u",
                         object_get_typename(OBJECT(d)), i);
        return false;
    }
    ide_bus_init_output_irq(&d->bus[i], isa_get_irq(NULL, port_info[i].isairq));

    bmdma_init(&d->bus[i], &d->bmdma[i], d);
    ide_bus_register_restart_cb(&d->bus[i]);

    return true;
}

static void pci_piix_ide_realize(PCIDevice *dev, Error **errp)
{
    PCIIDEState *d = PCI_IDE(dev);
    uint8_t *pci_conf = dev->config;

    pci_conf[PCI_CLASS_PROG] = 0x80; // legacy ATA mode

    bmdma_setup_bar(d);
    pci_register_bar(dev, 4, PCI_BASE_ADDRESS_SPACE_IO, &d->bmdma_bar);

    for (unsigned i = 0; i < 2; i++) {
        if (!pci_piix_init_bus(d, i, errp)) {
            return;
        }
    }
}

static void pci_piix_ide_exitfn(PCIDevice *dev)
{
    PCIIDEState *d = PCI_IDE(dev);
    unsigned i;

    for (i = 0; i < 2; ++i) {
        memory_region_del_subregion(&d->bmdma_bar, &d->bmdma[i].extra_io);
        memory_region_del_subregion(&d->bmdma_bar, &d->bmdma[i].addr_ioport);
    }
}


static const Property piix_ide_properties[] = {
    DEFINE_PROP_BOOL("force-bus-master", PCIIDEState, force_bus_master, false),
};

/*
 * Keep PCI_COMMAND_MASTER set whatever the guest or its firmware writes. See
 * the comment on force_bus_master in include/hw/ide/pci.h.
 *
 * The bit is folded into the value BEFORE the write rather than poked into
 * the config space afterwards. Writing it afterwards sets the byte the guest
 * reads back - enough for FreeBSD to report WDMA2 - while bypassing the
 * bus-master bookkeeping pci_default_write_config() does for a write that
 * touches the command register. The device's DMA address space then stays
 * disabled, so every transfer silently reads zeroes and reports no error, and
 * the guest sees an empty disk.
 */
static void pci_piix_ide_write_config(PCIDevice *dev, uint32_t addr,
                                      uint32_t val, int len)
{
    PCIIDEState *d = PCI_IDE(dev);

    /* PCI_COMMAND_MASTER lives in the low byte of the command word. */
    if (d->force_bus_master && addr <= PCI_COMMAND && addr + len > PCI_COMMAND) {
        val |= (uint32_t)PCI_COMMAND_MASTER << ((PCI_COMMAND - addr) * 8);
    }

    pci_default_write_config(dev, addr, val, len);
}

/* NOTE: for the PIIX3, the IRQs and IOports are hardcoded */
static void piix3_ide_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, piix_ide_reset);
    dc->vmsd = &vmstate_ide_pci;
    k->realize = pci_piix_ide_realize;
    k->config_write = pci_piix_ide_write_config;
    k->exit = pci_piix_ide_exitfn;
    k->vendor_id = PCI_VENDOR_ID_INTEL;
    k->device_id = PCI_DEVICE_ID_INTEL_82371SB_1;
    k->class_id = PCI_CLASS_STORAGE_IDE;
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
    dc->hotpluggable = false;
    device_class_set_props(dc, piix_ide_properties);
}

static const TypeInfo piix3_ide_info = {
    .name          = TYPE_PIIX3_IDE,
    .parent        = TYPE_PCI_IDE,
    .class_init    = piix3_ide_class_init,
};

/* NOTE: for the PIIX4, the IRQs and IOports are hardcoded */
static void piix4_ide_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, piix_ide_reset);
    dc->vmsd = &vmstate_ide_pci;
    k->realize = pci_piix_ide_realize;
    k->config_write = pci_piix_ide_write_config;
    k->exit = pci_piix_ide_exitfn;
    k->vendor_id = PCI_VENDOR_ID_INTEL;
    k->device_id = PCI_DEVICE_ID_INTEL_82371AB;
    k->class_id = PCI_CLASS_STORAGE_IDE;
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
    dc->hotpluggable = false;
    device_class_set_props(dc, piix_ide_properties);
}

static const TypeInfo piix4_ide_info = {
    .name          = TYPE_PIIX4_IDE,
    .parent        = TYPE_PCI_IDE,
    .class_init    = piix4_ide_class_init,
};

static void piix_ide_register_types(void)
{
    type_register_static(&piix3_ide_info);
    type_register_static(&piix4_ide_info);
}

type_init(piix_ide_register_types)
