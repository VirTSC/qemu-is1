/*
 * QEMU i440FX PCI Bridge Emulation
 *
 * Copyright (c) 2006 Fabrice Bellard
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
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/range.h"
#include "hw/i386/pc.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_host.h"
#include "hw/pci-host/i440fx.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "qapi/visitor.h"
#include "qemu/error-report.h"
#include "qom/object.h"
#include "trace.h"

/*
 * I440FX chipset data sheet.
 * https://wiki.qemu.org/File:29054901.pdf
 */

OBJECT_DECLARE_SIMPLE_TYPE(I440FXState, I440FX_PCI_HOST_BRIDGE)

struct I440FXState {
    PCIHostState parent_obj;

    MemoryRegion *system_memory;
    MemoryRegion *io_memory;
    MemoryRegion *pci_address_space;
    MemoryRegion *ram_memory;
    Range pci_hole;
    uint64_t below_4g_mem_size;
    uint64_t above_4g_mem_size;
    uint64_t pci_hole64_size;
    bool pci_hole64_fix;

    char *pci_type;
};

#define I440FX_PAM      0x59
#define I440FX_PAM_SIZE 7
#define I440FX_SMRAM    0x72

/* Keep it 2G to comply with older win32 guests */
#define I440FX_PCI_HOST_HOLE64_SIZE_DEFAULT (1ULL << 31)

/* Older coreboot versions (4.0 and older) read a config register that doesn't
 * exist in real hardware, to get the RAM size from QEMU.
 */
#define I440FX_COREBOOT_RAM_SIZE 0x57

/*
 * AGP.
 *
 * The real 440FX has no AGP; this is a 440BX-shaped aperture bolted onto
 * it, enabled only with -global i440FX.agp=on, so the default pc machine's
 * config space and migration stream are untouched. It exists for guests whose AGP
 * driver will not hand out /dev/agpgart unless the host bridge advertises a
 * genuine aperture: FreeBSD 4.x's agp_intel probe tests class 06, subclass 00,
 * an AGP capability in the capability list, and then the device ID. The
 * device ID test has an "Intel Generic" fallback that accepts any Intel
 * vendor ID, so 8086:1237 matches as soon as the capability is there and the
 * ID does not have to be forged (which matters: SeaBIOS finds the i440FX by
 * device ID to set up PAM and PIIX interrupt routing).
 *
 * Registers are the 440BX ones agp_intel actually touches: APBASE (BAR0),
 * APSIZE, ATTBASE, AGPCTRL and ERRSTS. NBXCFG (0x50) needs nothing: config
 * space from 0x40 up is writable storage by default.
 *
 * The aperture is plain RAM, not a translation through the guest's GATT.
 * That is a deliberate deviation, and the reason is that the guest CPU
 * renders into the aperture: under KVM the window has to be a RAM memory slot
 * or every pixel becomes an MMIO exit, and neither an IOMMU memory region nor
 * a per-page alias fan-out can be that. So the aperture is the storage, the
 * pages the guest binds into the GATT are shadowed rather than used, and an
 * emulated bus-master reading an aperture address sees exactly what the CPU
 * wrote there. This is indistinguishable from real translation as long as the
 * guest reaches AGP memory only through the aperture - which is what
 * FreeBSD's agp(4) does, since agp_mmap() hands out aperture page numbers -
 * and as long as no two GATT entries alias the same page.
 *
 * APSIZE is writable because agp_intel probes the aperture-size mask by
 * writing 0x3f and reading it back, but the BAR keeps the size it was
 * realized with; shrinking APSIZE only shrinks the GATT the guest builds.
 */
#define I440FX_ERRSTS   0x91 /* 2 bytes, write-1-to-clear */
#define I440FX_AGPCTRL  0xb0
#define I440FX_APSIZE   0xb4
#define I440FX_ATTBASE  0xb8

#define I440FX_AGP_CAP        0xa0 /* where the 440BX puts it */
#define I440FX_AGP_CAP_SIZE   0x0c /* id/next/rev, AGPSTAT, AGPCMD */
#define I440FX_AGP_STATUS     0x04 /* offsets within the capability */
#define I440FX_AGP_COMMAND    0x08

#define I440FX_APSIZE_MASK    0x3f

/*
 * AGP 2.0, 1x and 2x, side-band addressing, 31-deep request queue: what a
 * 440BX reports. Nothing here reads it except agp_get_info()'s ai_mode and
 * agp_v2_enable()'s rate negotiation.
 */
#define I440FX_AGP_STATUS_VALUE 0x1f000203
#define I440FX_AGP_COMMAND_MASK 0x1f000317

#define I440FX_AGP_APERTURE_SIZE_DEFAULT (128 * MiB)

/* APSIZE encodes (size / 4 MiB - 1), inverted against the mask. */
static uint8_t i440fx_agp_apsize(uint64_t size)
{
    return ((size / (4 * MiB)) - 1) ^ I440FX_APSIZE_MASK;
}

static uint64_t i440fx_agp_aperture(PCIDevice *dev)
{
    uint8_t apsize = dev->config[I440FX_APSIZE] & I440FX_APSIZE_MASK;

    return ((uint64_t)((apsize ^ I440FX_APSIZE_MASK) + 1)) * 4 * MiB;
}

static void i440fx_agp_realize(PCIDevice *dev, Error **errp)
{
    ERRP_GUARD();
    PCII440FXState *f = I440FX_PCI_DEVICE(dev);
    uint64_t size = f->agp_aperture_size;

    if (!is_power_of_2(size) || size < 4 * MiB || size > 256 * MiB) {
        error_setg(errp, "agp-aperture-size must be a power of 2 "
                   "between 4 MiB and 256 MiB");
        return;
    }

    if (pci_add_capability(dev, PCI_CAP_ID_AGP, I440FX_AGP_CAP,
                           I440FX_AGP_CAP_SIZE, errp) < 0) {
        return;
    }
    dev->config[I440FX_AGP_CAP + 2] = 0x20; /* revision 2.0 */
    pci_set_long(dev->config + I440FX_AGP_CAP + I440FX_AGP_STATUS,
                 I440FX_AGP_STATUS_VALUE);
    pci_set_long(dev->wmask + I440FX_AGP_CAP + I440FX_AGP_COMMAND,
                 I440FX_AGP_COMMAND_MASK);

    memory_region_init_ram(&f->agp_aperture, OBJECT(dev),
                           "i440fx-agp-aperture", size, errp);
    if (*errp) {
        return;
    }
    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY |
                     PCI_BASE_ADDRESS_MEM_PREFETCH, &f->agp_aperture);

    dev->config[I440FX_APSIZE] = i440fx_agp_apsize(size);
    dev->wmask[I440FX_APSIZE] = I440FX_APSIZE_MASK;
    pci_set_long(dev->wmask + I440FX_ATTBASE, 0xfffff000);
    pci_set_long(dev->wmask + I440FX_AGPCTRL, 0x00003f80);
    /* wmask is 0xff from 0x40 up by default, and must not overlap w1cmask. */
    dev->wmask[I440FX_ERRSTS] = 0;
    dev->wmask[I440FX_ERRSTS + 1] = 0;
    dev->w1cmask[I440FX_ERRSTS] = 0xff;
    dev->w1cmask[I440FX_ERRSTS + 1] = 0xff;
}

static void i440fx_realize(PCIDevice *dev, Error **errp)
{
    PCII440FXState *f = I440FX_PCI_DEVICE(dev);

    dev->config[I440FX_SMRAM] = 0x02;

    if (object_property_get_bool(qdev_get_machine(), "iommu", NULL)) {
        warn_report("i440fx doesn't support emulated iommu");
    }

    if (f->agp) {
        i440fx_agp_realize(dev, errp);
    }
}

static void i440fx_update_memory_mappings(PCII440FXState *d)
{
    int i;
    PCIDevice *pd = PCI_DEVICE(d);

    memory_region_transaction_begin();
    for (i = 0; i < ARRAY_SIZE(d->pam_regions); i++) {
        pam_update(&d->pam_regions[i], i,
                   pd->config[I440FX_PAM + DIV_ROUND_UP(i, 2)]);
    }
    memory_region_set_enabled(&d->smram_region,
                              !(pd->config[I440FX_SMRAM] & SMRAM_D_OPEN));
    memory_region_set_enabled(&d->smram,
                              pd->config[I440FX_SMRAM] & SMRAM_G_SMRAME);
    memory_region_transaction_commit();
}


static void i440fx_write_config(PCIDevice *dev,
                                uint32_t address, uint32_t val, int len)
{
    PCII440FXState *d = I440FX_PCI_DEVICE(dev);

    /* XXX: implement SMRAM.D_LOCK */
    pci_default_write_config(dev, address, val, len);
    if (ranges_overlap(address, len, I440FX_PAM, I440FX_PAM_SIZE) ||
        range_covers_byte(address, len, I440FX_SMRAM)) {
        i440fx_update_memory_mappings(d);
    }
    if (d->agp && ranges_overlap(address, len, I440FX_AGPCTRL, 12)) {
        trace_i440fx_agp_config_write(address, val, len,
                                      pci_get_long(dev->config + I440FX_ATTBASE),
                                      i440fx_agp_aperture(dev));
    }
}

static int i440fx_post_load(void *opaque, int version_id)
{
    PCII440FXState *d = opaque;

    i440fx_update_memory_mappings(d);
    return 0;
}

static const VMStateDescription vmstate_i440fx = {
    .name = "I440FX",
    .version_id = 3,
    .minimum_version_id = 3,
    .post_load = i440fx_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, PCII440FXState),
        /* Used to be smm_enabled, which was basically always zero because
         * SeaBIOS hardly uses SMM.  SMRAM is now handled by CPU code.
         */
        VMSTATE_UNUSED(1),
        VMSTATE_END_OF_LIST()
    }
};

static void i440fx_pcihost_get_pci_hole_start(Object *obj, Visitor *v,
                                              const char *name, void *opaque,
                                              Error **errp)
{
    I440FXState *s = I440FX_PCI_HOST_BRIDGE(obj);
    uint64_t val64;
    uint32_t value;

    val64 = range_is_empty(&s->pci_hole) ? 0 : range_lob(&s->pci_hole);
    value = val64;
    assert(value == val64);
    visit_type_uint32(v, name, &value, errp);
}

static void i440fx_pcihost_get_pci_hole_end(Object *obj, Visitor *v,
                                            const char *name, void *opaque,
                                            Error **errp)
{
    I440FXState *s = I440FX_PCI_HOST_BRIDGE(obj);
    uint64_t val64;
    uint32_t value;

    val64 = range_is_empty(&s->pci_hole) ? 0 : range_upb(&s->pci_hole) + 1;
    value = val64;
    assert(value == val64);
    visit_type_uint32(v, name, &value, errp);
}

/*
 * The 64bit PCI hole start is set by the Guest firmware
 * as the address of the first 64bit PCI MEM resource.
 * If no PCI device has resources on the 64bit area,
 * the 64bit PCI hole will start after "over 4G RAM" and the
 * reserved space for memory hotplug if any.
 */
static uint64_t i440fx_pcihost_get_pci_hole64_start_value(Object *obj)
{
    PCIHostState *h = PCI_HOST_BRIDGE(obj);
    I440FXState *s = I440FX_PCI_HOST_BRIDGE(obj);
    Range w64;
    uint64_t value;

    pci_bus_get_w64_range(h->bus, &w64);
    value = range_is_empty(&w64) ? 0 : range_lob(&w64);
    if (!value && s->pci_hole64_fix) {
        value = pc_pci_hole64_start();
    }
    return value;
}

static void i440fx_pcihost_get_pci_hole64_start(Object *obj, Visitor *v,
                                                const char *name,
                                                void *opaque, Error **errp)
{
    PCIHostState *h = PCI_HOST_BRIDGE(obj);
    uint64_t hole64_start;

    if (!h->bus) {
        error_setg(errp, "PCI host bridge not realized");
        return;
    }
    hole64_start = i440fx_pcihost_get_pci_hole64_start_value(obj);
    visit_type_uint64(v, name, &hole64_start, errp);
}

/*
 * The 64bit PCI hole end is set by the Guest firmware
 * as the address of the last 64bit PCI MEM resource.
 * Then it is expanded to the PCI_HOST_PROP_PCI_HOLE64_SIZE
 * that can be configured by the user.
 */
static void i440fx_pcihost_get_pci_hole64_end(Object *obj, Visitor *v,
                                              const char *name, void *opaque,
                                              Error **errp)
{
    PCIHostState *h = PCI_HOST_BRIDGE(obj);
    I440FXState *s = I440FX_PCI_HOST_BRIDGE(obj);
    uint64_t hole64_start;
    Range w64;
    uint64_t value, hole64_end;

    if (!h->bus) {
        error_setg(errp, "PCI host bridge not realized");
        return;
    }
    hole64_start = i440fx_pcihost_get_pci_hole64_start_value(obj);
    pci_bus_get_w64_range(h->bus, &w64);
    value = range_is_empty(&w64) ? 0 : range_upb(&w64) + 1;
    hole64_end = ROUND_UP(hole64_start + s->pci_hole64_size, 1ULL << 30);
    if (s->pci_hole64_fix && value < hole64_end) {
        value = hole64_end;
    }
    visit_type_uint64(v, name, &value, errp);
}

static void i440fx_pcihost_initfn(Object *obj)
{
    I440FXState *s = I440FX_PCI_HOST_BRIDGE(obj);
    PCIHostState *phb = PCI_HOST_BRIDGE(obj);

    memory_region_init_io(&phb->conf_mem, obj, &pci_host_conf_le_ops, phb,
                          "pci-conf-idx", 4);
    memory_region_init_io(&phb->data_mem, obj, &pci_host_data_le_ops, phb,
                          "pci-conf-data", 4);

    object_property_add_link(obj, PCI_HOST_PROP_RAM_MEM, TYPE_MEMORY_REGION,
                             (Object **) &s->ram_memory,
                             qdev_prop_allow_set_link_before_realize, 0);

    object_property_add_link(obj, PCI_HOST_PROP_PCI_MEM, TYPE_MEMORY_REGION,
                             (Object **) &s->pci_address_space,
                             qdev_prop_allow_set_link_before_realize, 0);

    object_property_add_link(obj, PCI_HOST_PROP_SYSTEM_MEM, TYPE_MEMORY_REGION,
                             (Object **) &s->system_memory,
                             qdev_prop_allow_set_link_before_realize, 0);

    object_property_add_link(obj, PCI_HOST_PROP_IO_MEM, TYPE_MEMORY_REGION,
                             (Object **) &s->io_memory,
                             qdev_prop_allow_set_link_before_realize, 0);
}

static void i440fx_pcihost_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    I440FXState *s = I440FX_PCI_HOST_BRIDGE(dev);
    PCIHostState *phb = PCI_HOST_BRIDGE(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    PCIBus *b;
    PCIDevice *d;
    PCII440FXState *f;
    unsigned i;

    memory_region_add_subregion(s->io_memory, 0xcf8, &phb->conf_mem);
    sysbus_init_ioports(sbd, 0xcf8, 4);

    memory_region_add_subregion(s->io_memory, 0xcfc, &phb->data_mem);
    sysbus_init_ioports(sbd, 0xcfc, 4);

    /* register i440fx 0xcf8 port as coalesced pio */
    memory_region_set_flush_coalesced(&phb->data_mem);
    memory_region_add_coalescing(&phb->conf_mem, 0, 4);

    b = pci_root_bus_new(dev, NULL, s->pci_address_space,
                         s->io_memory, 0, TYPE_PCI_BUS);
    phb->bus = b;

    d = pci_create_simple(b, 0, s->pci_type);
    f = I440FX_PCI_DEVICE(d);

    range_set_bounds(&s->pci_hole, s->below_4g_mem_size,
                     IO_APIC_DEFAULT_ADDRESS - 1);

    /* setup pci memory mapping */
    pc_pci_as_mapping_init(s->system_memory, s->pci_address_space);

    /* if *disabled* show SMRAM to all CPUs */
    memory_region_init_alias(&f->smram_region, OBJECT(d), "smram-region",
                             s->pci_address_space, SMRAM_C_BASE, SMRAM_C_SIZE);
    memory_region_add_subregion_overlap(s->system_memory, SMRAM_C_BASE,
                                        &f->smram_region, 1);
    memory_region_set_enabled(&f->smram_region, true);

    /* smram, as seen by SMM CPUs */
    memory_region_init(&f->smram, OBJECT(d), "smram", 4 * GiB);
    memory_region_set_enabled(&f->smram, true);
    memory_region_init_alias(&f->low_smram, OBJECT(d), "smram-low",
                             s->ram_memory, SMRAM_C_BASE, SMRAM_C_SIZE);
    memory_region_set_enabled(&f->low_smram, true);
    memory_region_add_subregion(&f->smram, SMRAM_C_BASE, &f->low_smram);
    object_property_add_const_link(qdev_get_machine(), "smram",
                                   OBJECT(&f->smram));

    init_pam(&f->pam_regions[0], OBJECT(d), s->ram_memory, s->system_memory,
             s->pci_address_space, PAM_BIOS_BASE, PAM_BIOS_SIZE);
    for (i = 0; i < ARRAY_SIZE(f->pam_regions) - 1; ++i) {
        init_pam(&f->pam_regions[i + 1], OBJECT(d), s->ram_memory,
                 s->system_memory, s->pci_address_space,
                 PAM_EXPAN_BASE + i * PAM_EXPAN_SIZE, PAM_EXPAN_SIZE);
    }

    ram_addr_t ram_size = s->below_4g_mem_size + s->above_4g_mem_size;
    ram_size = ram_size / 8 / 1024 / 1024;
    if (ram_size > 255) {
        ram_size = 255;
    }
    d->config[I440FX_COREBOOT_RAM_SIZE] = ram_size;

    i440fx_update_memory_mappings(f);
}

static const Property i440fx_pci_props[] = {
    DEFINE_PROP_BOOL("agp", PCII440FXState, agp, false),
    DEFINE_PROP_SIZE("agp-aperture-size", PCII440FXState, agp_aperture_size,
                     I440FX_AGP_APERTURE_SIZE_DEFAULT),
};

static void i440fx_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = i440fx_realize;
    k->config_write = i440fx_write_config;
    k->vendor_id = PCI_VENDOR_ID_INTEL;
    k->device_id = PCI_DEVICE_ID_INTEL_82441;
    k->revision = 0x02;
    k->class_id = PCI_CLASS_BRIDGE_HOST;
    dc->desc = "Host bridge";
    dc->vmsd = &vmstate_i440fx;
    device_class_set_props(dc, i440fx_pci_props);
    /*
     * PCI-facing part of the host bridge, not usable without the
     * host-facing part, which can't be device_add'ed, yet.
     */
    dc->user_creatable = false;
    dc->hotpluggable   = false;
}

static const TypeInfo i440fx_info = {
    .name          = TYPE_I440FX_PCI_DEVICE,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PCII440FXState),
    .class_init    = i440fx_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static const char *i440fx_pcihost_root_bus_path(PCIHostState *host_bridge,
                                                PCIBus *rootbus)
{
    return "0000:00";
}

static const Property i440fx_props[] = {
    DEFINE_PROP_SIZE(PCI_HOST_PROP_PCI_HOLE64_SIZE, I440FXState,
                     pci_hole64_size, I440FX_PCI_HOST_HOLE64_SIZE_DEFAULT),
    DEFINE_PROP_SIZE(PCI_HOST_BELOW_4G_MEM_SIZE, I440FXState,
                     below_4g_mem_size, 0),
    DEFINE_PROP_SIZE(PCI_HOST_ABOVE_4G_MEM_SIZE, I440FXState,
                     above_4g_mem_size, 0),
    DEFINE_PROP_BOOL("x-pci-hole64-fix", I440FXState, pci_hole64_fix, true),
    DEFINE_PROP_STRING(I440FX_HOST_PROP_PCI_TYPE, I440FXState, pci_type),
};

static void i440fx_pcihost_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIHostBridgeClass *hc = PCI_HOST_BRIDGE_CLASS(klass);

    hc->root_bus_path = i440fx_pcihost_root_bus_path;
    dc->realize = i440fx_pcihost_realize;
    dc->fw_name = "pci";
    device_class_set_props(dc, i440fx_props);
    /* Reason: needs to be wired up by pc_init1 */
    dc->user_creatable = false;

    object_class_property_add(klass, PCI_HOST_PROP_PCI_HOLE_START, "uint32",
                              i440fx_pcihost_get_pci_hole_start,
                              NULL, NULL, NULL);

    object_class_property_add(klass, PCI_HOST_PROP_PCI_HOLE_END, "uint32",
                              i440fx_pcihost_get_pci_hole_end,
                              NULL, NULL, NULL);

    object_class_property_add(klass, PCI_HOST_PROP_PCI_HOLE64_START, "uint64",
                              i440fx_pcihost_get_pci_hole64_start,
                              NULL, NULL, NULL);

    object_class_property_add(klass, PCI_HOST_PROP_PCI_HOLE64_END, "uint64",
                              i440fx_pcihost_get_pci_hole64_end,
                              NULL, NULL, NULL);
}

static const TypeInfo i440fx_pcihost_info = {
    .name          = TYPE_I440FX_PCI_HOST_BRIDGE,
    .parent        = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(I440FXState),
    .instance_init = i440fx_pcihost_initfn,
    .class_init    = i440fx_pcihost_class_init,
};

static void i440fx_register_types(void)
{
    type_register_static(&i440fx_info);
    type_register_static(&i440fx_pcihost_info);
}

type_init(i440fx_register_types)
