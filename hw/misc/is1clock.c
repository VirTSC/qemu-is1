/*
 * is1-clock - the host's wall clock, readable by the IntelliStar 1 guest.
 *
 * The guest keeps time with the TSC, and FreeBSD 4.8 measures the TSC's rate
 * once at boot against the i8254. In a VM that measurement can come out a few
 * percent high, and the guest clock then runs slow for the whole boot. renderd
 * compares its frame count against time() and kills itself once they are four
 * seconds apart, so a slow guest clock ends in "Time drifted too much".
 *
 * Rather than second-guessing the guest's timekeeping, this hands it the
 * host's time and lets a small guest daemon (virtsc src/is1clock) step the
 * guest clock back into line whenever it wanders.
 *
 * The time reported follows -rtc base=, so it is the same "now" the guest
 * read from the RTC at boot: with base=localtime, the guest's UTC is the
 * host's local time, and the device reports that too.
 *
 * Registers, all 32-bit, at iobase (default 0x530):
 *   +0x0  read:  IS1CLOCK_MAGIC, to detect the device
 *         write: latch the current time into the three registers below
 *   +0x4  read:  latched seconds since the epoch, low 32 bits
 *   +0x8  read:  latched seconds since the epoch, high 32 bits
 *   +0xc  read:  latched nanoseconds, 0-999999999
 *   +0x10 read:  the guest's TSC rate in Hz, low 32 bits (latched too)
 *   +0x14 read:  the guest's TSC rate in Hz, high 32 bits; 0 in both means
 *                the rate is not known
 *
 * The TSC rate lets the guest replace its own boot-time measurement of it
 * (machdep.tsc_freq), which is where the slow clock comes from. Under KVM the
 * rate is the one KVM gives the vCPU. Under TCG the guest's TSC is QEMU's
 * tick counter, whose rate QEMU never records, so it is measured here against
 * the virtual clock over the whole time since the device was created. Both
 * stop together when the VM is paused, and both follow the instruction count
 * under -icount, so the ratio holds either way.
 */
#include "qemu/osdep.h"
#include "qemu/cutils.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "hw/isa/isa.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/rtc.h"
#include "system/tcg.h"
#include "system/cpu-timers.h"
#include "hw/core/cpu.h"
#include "qom/object.h"

#define TYPE_IS1CLOCK "is1-clock"
OBJECT_DECLARE_SIMPLE_TYPE(Is1ClockState, IS1CLOCK)

#define IS1CLOCK_IO_SIZE    0x20
#define IS1CLOCK_MAGIC      0x43315349  /* "IS1C" */

#define IS1CLOCK_REG_MAGIC  0x0
#define IS1CLOCK_REG_SEC_LO 0x4
#define IS1CLOCK_REG_SEC_HI 0x8
#define IS1CLOCK_REG_NSEC   0xc
#define IS1CLOCK_REG_TSC_LO 0x10
#define IS1CLOCK_REG_TSC_HI 0x14

/* Below this much virtual time the TCG measurement is too coarse to report. */
#define IS1CLOCK_TSC_MIN_NS (NANOSECONDS_PER_SECOND / 10)

struct Is1ClockState {
    ISADevice parent_obj;

    MemoryRegion io;
    uint32_t io_base;

    int64_t sec;
    uint32_t nsec;
    uint64_t tsc_hz;

    /* Where the TCG TSC measurement starts from. */
    int64_t base_ticks;
    int64_t base_ns;
};

/* The guest's TSC rate in Hz, or 0 if it cannot be told. */
static uint64_t is1clock_tsc_hz(Is1ClockState *s)
{
    int64_t hz = 0, ticks, ns;

    /* KVM and HVF fill this in from the accelerator; TCG leaves it at 0. */
    if (first_cpu) {
        hz = object_property_get_int(OBJECT(first_cpu), "tsc-frequency",
                                     NULL);
    }
    if (hz > 0) {
        return hz;
    }
    if (!tcg_enabled()) {
        return 0;
    }

    ticks = cpus_get_elapsed_ticks() - s->base_ticks;
    ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->base_ns;
    if (ns < IS1CLOCK_TSC_MIN_NS || ticks <= 0) {
        return 0;
    }
    /* muldiv64() takes a 32-bit divisor; ns passes that after 4.29 s. */
    return (uint64_t)((double)ticks * NANOSECONDS_PER_SECOND / ns + 0.5);
}

/*
 * Take the host clock and shift it by whole seconds to match the RTC's base.
 * The two are read separately, so retry if a second boundary fell between
 * them; otherwise the offset would be out by one for that read.
 */
static void is1clock_latch(Is1ClockState *s)
{
    int64_t ns = 0, offset = 0;
    struct tm tm;
    int i;

    for (i = 0; i < 3; i++) {
        ns = qemu_clock_get_ns(QEMU_CLOCK_HOST);
        qemu_get_timedate(&tm, 0);
        if (qemu_clock_get_ns(QEMU_CLOCK_HOST) / NANOSECONDS_PER_SECOND ==
            ns / NANOSECONDS_PER_SECOND) {
            break;
        }
    }
    offset = (int64_t)mktimegm(&tm) - ns / NANOSECONDS_PER_SECOND;

    s->sec = ns / NANOSECONDS_PER_SECOND + offset;
    s->nsec = ns % NANOSECONDS_PER_SECOND;
    s->tsc_hz = is1clock_tsc_hz(s);
}

static uint64_t is1clock_read(void *opaque, hwaddr addr, unsigned size)
{
    Is1ClockState *s = opaque;

    switch (addr) {
    case IS1CLOCK_REG_MAGIC:
        return IS1CLOCK_MAGIC;
    case IS1CLOCK_REG_SEC_LO:
        return (uint32_t)s->sec;
    case IS1CLOCK_REG_SEC_HI:
        return (uint32_t)((uint64_t)s->sec >> 32);
    case IS1CLOCK_REG_NSEC:
        return s->nsec;
    case IS1CLOCK_REG_TSC_LO:
        return (uint32_t)s->tsc_hz;
    case IS1CLOCK_REG_TSC_HI:
        return (uint32_t)(s->tsc_hz >> 32);
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "is1-clock: read of 0x%" HWADDR_PRIx
                      "\n", addr);
        return 0;
    }
}

static void is1clock_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    Is1ClockState *s = opaque;

    if (addr == IS1CLOCK_REG_MAGIC) {
        is1clock_latch(s);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "is1-clock: write of 0x%" HWADDR_PRIx
                      "\n", addr);
    }
}

static const MemoryRegionOps is1clock_io_ops = {
    .read = is1clock_read,
    .write = is1clock_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void is1clock_realize(DeviceState *dev, Error **errp)
{
    ISADevice *isa = ISA_DEVICE(dev);
    Is1ClockState *s = IS1CLOCK(dev);

    memory_region_init_io(&s->io, OBJECT(s), &is1clock_io_ops, s,
                          "is1-clock", IS1CLOCK_IO_SIZE);
    memory_region_add_subregion(isa_address_space_io(isa), s->io_base,
                                &s->io);

    s->base_ticks = cpus_get_elapsed_ticks();
    s->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static const VMStateDescription vmstate_is1clock = {
    .name = TYPE_IS1CLOCK,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_INT64(sec, Is1ClockState),
        VMSTATE_UINT32(nsec, Is1ClockState),
        VMSTATE_UINT64(tsc_hz, Is1ClockState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property is1clock_properties[] = {
    /* Just past is1gl's 0x520-0x527. */
    DEFINE_PROP_UINT32("iobase", Is1ClockState, io_base, 0x530),
};

static void is1clock_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = is1clock_realize;
    dc->vmsd = &vmstate_is1clock;
    dc->desc = "IntelliStar host wall clock";
    device_class_set_props(dc, is1clock_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo is1clock_info = {
    .name          = TYPE_IS1CLOCK,
    .parent        = TYPE_ISA_DEVICE,
    .instance_size = sizeof(Is1ClockState),
    .class_init    = is1clock_class_init,
};

static void is1clock_register_types(void)
{
    type_register_static(&is1clock_info);
}

type_init(is1clock_register_types)
