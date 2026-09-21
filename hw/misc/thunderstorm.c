/*
 * Thunderstorm video card (The Weather Channel / Wind River) — QEMU device model.
 *
 * The card is a PowerPC/VxWorks board behind a Marvell (Galileo) GT-64260 PCI
 * bridge, used by the IntelliStar 1 to overlay rendered graphics onto a live
 * video feed. This models it well enough for the unmodified FreeBSD 4.8 driver
 * (tsc.ko) to attach, so the IntelliStar software can run without the hardware.
 *
 * With present=on the model takes the card's full path: the 'tsc7' gate, the
 * CPLD reset pulse, the firmware upload, the boot-status and version handshake,
 * and the I2O message unit - four queue ports, a message pool in BAR1, and a
 * level interrupt on INTA#. With present=off it withholds 'tsc7', which makes
 * tsc_attach print "Board apparently not ready to run", skip IRQ setup, GART
 * and firmware entirely, still create all nine /dev nodes, and return success.
 * That degraded path is what the earlier bring-up phases were built on and is
 * still the default.
 *
 * Every register access is traced, and so is every message. The traces are how
 * each phase gets validated against notes/thunderstorm-device-spec.md; the
 * message-level ones are also how we find out what the application asks for
 * next.
 *
 * Usage:
 *     -device thunderstorm
 * Function 1 is created automatically; the driver requires both functions to
 * probe before tsc_attach will do anything.
 */

#include "qemu/osdep.h"
#include <math.h>
#include <poll.h>
#include "system/rtc.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "hw/pci/pci_device.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "chardev/char-fe.h"
#include "qemu/module.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "ui/console.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "migration/vmstate.h"
#include "trace.h"

#define TYPE_THUNDERSTORM     "thunderstorm"
#define TYPE_THUNDERSTORM_FN1 "thunderstorm-fn1"

OBJECT_DECLARE_SIMPLE_TYPE(ThunderstormState, THUNDERSTORM)
OBJECT_DECLARE_SIMPLE_TYPE(ThunderstormFn1State, THUNDERSTORM_FN1)

/* PCI identity — from tsc_probe. 0x6431 is accepted by the driver as a variant. */
#define TS_VENDOR_ID 0x11AB /* Marvell, formerly Galileo Technology */
#define TS_DEVICE_ID 0x6430 /* GT-64260 */

/*
 * BAR sizes. fn0 BAR0/BAR1 are the PCI slave windows onto the card's two SDRAM
 * banks, 64 MB each, from firmware sysHwInit. Critically, bank 1's base equals
 * BAR1's base, which is why an I2O MFA (a byte offset into that bank) is also a
 * byte offset into BAR1 with no fixup.
 *
 * fn1's BARs are local-bus chip selects, not sized by sysHwInit. tscCpld only
 * needs to reach the reset byte at 0x13; tscFlash needs 0x100000 + 2 MB if the
 * fallback firmware path is ever exercised.
 */
#define TS_BAR_REG_SIZE   (64 * MiB)
#define TS_BAR_MEM_SIZE   (64 * MiB)
#define TS_BAR_GALIO_SIZE (64 * KiB)
#define TS_BAR_FPGA_SIZE  (64 * KiB)
#define TS_BAR_CPLD_SIZE  (64 * KiB)
#define TS_BAR_FLASH_SIZE (4 * MiB)

/* BAR indices, i.e. the rid values the driver passes to bus_alloc_resource. */
#define TS_BAR_0 0 /* rid 0x10 */
#define TS_BAR_1 1 /* rid 0x14 */
#define TS_BAR_4 4 /* rid 0x20 */

/*
 * tscReg layout. Offsets below 0x100 are GT-64260 messaging-unit registers and
 * are endian-corrected by the bridge, so the host sees native values. The 0x8000
 * block is PPC SDRAM scratch and is byte-transparent, so anything we place there
 * must be pre-swapped to big-endian for the driver to read it correctly.
 */
#define TS_REG_OMR0          0x18   /* outbound message reg 0: one vterm char */
#define TS_REG_DOORBELL_IN   0x20   /* host -> card; a write here boots the image */
#define TS_REG_DOORBELL_OUT  0x2C   /* holds 'tsc7' once the boot ROM is up */
#define TS_REG_CAUSE         0x30   /* outbound interrupt cause, R/W1C */
#define TS_REG_MASK          0x34   /* outbound interrupt mask, 1 = MASKED */
#define TS_REG_QUEUE_IN      0x40   /* read: inbound free, write: inbound post */
#define TS_REG_QUEUE_OUT     0x44   /* read: outbound post, write: outbound free */

#define TS_MAGIC_TSC7        0x74736337 /* 'tsc7' */
#define TS_QUEUE_EMPTY       0xFFFFFFFF

#define TS_SDRAM_BOOT_STATUS 0x8000 /* polled until 0x05000000 (big-endian 5) */
#define TS_SDRAM_HOST_READY  0x8004 /* host clears it, never reads back */
#define TS_SDRAM_VERSION     0x8008 /* big-endian version word */
#define TS_FW_LOAD_OFFSET    0x10000 /* firmware blob lands here, 2 MB */

#define TS_CAUSE_VTERM       0x1
#define TS_CAUSE_I2O         0x8

/*
 * Host->guest vterm queue. The guest takes one byte per interrupt, so a burst
 * typed at the chardev has to wait somewhere. 4 KB is far more than a console
 * ever needs and still trivial next to the 64 KB rbuf on the other side.
 */
#define TS_VTERM_FIFO        4096

/* tscGalio: the master interrupt gate. Without bits 21-22 set, INTA# is dead. */
#define TS_GALIO_INT_GATE    0x0C24
#define TS_GALIO_GATE_BITS   0x600000

/* Size of the low tscReg window we actually back with storage. */
#define TS_REG_SHADOW_SIZE   0x10000

/*
 * I2O message unit.
 *
 * The message pool lives in tscMem (fn0 BAR1). An MFA is a byte offset into
 * that BAR with no fixup at all - the card's SDRAM bank 1 base and BAR1's base
 * are the same address - and the driver never bounds-checks one, so the pool
 * may be laid out freely. 128 messages each way at stride 0x100 is 64 KB in
 * total, which is the pool size the real card was observed to use.
 */
#define TS_MSG_SIZE      0x100
#define TS_MSG_COUNT     128
#define TS_POOL_IN_BASE  0x00010000
#define TS_POOL_OUT_BASE 0x00018000

/* Message header. Native little-endian x86 layout - do not byte-swap. */
#define TS_MSG_NEXT     0  /* card-owned free-list link; the host never touches it */
#define TS_MSG_CMD      4  /* BSD ioctl number */
#define TS_MSG_DEVICE   8  /* minor 0..8, or 9 = SYS_DEV */
#define TS_MSG_STATUS  12  /* errno; becomes the ioctl return value */
#define TS_MSG_WHO     16  /* opaque host token; MUST be echoed verbatim */
#define TS_MSG_DATA    20
#define TS_MSG_DATA_MAX (TS_MSG_SIZE - TS_MSG_DATA)

/* BSD ioctl encoding: length in 16-28, group in 8-15, nr in 0-7. */
#define TS_IOC_LEN(c)   (((c) >> 16) & 0x1FFF)
#define TS_IOC_GROUP(c) (((c) >> 8) & 0xFF)
#define TS_IOC_NR(c)    ((c) & 0xFF)

#define TS_CMD_OPEN      0x200056C0 /* driver-generated on device open  */
#define TS_CMD_CLOSE     0x200056C1 /* ... and close                    */
#define TS_CMD_REQBUFS   0xC0105608 /* _IOWR('V',8,struct v4l2_requestbuffers) */
#define TS_CMD_QUERYBUF  0xC0405609 /* _IOWR('V',9,struct v4l2_buffer)         */
#define TS_CMD_QBUF      0xC040410F /* _IOWR('A',15,struct v4l2_buffer) */
#define TS_CMD_S_CTRL    0x8008561C
#define TS_CMD_G_INPUT   0x40045626 /* _IOR ('V',38,int) */
#define TS_CMD_S_INPUT   0xC0045627 /* _IOWR('V',39,int) */
#define TS_CMD_G_OUTPUT  0x4004562E /* _IOR ('V',46,int) */
#define TS_CMD_S_OUTPUT  0xC004562F /* _IOWR('V',47,int) */
#define TS_CMD_STREAMON  0x80045612
#define TS_CMD_STREAMOFF 0x80045613
#define TS_CMD_SET_AGP   0x200041C4 /* _IO('A',196), fire and forget    */
#define TS_CMD_TERM_NR   194        /* _IOWR('A',194,len), vterm output */

#define TS_ENOTTY        25         /* FreeBSD errno for "not a typewriter" */
#define TS_EINVAL        22

/*
 * TSH_MAX_FRAMES from ThunderstormHost.h. The driver ignores it and computes
 * count = agp_space >> 21 for minor 0, but it is still the vendor's own number
 * for how many frames the card will hold, and nothing has contradicted it.
 */
#define TS_MAX_FRAMES    10

/*
 * Playback buffers, for the minor-1 spool. They live in tscMem above the
 * message pool. 2 MB each is the stride the driver itself uses for capture
 * frames in VIDIOC_SET_AGP, and it covers an ARGB8888 NTSC frame
 * (720 x 486 x 4 = 1.33 MB) with room to spare.
 *
 * The offsets are a guess: nothing recovered so far says where the card puts
 * playback buffers, only that QUERYBUF on minor 1 is forwarded to it rather
 * than answered locally. If the driver's mmap for minor 1 does not resolve
 * these against tscMem, that will show up as vspoold failing at the next step
 * along, which is exactly the signal we want.
 */
#define TS_PB_BASE       0x00200000
#define TS_PB_STRIDE     0x00200000

/* struct v4l2_buffer, 64 bytes; offsets from videodev.h. */
#define TS_VB_INDEX   0
#define TS_VB_TYPE    4
#define TS_VB_OFFSET  8
#define TS_VB_LENGTH  12
#define TS_VB_BYTESUSED 16
#define TS_VB_FLAGS     20
#define TS_VB_TIMESTAMP 24  /* s64, native little-endian */
/*
 * struct v4l2_timecode, and note it is NOT the mainline v4l2 layout: this
 * unit's videodev.h puts frames/seconds/minutes/hours FIRST and flags/type
 * after, where mainline puts type and flags first. Checked against
 * reference/wrs-headers/videodev.h, and cross-checked by TS_VB_SEQUENCE
 * landing at 48 either way.
 */
#define TS_VB_TIMECODE  32
#define TS_VB_TC_FRAMES  (TS_VB_TIMECODE + 0)
#define TS_VB_TC_SECONDS (TS_VB_TIMECODE + 1)
#define TS_VB_TC_MINUTES (TS_VB_TIMECODE + 2)
#define TS_VB_TC_HOURS   (TS_VB_TIMECODE + 3)
#define TS_VB_TC_FLAGS   (TS_VB_TIMECODE + 8)
#define TS_VB_TC_TYPE    (TS_VB_TIMECODE + 12)
#define V4L2_TC_TYPE_30FPS      3
#define V4L2_TC_FLAG_DROPFRAME  0x0001
#define V4L2_BUF_FLAG_TIMECODE  0x0100
#define TS_VB_SEQUENCE  48
#define TS_VB_PA        52

#define V4L2_BUF_FLAG_MAPPED 0x0001
#define V4L2_BUF_FLAG_QUEUED 0x0002
#define V4L2_BUF_FLAG_DONE   0x0004

/*
 * The frame loop.
 *
 * QBUF arrives with who == 0 - fire and forget, nobody is asleep on it. The
 * card takes the buffer, does a frame's worth of work with it, and sends it
 * back up as a QBUF *completion*, which the driver's ISR routes to that minor's
 * buffer queue and selwakeup()s. That return is the only clock the application
 * has: renderd corrects its frame loop against it, which is why the frame rate
 * was measured as paced rather than CPU-bound.
 *
 * So one completion per frame period, per streaming minor, from a wall-clock
 * timer. NTSC is 30000/1001 frames per second.
 */
#define TS_MINORS        9  /* the nine /dev nodes make_devices() creates */

/*
 * Frame geometry, verbatim from ThunderstormHost.h. A TSH_FRAME starts with
 * its video plane at offset 0: 480 lines of 1024 dwords, of which the first
 * 720 are active picture, packed ARGB. The 1024-dword line is not padding for
 * its own sake - input is 4:2:2 YCrCb at 720 samples and output is ARGB at
 * 720 pixels, and the vendor aligned both to a common 4096-byte line.
 *
 * The app confirms it from the other side: renderd reads back with
 * GL_PACK_ROW_LENGTH = 1024 and GL_BGRA/GL_UNSIGNED_INT_8_8_8_8_REV, which is
 * this exact layout.
 */
#define TS_VIDEO_LINES   480
#define TS_VIDEO_ACTIVE  720
#define TS_VIDEO_STRIDE  4096

/*
 * The video plane is a union, and the two halves are different colour spaces.
 * TSH_VIDEO is:
 *
 *      TSH_YCRCB in [480][1024]    4:2:2, Cb-Y0-Cr-Y1 packed per dword
 *      TSH_ARGB  out[480][1024]    ARGB, one pixel per dword
 *
 * Output is what renderd writes and what we present: 720 active pixels of the
 * 1024-dword line. Input is what the capture engine writes and renderd reads,
 * and it is *two* pixels per dword - so 720 active pixels is 360 dwords, and a
 * full line is 2048 pixels. renderd confirms this from the other side: it
 * uploads the captured video with
 *
 *      glTexImage2D(GL_YCBCR_MESA, ..., GL_UNSIGNED_SHORT_8_8_APPLE)
 *      glPixelStorei(GL_UNPACK_ROW_LENGTH, 2048)
 *
 * which is 4:2:2 at exactly that row length.
 */
#define TS_VIDEO_IN_DWORDS 360   /* 720 active pixels, two per dword */
#define TS_VIDEO_LINE_DWORDS (TS_VIDEO_STRIDE / 4)  /* the whole 1024-dword line */
#define TS_FRAME_NS      (NANOSECONDS_PER_SECOND * 1001 / 30000)
#define TS_QUEUED_MAX    64

/*
 * The rest of TSH_FRAME, past the picture. Laid out exactly as the header
 * declares it, so the offsets are additions rather than a guess:
 *
 *      TSH_VIDEO     video       480 * 4096          = 1966080
 *      TSH_BLANKING  blanking     45 *  512 * 4      =   92160
 *      TSH_AUDIO     audio      2048 *    2 * 4      =   16384
 *      long          audioCount                      =       4
 *
 * audio is two interleaved channels of signed 24-bit samples, sign-extended
 * into a dword, A then B; audioCount is the number of A/B *pairs* that are
 * valid. The whole struct rounds up inside the 0x200000 per-frame stride the
 * host declares in VIDIOC_SET_AGP, so these offsets are always in range.
 *
 * Nothing in the model consumes audio - there is no output sink for it. This
 * is instrumentation only, to settle whether the application ever produces
 * any: every frame observed up to now has had audioCount == 0, and the
 * suspicion was that this is because only one product had ever been driven.
 */
#define TS_BLANKING_BYTES  (45 * 512 * 4)
#define TS_AUDIO_SAMPLES   (2048 * 2)          /* dwords, A/B interleaved */
#define TS_AUDIO_OFF       (TS_VIDEO_LINES * TS_VIDEO_STRIDE + TS_BLANKING_BYTES)
#define TS_AUDIO_COUNT_OFF (TS_AUDIO_OFF + TS_AUDIO_SAMPLES * 4)
#define TS_VIDEO_BYTES     (TS_VIDEO_LINES * TS_VIDEO_STRIDE)
/* Everything the host declared per frame that is not picture. */
#define TS_FRAME_TAIL_BYTES (0x200000 - TS_VIDEO_BYTES)
/*
 * The output stream: a 32-byte header, then 720x480 BGRA top-down, then
 * `pairs` interleaved A/B sample pairs exactly as they sit in the frame -
 * signed 24-bit sign-extended into 32-bit dwords. The samples are passed
 * through unscaled because that is what the card carries; a player that
 * wants full-scale 32-bit shifts left by 8.
 *
 * Magic is 'ITS2': the v1 stream had a 16-byte header and no audio, and a
 * stale viewer reading a v2 stream would desynchronise silently rather than
 * complain.
 */
#define TS_OUTPUT_MAGIC  0x32535449   /* 'ITS2' */
#define TS_OUTPUT_HDR    32
#define TS_OUTPUT_RATE   48000
/*
 * The last two members of TSH_FRAME, after audioCount and vbiTypes[45].
 * vbiTypes is 45 bytes of unsigned char starting at 2074628, so it ends at
 * 2074673 and the next long aligns at 2074676. Both are `long`, i.e. 32-bit
 * on this target.
 */
#define TS_VBITYPES_OFF    (TS_AUDIO_COUNT_OFF + 4)
#define TS_TSTAMP_OFF      ((TS_VBITYPES_OFF + 45 + 3) & ~3)
#define TS_ID_OFF          (TS_TSTAMP_OFF + 4)

typedef struct TsFrameQueue {
    uint8_t  buf[TS_QUEUED_MAX][64]; /* the v4l2_buffer the host handed down */
    uint32_t head, count;
    uint32_t sequence;
} TsFrameQueue;

typedef struct TsQueue {
    uint32_t mfa[TS_MSG_COUNT];
    uint32_t head;
    uint32_t count;
} TsQueue;

struct ThunderstormState {
    PCIDevice parent_obj;

    MemoryRegion reg;   /* BAR0 tscReg   (SCS0)   */
    MemoryRegion mem;   /* BAR1 tscMem   (SCS1)   */
    MemoryRegion galio; /* BAR4 tscGalio (INTMEM) */

    uint8_t *mem_ram;   /* host view of tscMem, where the message pool lives */
    QEMUBH *msg_bh;

    PCIDevice *fn1;

    /* Properties */
    bool present;        /* report 'tsc7', i.e. take the full attach path */
    uint32_t version;    /* firmware version reported at TS_SDRAM_VERSION */

    /* Register state */
    uint8_t shadow[TS_REG_SHADOW_SIZE];
    uint32_t cause;
    uint32_t mask;
    uint32_t boot_status;
    uint32_t galio_gate;
    bool irq_level;

    /* I2O queues. in_post is ours; the other three are what the ports expose. */
    TsQueue in_free, in_post, out_free, out_post;

    /* Frame loop: buffers the host has queued, waiting to be handed back. */
    TsFrameQueue frames[TS_MINORS];
    QEMUTimer *frame_timer;
    int64_t next_tick;   /* absolute deadline, so the period cannot drift */
    int64_t last_tick;
    bool streaming;
    uint64_t frames_returned;
    uint64_t frames_starved;

    uint32_t av_route[2];  /* [0] = current input, [1] = current output */

    /*
     * The capture side: what the card hands the host as "incoming video".
     * There is no DVB-ASI feed here, so it is a test pattern.
     */
    char *input;            /* "bars" | "black" | "none" */
    char *input_file;       /* raw ARGB8888 720x480, overrides input */
    uint32_t *pattern;      /* TS_VIDEO_ACTIVE * TS_VIDEO_LINES, or NULL */
    uint64_t frames_filled;

    /* The card's video output, presented as a second QEMU console. */
    QemuConsole *con;
    bool display;
    uint32_t display_rate;   /* console refreshes per second */
    int64_t display_last;
    uint64_t frames_shown;

    /*
     * What goes in v4l2_buffer.timestamp (stamp_t, a raw s64). The card's
     * notion of time is the only house clock a genlocked device has, and
     * renderd's layer activation takes an absolute time that reads as zero
     * here - so what we put in this field is a live question, not a detail.
     * "virtual-us" is what the model did originally: microseconds since the
     * VM started, which is neither an epoch nor a documented unit.
     */
    char *stamp;
    /*
     * What goes in v4l2_buffer.timecode: "utc" (default), "free" or "off".
     *
     * This is renderd's clock, and until 2026-09-18 the model left it as
     * zeros - which pinned the application's notion of "now" at 00:00:00:00
     * and meant no timed layer activation ever fired. The whole local
     * segment had to be signalled with time=0 to render at all. See
     * notes/local-segment-activation.md.
     */
    char *timecode;
    /*
     * Capture-side audio: "off" (default), "tone" or "silence".
     *
     * The same TSH_FRAME travels both ways - the card fills it with input,
     * the host processes it, the card retransmits it - and unlike the video
     * plane, which is a union of an input and an output layout, the audio
     * plane is a single buffer with a single audioCount. So the expected flow
     * is that the card delivers captured audio in the frame and the host
     * modifies it in place. This model has never delivered any, which means
     * renderd has seen audioCount == 0 on every frame for the life of the
     * project and has had nothing to process. See todo #8.
     */
    char *audio;
    double audio_phase;
    bool audio_selftest;
    bool audio_heard;
    uint32_t audio_cadence;

    /*
     * And what goes in TSH_FRAME.tstamp on captured frames. The model has
     * never written it, so it has always read as exactly zero - which is
     * suspicious given renderd's layer activation compares against a clock
     * that also reads exactly zero, and given that on the real unit the card
     * is the time base for the whole application.
     */
    char *tstamp;

    char *frame_log;         /* CSV of frame-clock intervals, for measuring */
    FILE *frame_log_f;

    /*
     * The card's picture, streamed out as raw BGRA for a viewer on the host.
     * The handoff asked for this: the frames the app DMAs across are fill and
     * key already separated, and piping them out is a better artifact than
     * the real card produced. tools/is1view is the viewer.
     *
     * Writes are non-blocking and dropped if the reader is slow. That is not
     * politeness - this runs in the main loop, and the console path's own
     * comment records that doing 1.4 MB of work here thirty times a second is
     * what used to make the frame clock jitter.
     */
    char *output;
    int   output_fd;
    int   output_pipe_sz;
    uint8_t *output_buf;
    size_t output_off, output_pending;
    uint64_t output_frames, output_dropped;

    /*
     * Audio, observed rather than carried. audio_frames counts output frames
     * that arrived with a non-zero audioCount; audio_peak is the largest
     * absolute sample seen, which separates "the app filled the buffer" from
     * "the app filled the buffer with silence".
     */
    uint64_t audio_frames;
    uint32_t audio_count_max;
    uint32_t audio_peak;
    bool audio_seen;
    bool audio_probed;
    bool audio_mapped;

    /*
     * Every distinct (cmd, device) the host has sent, reported once each.
     * The trace backend already records all of them, but a trace has to be
     * asked for in advance and then read back, and the question "what does the
     * application actually ask this card to do" keeps coming up - most
     * recently over audio, where VIDIOC_S_AUDIO has never been observed and it
     * matters whether that is because renderd never sends it.
     */
    GHashTable *cmds_seen;

    /* Whatever the last VIDIOC_SET_AGP said. */
    uint32_t agp_count, agp_base, agp_size, agp_offset, agp_stride;

    /*
     * vterm (minor 2): the card's debug console, and the only interrupt source
     * in this device that the IntelliStar application never touches.
     *
     * Guest -> host is VIDIOC_TERM, consumed here and written to the chardev.
     * Host -> guest is one character at a time: the byte goes in OMR0 and
     * cause bit 0 is raised. tsc_intr reads OMR0 exactly once per interrupt,
     * stores the low byte into rbuf[rbtail], wraps rbtail at 0x10000 and
     * write-1-clears bit 0 - so anything arriving faster than the guest acks
     * has to be queued here and re-armed on each ack.
     */
    /*
     * Live capture input: real video and audio fed in from outside, instead of
     * the synthetic pattern. See notes/av-io-design.md.
     *
     * A reader thread owns the pipes and the staging buffers; it converts
     * nothing the guest can see and never touches guest memory. ts_frame_fill()
     * still does the one copy into the guest on the QBUF bottom half, taking
     * the newest complete frame under this lock - so the thread may block on
     * the network or a slow producer for as long as it likes, and the frame
     * clock does not care.
     *
     * Falls back to the synthetic pattern whenever a frame has not arrived, so
     * a stalled or absent source shows colour bars rather than a freeze.
     */
    char *input_pipe;         /* raw yuyv422 720x480, one frame per period */
    char *input_audio;        /* raw s32le 48 kHz stereo, or NULL */
    int   in_fd, in_afd;
    QemuThread in_thread;
    QemuMutex in_lock;
    bool  in_running, in_stopping;
    uint32_t *in_front;       /* newest complete frame, guarded by in_lock */
    uint32_t *in_back;        /* the one being read into */
    bool  in_valid;           /* in_front holds a frame */
    /*
     * Audio is a CONTINUOUS RING the consumer drains at the card's own rate,
     * not a per-frame buffer.
     *
     * It used to be "newest audio", one frame, written by the reader thread
     * and sampled once per card frame. That silently skips a block whenever
     * the producer is ahead and repeats one whenever it is behind, and since
     * the producer's clock (ffmpeg's -re, or a live source) is never exactly
     * the card's 30000/1001, it is always one or the other. Audible as pitch
     * flutter and micro-stutter at small skew and as outright distortion at
     * larger skew - and invisible to any check that only looks at levels,
     * because the level is right, it is the timeline that is wrong.
     */
    uint8_t *in_aring;        /* s32le stereo pairs, guarded by in_lock */
    size_t   in_aring_cap, in_aring_n;
    bool     in_aprimed;      /* enough buffered to start emitting */
    uint32_t *in_stage;       /* copied out under the lock, DMA'd outside it */
    uint8_t  *in_audio_stage;
    uint64_t in_frames, in_underruns, in_short;
    uint64_t in_aunder, in_aover;
    bool in_reported;

    CharFrontend vterm_chr;
    uint8_t vterm_fifo[TS_VTERM_FIFO];
    unsigned vterm_head, vterm_tail, vterm_used;
    uint64_t vterm_rx, vterm_tx, vterm_overrun;

    /* Stats, so a bring-up run can be summarised without reading every trace */
    uint64_t fw_bytes;
};

struct ThunderstormFn1State {
    PCIDevice parent_obj;

    MemoryRegion fpga;  /* BAR0 tscFpga  (CS0)    */
    MemoryRegion cpld;  /* BAR1 tscCpld  (CS1)    */
    MemoryRegion flash; /* BAR4 tscFlash (BTCS)   */

    uint8_t cpld_regs[TS_BAR_CPLD_SIZE];
};

/*
 * The message unit and the frame loop call into each other: a QBUF arriving
 * enqueues a frame, and the timer that returns frames posts messages.
 */
static void ts_frame_push(ThunderstormState *s, uint32_t device,
                          const uint8_t *vb);
static void ts_stream_start(ThunderstormState *s);
static void ts_frame_present(ThunderstormState *s, const uint8_t *vb);
static void ts_frame_stream(ThunderstormState *s, const uint8_t *vb);
static bool ts_stream_flush(ThunderstormState *s);
static void ts_frame_audio(ThunderstormState *s, const uint8_t *vb);
static void ts_frame_fill(ThunderstormState *s, const uint8_t *vb);
/* Whole video frames held while waiting for the audio that goes with them. */
#define TS_IN_VSLOTS 4

/* Frames of audio buffered before any is emitted. */
#define TS_IN_APRIME 4

static void ts_fill_audio(ThunderstormState *s, hwaddr base, uint32_t live_pairs);

/* ------------------------------------------------------ I2O message unit */

static void ts_q_reset(TsQueue *q)
{
    q->head = 0;
    q->count = 0;
}

static void ts_q_push(TsQueue *q, uint32_t mfa)
{
    if (q->count == TS_MSG_COUNT) {
        /*
         * Unreachable while every MFA in flight came out of our own pool, and
         * the guest cannot manufacture extras: an MFA it invents is rejected
         * by ts_msg() instead. Drop rather than corrupt the ring.
         */
        return;
    }
    q->mfa[(q->head + q->count) % TS_MSG_COUNT] = mfa;
    q->count++;
}

static bool ts_q_pop(TsQueue *q, uint32_t *mfa)
{
    if (q->count == 0) {
        return false;
    }
    *mfa = q->mfa[q->head];
    q->head = (q->head + 1) % TS_MSG_COUNT;
    q->count--;
    return true;
}

/* An MFA is only ever an offset into the pool we laid out. */
static uint8_t *ts_msg(ThunderstormState *s, uint32_t mfa)
{
    bool in  = mfa >= TS_POOL_IN_BASE &&
               mfa < TS_POOL_IN_BASE + TS_MSG_COUNT * TS_MSG_SIZE;
    bool out = mfa >= TS_POOL_OUT_BASE &&
               mfa < TS_POOL_OUT_BASE + TS_MSG_COUNT * TS_MSG_SIZE;

    if ((!in && !out) || (mfa & (TS_MSG_SIZE - 1))) {
        return NULL;
    }
    return s->mem_ram + mfa;
}

/*
 * INTA# is level-triggered and has three gates: the cause bits, the mask
 * (inverted - 1 means masked), and tscGalio[0x0C24] bits 21-22, which cut the
 * line off entirely. Cause bit 3 is not acked by a write; it simply follows
 * "is there anything on the outbound post queue", which is what the driver's
 * drain-until-empty ISR loop expects.
 */
static void ts_update_irq(ThunderstormState *s)
{
    bool gated = (s->galio_gate & TS_GALIO_GATE_BITS) == TS_GALIO_GATE_BITS;
    bool level;

    if (s->out_post.count) {
        s->cause |= TS_CAUSE_I2O;
    } else {
        s->cause &= ~(uint32_t)TS_CAUSE_I2O;
    }

    level = gated && (s->cause & ~s->mask) != 0;
    if (level != s->irq_level) {
        s->irq_level = level;
        trace_thunderstorm_irq(level, s->cause, s->mask, gated);
        pci_set_irq(PCI_DEVICE(s), level);
    }
}

/*
 * Hand the guest the next queued vterm byte, if it has finished with the last.
 *
 * Cause bit 0 is the interlock: tsc_intr reads OMR0 once and then W1Cs the bit,
 * so while it is still set the previous character has not been collected and
 * writing OMR0 again would lose it.
 */
static void ts_vterm_kick(ThunderstormState *s)
{
    if (!s->present || (s->cause & TS_CAUSE_VTERM) || !s->vterm_used) {
        return;
    }
    stl_le_p(s->shadow + TS_REG_OMR0, s->vterm_fifo[s->vterm_head]);
    s->vterm_head = (s->vterm_head + 1) % TS_VTERM_FIFO;
    s->vterm_used--;
    s->vterm_rx++;
    s->cause |= TS_CAUSE_VTERM;
    ts_update_irq(s);
}

static int ts_vterm_can_receive(void *opaque)
{
    ThunderstormState *s = opaque;

    return TS_VTERM_FIFO - s->vterm_used;
}

static void ts_vterm_receive(void *opaque, const uint8_t *buf, int size)
{
    ThunderstormState *s = opaque;
    int i;

    for (i = 0; i < size; i++) {
        if (s->vterm_used == TS_VTERM_FIFO) {
            s->vterm_overrun++;
            break;
        }
        s->vterm_fifo[s->vterm_tail] = buf[i];
        s->vterm_tail = (s->vterm_tail + 1) % TS_VTERM_FIFO;
        s->vterm_used++;
    }
    ts_vterm_kick(s);
}

/* Post a message up to the host. */
static void ts_send(ThunderstormState *s, uint32_t cmd, uint32_t device,
                    uint32_t who, uint32_t status,
                    const void *data, size_t len)
{
    uint32_t mfa;
    uint8_t *m;

    if (!ts_q_pop(&s->out_free, &mfa)) {
        trace_thunderstorm_i2o_starved("outbound");
        return;
    }
    m = ts_msg(s, mfa);

    stl_le_p(m + TS_MSG_NEXT, 0);
    stl_le_p(m + TS_MSG_CMD, cmd);
    stl_le_p(m + TS_MSG_DEVICE, device);
    stl_le_p(m + TS_MSG_STATUS, status);
    stl_le_p(m + TS_MSG_WHO, who);
    memset(m + TS_MSG_DATA, 0, TS_MSG_DATA_MAX);
    if (data && len) {
        memcpy(m + TS_MSG_DATA, data, MIN(len, TS_MSG_DATA_MAX));
    }

    ts_q_push(&s->out_post, mfa);
    trace_thunderstorm_i2o_reply(mfa, cmd, status, who);
}

static void ts_msg_process(ThunderstormState *s, uint32_t mfa)
{
    uint8_t *m = ts_msg(s, mfa);
    uint32_t cmd, device, who;
    uint32_t status = 0;
    bool reply = true;
    uint8_t payload[TS_MSG_DATA_MAX];
    size_t payload_len = 0;

    if (!m) {
        /* Not an MFA we ever handed out. Real hardware would fault; ignore. */
        return;
    }

    cmd = ldl_le_p(m + TS_MSG_CMD);
    device = ldl_le_p(m + TS_MSG_DEVICE);
    who = ldl_le_p(m + TS_MSG_WHO);
    trace_thunderstorm_i2o_post(mfa, cmd, device, who);

    if (s->cmds_seen) {
        gpointer key = GUINT_TO_POINTER(cmd ^ (device << 28));

        if (!g_hash_table_contains(s->cmds_seen, key)) {
            g_hash_table_add(s->cmds_seen, key);
            warn_report("thunderstorm: first use of cmd 0x%08x on device %u "
                        "(group '%c' nr %u len %u)", cmd, device,
                        TS_IOC_GROUP(cmd) ? TS_IOC_GROUP(cmd) : '?',
                        TS_IOC_NR(cmd), TS_IOC_LEN(cmd));
        }
    }

    switch (cmd) {
    case TS_CMD_OPEN:
    case TS_CMD_CLOSE:
        break;

    case TS_CMD_REQBUFS: {
        /*
         * struct v4l2_requestbuffers { int count; u32 type; u32 reserved[2]; }
         *
         * REQBUFS is handled locally by the driver for minor 0, where it turns
         * into req_agp_mem against the AGP aperture - but minor 1, the playback
         * spool, forwards it to the card, and vspoold dies at
         * SpoolerThread.cpp:85 "failed to request buffers" without an answer.
         * V4L2 lets the card grant fewer than asked, so cap and echo back.
         */
        uint32_t want = ldl_le_p(m + TS_MSG_DATA + 0);
        uint32_t got = MIN(want, TS_MAX_FRAMES);

        memcpy(payload, m + TS_MSG_DATA, 16);
        stl_le_p(payload, got);
        payload_len = 16;
        trace_thunderstorm_reqbufs(device, want, ldl_le_p(m + TS_MSG_DATA + 4), got);
        break;
    }

    case TS_CMD_QUERYBUF: {
        /*
         * Local to the driver for minor 0, forwarded to the card for minor 1 -
         * so the card is what decides where the playback buffers are.
         */
        uint32_t index = ldl_le_p(m + TS_MSG_DATA + TS_VB_INDEX);

        if (index >= TS_MAX_FRAMES) {
            status = TS_EINVAL;
            break;
        }
        memcpy(payload, m + TS_MSG_DATA, 64);
        stl_le_p(payload + TS_VB_OFFSET, TS_PB_BASE + index * TS_PB_STRIDE);
        stl_le_p(payload + TS_VB_LENGTH, TS_PB_STRIDE);
        stl_le_p(payload + TS_VB_PA,
                 pci_get_long(PCI_DEVICE(s)->config + PCI_BASE_ADDRESS_1) +
                 TS_PB_BASE + index * TS_PB_STRIDE);
        payload_len = 64;
        trace_thunderstorm_querybuf(device, index,
                                    TS_PB_BASE + index * TS_PB_STRIDE,
                                    TS_PB_STRIDE);
        break;
    }

    case TS_CMD_QBUF:
        /*
         * who is 0: the host is not waiting, and must not get a reply now.
         * Answering here would be worse than useless - the driver's ISR
         * dispatches on the command word alone, so a reply carrying QBUF is
         * indistinguishable from the completion, and returning it immediately
         * removes the only clock the application has.
         */
        if (device == 0) {
            /*
             * Both the pixel reads and the pixel writes happen here, on the
             * message bottom half, and not in the frame timer.
             *
             * The timer is the clock the application steers by: renderd
             * corrects its frame loop against these completions, and it
             * complains when they wander. Doing ~2.8 MB of DMA and a console
             * update inside the timer callback put 8.8% of ticks past 36 ms
             * and one in a hundred past 64 - almost two frames late - because
             * the callback and the next deadline share the main loop.
             *
             * Presenting is a read of what renderd just composited, so it has
             * to be here anyway. Filling is a write of what the host will read
             * back as captured video, and the buffer is ours from now until we
             * hand it back, so doing it early is equally correct and keeps the
             * timer free to fire on time.
             */
            ts_frame_present(s, m + TS_MSG_DATA);
            ts_frame_stream(s, m + TS_MSG_DATA);
            ts_frame_audio(s, m + TS_MSG_DATA);
            ts_frame_fill(s, m + TS_MSG_DATA);
        }
        ts_frame_push(s, device, m + TS_MSG_DATA);
        ts_stream_start(s);
        reply = false;
        break;

    case TS_CMD_S_CTRL:
        /* Card-side control (brightness and friends). Nothing to model yet. */
        break;

    case TS_CMD_S_INPUT:
    case TS_CMD_S_OUTPUT:
        /*
         * A/V routing. One input and one output exist, so accept whatever is
         * asked for and echo it back - S_* here are _IOWR and the caller reads
         * the value it ended up with.
         */
        s->av_route[cmd == TS_CMD_S_INPUT ? 0 : 1] = ldl_le_p(m + TS_MSG_DATA);
        /* fall through */
    case TS_CMD_G_INPUT:
    case TS_CMD_G_OUTPUT:
        stl_le_p(payload,
                 s->av_route[(cmd == TS_CMD_S_INPUT || cmd == TS_CMD_G_INPUT)
                             ? 0 : 1]);
        payload_len = 4;
        break;

    case TS_CMD_STREAMON:
        /*
         * Unconditional on the real card too: videoControlTask replies 0
         * before it looks at any state, with no format or REQBUFS precondition.
         */
        ts_stream_start(s);
        break;

    case TS_CMD_STREAMOFF:
        break;

    case TS_CMD_SET_AGP:
        /*
         * Fire and forget, and the most interesting message in the protocol -
         * it is the only way the card ever learns where to DMA frames from.
         * Payload is [count, aperture_base, aperture_size, offset, stride,0,0].
         * Phase 4 will keep it; for now, logging it is the point.
         */
        s->agp_count  = ldl_le_p(m + TS_MSG_DATA + 0);
        s->agp_base   = ldl_le_p(m + TS_MSG_DATA + 4);
        s->agp_size   = ldl_le_p(m + TS_MSG_DATA + 8);
        s->agp_offset = ldl_le_p(m + TS_MSG_DATA + 12);
        s->agp_stride = ldl_le_p(m + TS_MSG_DATA + 16);
        trace_thunderstorm_set_agp(s->agp_count, s->agp_base, s->agp_size,
                                   s->agp_offset, s->agp_stride);
        reply = false;
        break;

    default:
        if (TS_IOC_GROUP(cmd) == 'A' && TS_IOC_NR(cmd) == TS_CMD_TERM_NR) {
            /*
             * vterm output. who is 0, so nothing is waiting on a reply.
             *
             * The length in the command word cannot be trusted exactly.
             * tsc_write builds it as
             *
             *     cmd = 0xc00441c2;  cmd |= (resid & 0x1fff) << 16;
             *
             * an OR onto a template that already carries len 4 - so the
             * encoded length is always (actual | 4), and a write whose length
             * has bit 2 clear encodes as four more than it is. The real card
             * cannot distinguish those cases either. Clamp to the message and
             * take it as given; the cost is at most four bytes of stale
             * message payload on a debug console. See the vterm section of
             * notes/thunderstorm-device-spec.md.
             */
            size_t len = TS_IOC_LEN(cmd);

            if (len > TS_MSG_DATA_MAX) {
                len = TS_MSG_DATA_MAX;
            }
            if (len && qemu_chr_fe_backend_connected(&s->vterm_chr)) {
                qemu_chr_fe_write_all(&s->vterm_chr, m + TS_MSG_DATA, len);
            }
            s->vterm_tx += len;
            trace_thunderstorm_i2o_consumed(cmd, device);
            reply = false;
            break;
        }
        /*
         * Answer rather than drop: a dropped 'V'-group message costs the
         * caller a full second in tsleep, and the trace below is how the next
         * phase learns what the application actually needs.
         */
        trace_thunderstorm_i2o_unhandled(cmd, device, TS_IOC_GROUP(cmd),
                                         TS_IOC_NR(cmd), TS_IOC_LEN(cmd));
        status = TS_ENOTTY;
        break;
    }

    if (reply) {
        ts_send(s, cmd, device, who, status, payload, payload_len);
    }
    /* The inbound MFA goes back to the card's free list either way. */
    ts_q_push(&s->in_free, mfa);
}

/*
 * Messages are handled from a bottom half rather than inside the guest's store
 * to the post port. The card is an independent processor: replying inline
 * would make the interrupt arrive before the posting instruction had even
 * retired, so the driver could reach its tsleep after the wakeup it is waiting
 * for. A BH puts the reply where a real card's would be.
 */
static void ts_msg_bh(void *opaque)
{
    ThunderstormState *s = opaque;
    uint32_t mfa;

    while (ts_q_pop(&s->in_post, &mfa)) {
        ts_msg_process(s, mfa);
    }
    ts_update_irq(s);
}

/* ------------------------------------------------------- capture input */

/*
 * What the host sees as incoming video.
 *
 * renderd does not render into a blank buffer: it DQBUFs one the card has
 * filled with captured video, composites its graphics on top, and QBUFs it
 * back for playout. That is why TStormFrameSource and TStormFrameSink exist
 * side by side over one set of buffers. So a buffer handed back unfilled is
 * not neutral - whatever it held shows through wherever the graphics are
 * transparent, which is most of the frame. Left alone it showed renderd its
 * own previous output, smeared.
 *
 * A real card with no input still puts something defined on that plane, so
 * this does too. Bars by default; "black" for a clean key; "none" to go back
 * to leaving buffers alone, which is only useful for reproducing that.
 */
#define TS_ARGB(r, g, b) (0xFF000000u | ((r) << 16) | ((g) << 8) | (b))

/* BT.601 studio-swing RGB -> YCbCr. */
static void ts_rgb_to_ycbcr(int r, int g, int b, int *y, int *cb, int *cr)
{
    *y  = (( 16843 * r +  33030 * g +  6423 * b + 32768) >> 16) + 16;
    *cb = ((-9714  * r + -19071 * g + 28784 * b + 32768) >> 16) + 128;
    *cr = (( 28784 * r + -24103 * g + -4681 * b + 32768) >> 16) + 128;
}

static uint32_t ts_pack_422(int y0, int cb, int y1, int cr)
{
    /*
     * The header calls this "Cb-Y0-Cr-Y1 packed as an unsigned long", but it
     * says so from the card's big-endian point of view, where that means the
     * bytes Cb,Y0,Cr,Y1 in ascending address order. What settles it from the
     * host side is the type renderd hands GL:
     * GL_UNSIGNED_SHORT_8_8_APPLE is 'yuvs', i.e. Y0,Cb,Y1,Cr in memory.
     *
     * So build a little-endian dword whose bytes come out in that order.
     */
    return ((uint32_t)(cr & 0xff) << 24) | ((uint32_t)(y1 & 0xff) << 16) |
           ((uint32_t)(cb & 0xff) << 8)  |  (uint32_t)(y0 & 0xff);
}

static void ts_pattern_bars(uint32_t *p)
{
    /*
     * EIA-style 75% bars: seven bars over the top two thirds, the reverse
     * sequence over the next twelfth, and a bottom section with 100% white and
     * a PLUGE triplet. Recognisable and easy to check by eye - it is not
     * broadcast-accurate SMPTE RP 219 and is not trying to be.
     *
     * Emitted as 4:2:2, which is what the capture side of the buffer is: two
     * pixels per dword, chroma shared between them.
     */
    static const uint8_t top[7][3] = {
        {191, 191, 191}, {191, 191, 0}, {0, 191, 191}, {0, 191, 0},
        {191, 0, 191}, {191, 0, 0}, {0, 0, 191},
    };
    static const uint8_t mid[7][3] = {
        {0, 0, 191}, {16, 16, 16}, {191, 0, 191}, {16, 16, 16},
        {0, 191, 191}, {16, 16, 16}, {191, 191, 191},
    };
    int x, y;
    int top_end = TS_VIDEO_LINES * 2 / 3;
    int mid_end = TS_VIDEO_LINES * 3 / 4;

    for (y = 0; y < TS_VIDEO_LINES; y++) {
        uint32_t *row = p + (size_t)y * TS_VIDEO_IN_DWORDS;

        for (x = 0; x < TS_VIDEO_ACTIVE; x += 2) {
            int rgb[2][3];
            int i, yy[2], cb[2], cr[2];

            for (i = 0; i < 2; i++) {
                int px = x + i;
                int bar = px * 7 / TS_VIDEO_ACTIVE;

                if (y < top_end) {
                    rgb[i][0] = top[bar][0];
                    rgb[i][1] = top[bar][1];
                    rgb[i][2] = top[bar][2];
                } else if (y < mid_end) {
                    rgb[i][0] = mid[bar][0];
                    rgb[i][1] = mid[bar][1];
                    rgb[i][2] = mid[bar][2];
                } else {
                    int v = 16;
                    if (px < TS_VIDEO_ACTIVE * 5 / 28) {
                        v = 16;
                    } else if (px < TS_VIDEO_ACTIVE * 10 / 28) {
                        v = 255;                       /* 100% white */
                    } else if (px < TS_VIDEO_ACTIVE * 20 / 28) {
                        v = 16;
                    } else if (px < TS_VIDEO_ACTIVE * 22 / 28) {
                        v = 8;                         /* PLUGE -4%  */
                    } else if (px < TS_VIDEO_ACTIVE * 24 / 28) {
                        v = 16;                        /* PLUGE  0%  */
                    } else if (px < TS_VIDEO_ACTIVE * 26 / 28) {
                        v = 24;                        /* PLUGE +4%  */
                    }
                    rgb[i][0] = rgb[i][1] = rgb[i][2] = v;
                }
                ts_rgb_to_ycbcr(rgb[i][0], rgb[i][1], rgb[i][2],
                                &yy[i], &cb[i], &cr[i]);
            }
            row[x / 2] = ts_pack_422(yy[0], (cb[0] + cb[1]) / 2,
                                     yy[1], (cr[0] + cr[1]) / 2);
        }
    }
}

/*
 * An orientation probe. The bars are hard to read once the scene has cut them
 * into quads, so this is deliberately unmistakable from any fragment: a luma
 * ramp that is bright at picture-top and dark at picture-bottom, with a red
 * band across the top eight lines and a blue band across the bottom eight.
 *
 * Which way round "picture-top" is, is exactly the question. The vendor header
 * defines line 0 as the first line - blanking index 0 maps to SMPTE line 1 -
 * so the model writes the top of the picture at buffer offset 0, which is what
 * a capture engine does. If it comes out inverted in the composite, then
 * renderd maps the texture the other way up, and a real card must be writing
 * the field bottom-up.
 */
static void ts_pattern_ramp(uint32_t *p)
{
    int x, y;

    for (y = 0; y < TS_VIDEO_LINES; y++) {
        uint32_t *row = p + (size_t)y * TS_VIDEO_IN_DWORDS;
        int luma = 235 - (219 * y) / (TS_VIDEO_LINES - 1);
        int yy, cb, cr;

        if (y < 8) {
            ts_rgb_to_ycbcr(255, 0, 0, &yy, &cb, &cr);        /* top: red  */
        } else if (y >= TS_VIDEO_LINES - 8) {
            ts_rgb_to_ycbcr(0, 0, 255, &yy, &cb, &cr);        /* bottom: blue */
        } else {
            yy = luma; cb = 128; cr = 128;
        }
        for (x = 0; x < TS_VIDEO_IN_DWORDS; x++) {
            row[x] = ts_pack_422(yy, cb, yy, cr);
        }
    }
}

/*
 * The capture plane is stored bottom-up.
 *
 * This was measured, not assumed, and it went the other way first. The vendor
 * header numbers blanking line 0 as SMPTE line 1 and says video in and out
 * share a line ordering, so the model originally wrote the capture plane
 * top-down, the same way the output plane demonstrably is. An orientation
 * probe - a luma ramp bright at picture-top, with a red band across the top
 * eight lines and blue across the bottom eight - came back inverted: blue at
 * the top of the picture, red at the bottom, ramp running the wrong way.
 *
 * renderd uploads this plane straight to a GL texture, and GL puts row 0 at
 * t=0, which is the bottom of a quad drawn the usual way. renderd does not
 * flip it, so a real card must hand it over bottom-up - it would otherwise
 * have been putting upside-down video to air, which a shipping product was
 * not doing. The application is the authority here and the header is not.
 *
 * So: generate the pattern the natural way up, then flip it once, here.
 */
/*
 * The full alignment probe: answers left/right as well as up/down, and shows
 * cropping or a shifted origin at the same time.
 *
 *   - a luma ramp, bright at picture-top, so vertical sense is readable from
 *     any fragment;
 *   - a four-pixel white border, so any missing edge is a crop;
 *   - four distinct corner blocks - top-left RED, top-right GREEN,
 *     bottom-left BLUE, bottom-right YELLOW - so a single corner identifies
 *     the orientation on its own, and a mirrored image is obvious because the
 *     colours swap sides rather than looking plausible;
 *   - a centre cross at x=360, y=240, so a horizontal or vertical offset shows
 *     up as the cross being off-centre.
 *
 * Written the natural way up; ts_pattern_flip() turns it over afterwards,
 * because the capture plane is stored bottom-up (notes/capture-orientation.md).
 */
static void ts_pattern_grid(uint32_t *p)
{
    int x, y;

    for (y = 0; y < TS_VIDEO_LINES; y++) {
        uint32_t *row = p + (size_t)y * TS_VIDEO_IN_DWORDS;

        for (x = 0; x < TS_VIDEO_ACTIVE; x += 2) {
            int r[2], g[2], b[2], i;

            for (i = 0; i < 2; i++) {
                int px = x + i;
                int luma = 200 - (160 * y) / (TS_VIDEO_LINES - 1);
                int corner = 48;

                r[i] = g[i] = b[i] = luma;

                if (px < corner && y < corner) {              /* top-left    */
                    r[i] = 255; g[i] = 0;   b[i] = 0;         /* red         */
                } else if (px >= TS_VIDEO_ACTIVE - corner && y < corner) {
                    r[i] = 0;   g[i] = 255; b[i] = 0;         /* green       */
                } else if (px < corner && y >= TS_VIDEO_LINES - corner) {
                    r[i] = 0;   g[i] = 0;   b[i] = 255;       /* blue        */
                } else if (px >= TS_VIDEO_ACTIVE - corner &&
                           y >= TS_VIDEO_LINES - corner) {
                    r[i] = 255; g[i] = 255; b[i] = 0;         /* yellow      */
                } else if (px < 4 || px >= TS_VIDEO_ACTIVE - 4 ||
                           y < 4 || y >= TS_VIDEO_LINES - 4) {
                    r[i] = g[i] = b[i] = 255;                 /* border      */
                } else if ((px >= TS_VIDEO_ACTIVE / 2 - 1 &&
                            px <= TS_VIDEO_ACTIVE / 2) ||
                           (y >= TS_VIDEO_LINES / 2 - 1 &&
                            y <= TS_VIDEO_LINES / 2)) {
                    r[i] = g[i] = b[i] = 255;                 /* centre cross */
                }
            }
            {
                int y0, cb0, cr0, y1, cb1, cr1;

                ts_rgb_to_ycbcr(r[0], g[0], b[0], &y0, &cb0, &cr0);
                ts_rgb_to_ycbcr(r[1], g[1], b[1], &y1, &cb1, &cr1);
                row[x / 2] = ts_pack_422(y0, (cb0 + cb1) / 2,
                                         y1, (cr0 + cr1) / 2);
            }
        }
    }
}

static void ts_pattern_flip(uint32_t *p)
{
    uint32_t tmp[TS_VIDEO_IN_DWORDS];
    int y;

    for (y = 0; y < TS_VIDEO_LINES / 2; y++) {
        uint32_t *a = p + (size_t)y * TS_VIDEO_IN_DWORDS;
        uint32_t *b = p + (size_t)(TS_VIDEO_LINES - 1 - y) * TS_VIDEO_IN_DWORDS;

        memcpy(tmp, a, sizeof(tmp));
        memcpy(a, b, sizeof(tmp));
        memcpy(b, tmp, sizeof(tmp));
    }
}

static void ts_pattern_init(ThunderstormState *s, Error **errp)
{
    /*
     * Only the active 720 pixels - 360 dwords - of each line need filling.
     * renderd's own logging settles it: the texture is 720x480 read at
     * src_stride 4096, so Mesa never looks past the first 1440 bytes of a row.
     * Filling the whole 4096-byte line was three times the DMA for nothing.
     */
    size_t pixels = (size_t)TS_VIDEO_IN_DWORDS * TS_VIDEO_LINES;
    const char *mode = s->input ? s->input : "bars";

    if (s->input_file) {
        gsize len = 0;
        char *data = NULL;

        if (!g_file_get_contents(s->input_file, &data, &len, NULL)) {
            error_setg(errp, "thunderstorm: cannot read input-file '%s'",
                       s->input_file);
            return;
        }
        if (len != pixels * 4) {
            error_setg(errp, "thunderstorm: input-file '%s' is %zu bytes, "
                       "expected %zu (packed 4:2:2 Cb-Y0-Cr-Y1, %d x %d)",
                       s->input_file, (size_t)len, pixels * 4,
                       TS_VIDEO_ACTIVE, TS_VIDEO_LINES);
            g_free(data);
            return;
        }
        s->pattern = (uint32_t *)data;
        ts_pattern_flip(s->pattern);
        return;
    }

    if (!strcmp(mode, "none")) {
        return;
    }
    s->pattern = g_new0(uint32_t, pixels);
    if (!strcmp(mode, "bars")) {
        ts_pattern_bars(s->pattern);
    } else if (!strcmp(mode, "ramp")) {
        ts_pattern_ramp(s->pattern);
    } else if (!strcmp(mode, "grid")) {
        ts_pattern_grid(s->pattern);
    } else if (!strcmp(mode, "black")) {
        size_t i;
        for (i = 0; i < pixels; i++) {
            s->pattern[i] = ts_pack_422(16, 128, 16, 128); /* 4:2:2 black */
        }
    } else {
        error_setg(errp, "thunderstorm: input must be bars, ramp, grid, black or none");
        g_free(s->pattern);
        s->pattern = NULL;
        return;
    }

    ts_pattern_flip(s->pattern);
}

/* Fill a buffer's video plane the way a capture engine would. */
/*
 * Put captured audio in the frame, the way the card would.
 *
 * TSH_AUDIO is two interleaved channels of signed 24-bit samples, each
 * sign-extended into a dword, A then B; audioCount is the number of A/B
 * PAIRS that are valid. The header says a frame carries "either 215 * 8 or
 * 215 * 7 samples ... 3440 or 3010 longs respectively", i.e. 1720 or 1505
 * pairs. That brackets 48 kHz at 29.97 fps (1601.6 pairs a frame) without
 * matching it exactly, and the cadence that produces the vendor's two
 * numbers is not stated, so this uses a fixed count inside their range and
 * says so rather than inventing a pattern.
 *
 * This exists because the project has never had a positive control on the
 * audio capture path. The video side has had one since Phase 5 (input=bars),
 * and it is what proved the capture plane was real.
 */
static void ts_fill_audio(ThunderstormState *s, hwaddr base,
                          uint32_t live_pairs)
{
    const char *mode = s->audio ? s->audio : "off";
    /*
     * The NTSC 48 kHz cadence. Five frames at 30000/1001 fps span exactly
     * 8008 sample pairs, so 1602/1601/1602/1601/1602 is 48000 Hz on the nose
     * and a flat 1600 would be 47952 - 0.1% slow, which is inaudible for a
     * second and a drifting hour. It also sits inside the vendor header's
     * own range ("215 * 8 or 215 * 7 samples", i.e. 1505..1720 pairs) and
     * gives the varying count the header says a real card produces.
     */
    static const uint32_t cadence[5] = { 1602, 1601, 1602, 1601, 1602 };
    uint32_t pairs = cadence[s->audio_cadence++ % 5];
    uint8_t buf[TS_AUDIO_SAMPLES * 4];
    uint8_t cnt[4];
    bool tone;
    uint32_t i;

    /*
     * Take exactly this frame's worth off the ring, on the CARD's clock.
     *
     * This is the only place capture audio advances, which is what makes it
     * continuous: the reader thread's arrival pattern cannot skip or repeat a
     * block any more, it can only make the ring longer or shorter.
     *
     * Prime before starting. Emitting the instant the first bytes land leaves
     * no slack at all, so the very next jitter underruns, and an underrun is
     * a click. A few frames buffered costs a few frames of latency and takes
     * the whole class of glitch away.
     */
    if (s->input_audio) {
        size_t want = (size_t)pairs * 2 * 4;

        qemu_mutex_lock(&s->in_lock);
        if (!s->in_aprimed && s->in_aring_n >= want * TS_IN_APRIME) {
            s->in_aprimed = true;
        }
        if (s->in_aprimed && s->in_aring_n >= want) {
            memcpy(s->in_audio_stage, s->in_aring, want);
            memmove(s->in_aring, s->in_aring + want, s->in_aring_n - want);
            s->in_aring_n -= want;
            live_pairs = pairs;
        } else if (s->in_aprimed) {
            /* Ran dry. Re-prime rather than dribble one short block after
             * another, which would click on every frame instead of once. */
            s->in_aprimed = false;
            s->in_aunder++;
        }
        qemu_mutex_unlock(&s->in_lock);
    }

    /*
     * Live audio wins over the synthetic modes, and carries even when
     * audio=off: if a real source is attached, "off" is about the tone
     * generator, not about muting the input.
     */
    if (live_pairs) {
        uint32_t k;

        if (live_pairs > TS_AUDIO_SAMPLES / 2) {
            live_pairs = TS_AUDIO_SAMPLES / 2;
        }
        /*
         * The producer sends s32le at full scale; the card carries signed
         * 24-bit sign-extended into a dword. Shift rather than scale, so a
         * full-scale input is exactly full-scale here.
         */
        for (k = 0; k < live_pairs * 2; k++) {
            int32_t v = (int32_t)ldl_le_p(s->in_audio_stage + k * 4);
            stl_le_p(buf + k * 4, (uint32_t)(v >> 8));
        }
        pci_dma_write(PCI_DEVICE(s), base + TS_AUDIO_OFF, buf,
                      live_pairs * 2 * 4);
        stl_le_p(cnt, live_pairs);
        pci_dma_write(PCI_DEVICE(s), base + TS_AUDIO_COUNT_OFF, cnt,
                      sizeof(cnt));

        /*
         * The same positive control the synthetic path has, and for the same
         * reason: a probe that reads zero is indistinguishable from never
         * having written. Read our own write back and say what is in the frame.
         */
        if (!s->audio_selftest) {
            uint8_t back[64];
            uint32_t j, peak = 0;

            s->audio_selftest = true;
            pci_dma_read(PCI_DEVICE(s), base + TS_AUDIO_OFF, back, sizeof(back));
            for (j = 0; j + 4 <= sizeof(back); j += 4) {
                int32_t bv = (int32_t)ldl_le_p(back + j);
                uint32_t mag = bv < 0 ? (uint32_t)-bv : (uint32_t)bv;

                if (mag > peak) {
                    peak = mag;
                }
            }
            warn_report("thunderstorm: LIVE audio self-test: %u pairs written "
                        "at +%d, first 16 samples peak %u",
                        live_pairs, TS_AUDIO_OFF, peak);
        }
        return;
    }

    if (!strcmp(mode, "off")) {
        return;
    }
    tone = !strcmp(mode, "tone");

    memset(buf, 0, sizeof(buf));
    for (i = 0; i < pairs; i++) {
        int32_t v = 0;

        if (tone) {
            /* 1 kHz at 48 kHz, a third of full scale for 24-bit. */
            v = (int32_t)(2796202.0 * sin(s->audio_phase));
            s->audio_phase += 2.0 * M_PI * 1000.0 / 48000.0;
            if (s->audio_phase > 2.0 * M_PI) {
                s->audio_phase -= 2.0 * M_PI;
            }
        }
        /* A then B, both channels the same. Sign-extended into a dword. */
        stl_le_p(buf + (i * 2 + 0) * 4, (uint32_t)v);
        stl_le_p(buf + (i * 2 + 1) * 4, (uint32_t)v);
    }

    pci_dma_write(PCI_DEVICE(s), base + TS_AUDIO_OFF, buf, pairs * 2 * 4);
    stl_le_p(cnt, pairs);
    pci_dma_write(PCI_DEVICE(s), base + TS_AUDIO_COUNT_OFF, cnt, sizeof(cnt));

    /*
     * Positive control, once. A probe that reads zero is exactly what this
     * project has been caught by before, so read our own write straight back
     * and say what is actually in the frame. Without this, "the host returned
     * silence" and "we never wrote a tone" look identical.
     */
    if (!s->audio_selftest) {
        uint8_t back[64];
        uint32_t k, peak = 0;

        s->audio_selftest = true;
        pci_dma_read(PCI_DEVICE(s), base + TS_AUDIO_OFF, back, sizeof(back));
        for (k = 0; k + 4 <= sizeof(back); k += 4) {
            int32_t v = (int32_t)ldl_le_p(back + k);
            uint32_t mag = v < 0 ? (uint32_t)-v : (uint32_t)v;
            if (mag > peak) {
                peak = mag;
            }
        }
        warn_report("thunderstorm: audio fill self-test: mode %s, %u pairs "
                    "written at +%d, first 16 samples peak %u",
                    mode, pairs, TS_AUDIO_OFF, peak);
    }
}


/* ------------------------------------------------- live capture input */

/*
 * Audio read-ahead, in bytes: sixteen frames at the longest cadence entry.
 * Only has to be bigger than the producer's burstiness, which for ffmpeg
 * writing two pipes from one thread is a few frames.
 */
#define TS_IN_ARING (16 * 1602 * 2 * 4)

/*
 * Pull whatever audio is available right now, without blocking. Returns
 * false only on EOF or a real error - "nothing there" is success.
 */
static bool ts_in_drain_audio(ThunderstormState *s, uint8_t *ring, size_t *n)
{
    while (*n < TS_IN_ARING) {
        ssize_t r = read(s->in_afd, ring + *n, TS_IN_ARING - *n);

        if (r > 0) {
            *n += (size_t)r;
        } else if (r == 0) {
            return false;                       /* writer closed */
        } else if (errno == EINTR) {
            continue;
        } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return true;                        /* drained, not broken */
        } else {
            return false;
        }
    }
    return true;
}

/*
 * How many whole video frames to hold while waiting for the audio that goes
 * with them. Both directions need slack: the producer writes two pipes and
 * its interleaving is not tight, so either stream can run ahead.
 */

/*
 * The reader thread's one loop.
 *
 * NEITHER STREAM MAY EVER WAIT ON THE OTHER. That is the whole design, and it
 * was arrived at by getting it wrong twice in both directions:
 *
 *   1. Reading a whole 691 KB video frame before touching audio. ffmpeg
 *      writes both pipes from one thread, so it blocked writing audio into a
 *      pipe nobody was draining, stopped producing video, and the video read
 *      never finished.
 *   2. Fixing that by draining audio while waiting for video - but then
 *      waiting for audio with a poll() on the audio fd alone. Exactly the
 *      same deadlock mirrored: ffmpeg blocked on video, this thread waiting
 *      for audio that ffmpeg could not get to.
 *
 * So: poll both, read whatever is ready into its own buffer, and publish a
 * frame only once a whole frame AND its audio are in hand. Nothing here ever
 * blocks on one descriptor while the other has data.
 *
 * If one stream runs more than the buffers ahead of the other the producer is
 * skewed beyond anything rate matching can fix, and the oldest buffered data
 * is dropped rather than stalling. For a live capture plane dropping is the
 * right answer and wedging never is.
 */
static void ts_input_run(ThunderstormState *s)
{
    size_t vbytes = (size_t)TS_VIDEO_IN_DWORDS * TS_VIDEO_LINES * 4;
    uint64_t skew = 0;
    uint8_t *vslot[TS_IN_VSLOTS];
    uint8_t *aring = g_malloc(TS_IN_ARING);
    size_t aring_n = 0, vfill = 0;
    int vq_head = 0, vq_n = 0, i;
    bool bad = false;

    for (i = 0; i < TS_IN_VSLOTS; i++) {
        vslot[i] = g_malloc(vbytes);
    }

    while (!qatomic_read(&s->in_stopping) && !bad) {
        struct pollfd pf[2];
        int nf = 0, vi = -1, ai = -1;

        /*
         * Hand the ring whatever audio has arrived. This is decoupled from
         * video publication on purpose: the consumer drains the ring on the
         * card's clock, so the reader's job is only to keep it fed.
         */
        if (aring_n) {
            qemu_mutex_lock(&s->in_lock);
            if (s->in_aring_n + aring_n > s->in_aring_cap) {
                /* Producer is running ahead of the card. Drop the oldest,
                 * which is one glitch, rather than the newest, which would
                 * be a permanent lag. */
                size_t drop = s->in_aring_n + aring_n - s->in_aring_cap;

                memmove(s->in_aring, s->in_aring + drop, s->in_aring_n - drop);
                s->in_aring_n -= drop;
                s->in_aover++;
            }
            memcpy(s->in_aring + s->in_aring_n, aring, aring_n);
            s->in_aring_n += aring_n;
            qemu_mutex_unlock(&s->in_lock);
            aring_n = 0;
        }

        /* Publish everything that is complete before asking for more. */
        while (vq_n > 0) {
            qemu_mutex_lock(&s->in_lock);
            {
                uint32_t *t = s->in_front;

                memcpy(s->in_back, vslot[vq_head], vbytes);
                s->in_front = s->in_back;
                s->in_back = t;
                s->in_valid = true;
                s->in_frames++;
            }
            qemu_mutex_unlock(&s->in_lock);
            vq_head = (vq_head + 1) % TS_IN_VSLOTS;
            vq_n--;
        }

        /*
         * Nothing could be published and a buffer is full, so the two streams
         * are further apart than the buffers allow. Drop the oldest of
         * whichever ran ahead; the alternative is to stop reading it, which
         * blocks the producer and therefore the other stream too.
         */
        if (vq_n == TS_IN_VSLOTS) {
            vq_head = (vq_head + 1) % TS_IN_VSLOTS;
            vq_n--;
            skew++;
            if (skew == 1) {
                warn_report("thunderstorm: capture video is arriving faster "
                            "than the card consumes it; dropping frames");
            }
        }

        if (vq_n < TS_IN_VSLOTS) {
            vi = nf;
            pf[nf].fd = s->in_fd;
            pf[nf].events = POLLIN;
            pf[nf].revents = 0;
            nf++;
        }
        if (s->in_afd >= 0 && aring_n < TS_IN_ARING) {
            ai = nf;
            pf[nf].fd = s->in_afd;
            pf[nf].events = POLLIN;
            pf[nf].revents = 0;
            nf++;
        }
        if (nf == 0) {
            continue;           /* both full; the drops above will free one */
        }
        if (poll(pf, nf, 200) < 0) {
            if (errno == EINTR) {
                continue;
            }
            bad = true;
            break;
        }
        if (ai >= 0 && pf[ai].revents) {
            if (!ts_in_drain_audio(s, aring, &aring_n)) {
                bad = true;
                break;
            }
        }
        if (vi >= 0 && pf[vi].revents) {
            uint8_t *dst = vslot[(vq_head + vq_n) % TS_IN_VSLOTS];
            ssize_t r = read(s->in_fd, dst + vfill, vbytes - vfill);

            if (r > 0) {
                vfill += (size_t)r;
                if (vfill == vbytes) {
                    vfill = 0;
                    vq_n++;
                }
            } else if (r == 0) {
                bad = true;     /* writer closed */
                break;
            } else if (errno != EINTR && errno != EAGAIN &&
                       errno != EWOULDBLOCK) {
                bad = true;
                break;
            }
        }
    }

    for (i = 0; i < TS_IN_VSLOTS; i++) {
        g_free(vslot[i]);
    }
    g_free(aring);

    qemu_mutex_lock(&s->in_lock);
    s->in_valid = false;      /* fall back to the pattern from here on */
    qemu_mutex_unlock(&s->in_lock);
}


static void ts_in_set_nonblock(int fd)
{
    int fl = fcntl(fd, F_GETFL, 0);

    if (fl >= 0) {
        fcntl(fd, F_SETFL, fl | O_NONBLOCK);
    }
}

static void ts_in_widen(int fd, int bytes)
{
#ifdef F_SETPIPE_SZ
    /* Best effort: capped by /proc/sys/fs/pipe-max-size, and not fatal. */
    if (fcntl(fd, F_SETPIPE_SZ, bytes) < 0) {
        /* nothing to do - the default size still works, just with more
         * round trips */
    }
#endif
}

/*
 * Open the pipes and run, reopening if the producer goes away.
 *
 * Opening a FIFO O_RDONLY blocks until a writer appears, which is exactly the
 * behaviour wanted here but is the reason this cannot happen in realize: it
 * would hang VM start until ffmpeg turned up. Here it simply means the card
 * shows colour bars until a source connects, and returns to bars when one
 * disconnects.
 */
static void *ts_input_open_thread(void *opaque)
{
    ThunderstormState *s = opaque;

    while (!qatomic_read(&s->in_stopping)) {
        s->in_fd = open(s->input_pipe, O_RDONLY);
        if (s->in_fd < 0) {
            if (qatomic_read(&s->in_stopping)) {
                break;
            }
            g_usleep(200 * 1000);
            continue;
        }
        if (s->input_audio) {
            s->in_afd = open(s->input_audio, O_RDONLY);
            if (s->in_afd < 0) {
                warn_report("thunderstorm: cannot open input-audio '%s', "
                            "carrying video only", s->input_audio);
            }
        }
        /*
         * Non-blocking only AFTER the open. Opening a FIFO O_RDONLY blocks
         * until a writer appears, which is the behaviour wanted here;
         * O_RDONLY|O_NONBLOCK would return immediately on an unconnected
         * pipe and spin. From here on the reader is poll()-driven and the
         * audio drain needs EAGAIN to know it has emptied the pipe.
         */
        ts_in_set_nonblock(s->in_fd);
        if (s->in_afd >= 0) {
            ts_in_set_nonblock(s->in_afd);
        }
        /*
         * Widen both pipes. A 691 KB video frame through the default 64 KiB
         * is eleven round trips, and the audio pipe is what the producer
         * backs up into while this thread is busy with one of them.
         */
        ts_in_widen(s->in_fd, 1024 * 1024);
        if (s->in_afd >= 0) {
            ts_in_widen(s->in_afd, 256 * 1024);
        }
        warn_report("thunderstorm: capture input connected (%s%s%s)",
                    s->input_pipe, s->in_afd >= 0 ? " + " : "",
                    s->in_afd >= 0 ? s->input_audio : "");

        ts_input_run(s);

        close(s->in_fd);
        s->in_fd = -1;
        if (s->in_afd >= 0) {
            close(s->in_afd);
            s->in_afd = -1;
        }
        if (!qatomic_read(&s->in_stopping)) {
            warn_report("thunderstorm: capture input disconnected after %"
                        PRIu64 " frames; showing the pattern again",
                        s->in_frames);
        }
    }
    return NULL;
}

static void ts_frame_fill(ThunderstormState *s, const uint8_t *vb)
{
    uint32_t index = ldl_le_p(vb + TS_VB_INDEX);
    hwaddr base;
    int row;
    const uint32_t *live = NULL, *src;
    uint32_t live_pairs = 0;

    if (!s->agp_stride) {
        return;
    }
    /*
     * Take the newest live frame if there is one, otherwise the synthetic
     * pattern. Copied out under the lock rather than DMA'd under it: holding
     * the reader out for the length of 480 DMA writes would be the one way
     * this could disturb the frame clock.
     */
    if (s->input_pipe) {
        qemu_mutex_lock(&s->in_lock);
        if (s->in_valid) {
            memcpy(s->in_stage, s->in_front,
                   (size_t)TS_VIDEO_IN_DWORDS * TS_VIDEO_LINES * 4);
            live = s->in_stage;
        } else {
            s->in_underruns++;
        }
        qemu_mutex_unlock(&s->in_lock);
        if (!s->in_reported && live) {
            s->in_reported = true;
            warn_report("thunderstorm: first live frame composited: "
                        "frames=%" PRIu64 " audio_pairs=%u afd=%d",
                        s->in_frames, live_pairs, s->in_afd);
        }
    }
    src = live ? live : s->pattern;
    if (!src) {
        return;
    }
    if (s->agp_count && index >= s->agp_count) {
        return;
    }
    base = (hwaddr)s->agp_base + s->agp_offset + (hwaddr)index * s->agp_stride;

    /*
     * Stamp the frame before filling it. A real card is the house clock for
     * everything above it, and this is the field it has to say so in; leaving
     * it zero is a choice the model made by omission.
     */
    if (s->tstamp && strcmp(s->tstamp, "off")) {
        int64_t host_ns = qemu_clock_get_ns(QEMU_CLOCK_HOST);
        int64_t v;
        uint8_t raw[4];

        if (!strcmp(s->tstamp, "host-ms")) {
            v = host_ns / SCALE_MS;
        } else if (!strcmp(s->tstamp, "host-us")) {
            v = host_ns / 1000;
        } else { /* host-s */
            v = host_ns / NANOSECONDS_PER_SECOND;
        }
        stl_le_p(raw, (uint32_t)v);
        pci_dma_write(PCI_DEVICE(s), base + TS_TSTAMP_OFF, raw, sizeof(raw));
    }

    ts_fill_audio(s, base, live_pairs);

    for (row = 0; row < TS_VIDEO_LINES; row++) {
        pci_dma_write(PCI_DEVICE(s), base + (hwaddr)row * TS_VIDEO_STRIDE,
                      src + (size_t)row * TS_VIDEO_IN_DWORDS,
                      TS_VIDEO_IN_DWORDS * 4);
    }
    s->frames_filled++;
}

/* ------------------------------------------------------- video output */

/*
 * Pull a frame out of AGP memory and put it on the card's console.
 *
 * This is the one place the model looks at pixels. The host tells the card
 * where they live exactly once, in VIDIOC_SET_AGP, as an aperture base plus an
 * offset and a per-frame stride; the v4l2_buffer's index selects which frame.
 * Everything needed to find them is guest-physical, so a plain bus-master read
 * is the whole mechanism - which is also why the aperture had to be real
 * memory rather than a translation (see notes/phase2-agp.md).
 */
/*
 * Write one frame to the output stream: a 16-byte header, then 720x480 raw
 * BGRA, top-down. That is the TSH_FRAME's own layout minus the 304 padding
 * pixels a line, so the only work is dropping the pad.
 *
 * The awkward part is that a frame is 1.38 MB and a pipe holds at most
 * /proc/sys/fs/pipe-max-size, which is 1 MB here - so a frame never fits in
 * one go, and a non-blocking write returns a short count. Treating that as
 * success tears every frame and desynchronises the stream permanently; the
 * viewer showed 1.4 fps of garbage.
 *
 * So the remainder is carried while a writable fd handler drains it. A new
 * frame is never started while the previous one is still going out, which
 * means the reader always sees whole frames in order, and congestion costs
 * whole dropped frames rather than a sheared picture. Nothing here ever
 * blocks: this runs in the main loop, and the console path's own comment
 * records that doing 1.4 MB of work here thirty times a second is what used
 * to make the frame clock jitter.
 */
static void ts_stream_write(void *opaque);

/*
 * Push whatever is left of the frame in flight. Returns true when the pipe
 * is empty of our data and a new frame may be started.
 *
 * The writable handler matters most on hosts such as macOS, which provide no
 * way to enlarge a FIFO. Waiting for the next frame tick before each write
 * would cap throughput at one small pipe-buffer's worth per frame period.
 */
static bool ts_stream_flush(ThunderstormState *s)
{
    ssize_t n;

    if (s->output_fd < 0 || !s->output_pending) {
        return true;
    }
    n = write(s->output_fd, s->output_buf + s->output_off, s->output_pending);
    if (n > 0) {
        s->output_off += (size_t)n;
        s->output_pending -= (size_t)n;
    }
    if (!s->output_pending) {
        s->output_frames++;
        qemu_set_fd_handler(s->output_fd, NULL, NULL, NULL);
        return true;
    }
    return false;
}

static void ts_stream_write(void *opaque)
{
    ThunderstormState *s = opaque;

    ts_stream_flush(s);
}

static void ts_frame_stream(ThunderstormState *s, const uint8_t *vb)
{
    uint32_t index = ldl_le_p(vb + TS_VB_INDEX);
    size_t video = (size_t)TS_VIDEO_ACTIVE * TS_VIDEO_LINES * 4;
    size_t total;
    uint32_t pairs = 0;
    hwaddr base;
    int row;

    if (s->output_fd < 0 || !s->agp_stride) {
        return;
    }

    /* Still flushing the previous frame? Push what we can and skip this one. */
    if (!ts_stream_flush(s)) {
        s->output_dropped++;
        return;
    }

    if (s->agp_count && index >= s->agp_count) {
        return;
    }
    base = (hwaddr)s->agp_base + s->agp_offset + (hwaddr)index * s->agp_stride;

    for (row = 0; row < TS_VIDEO_LINES; row++) {
        pci_dma_read(PCI_DEVICE(s), base + (hwaddr)row * TS_VIDEO_STRIDE,
                     s->output_buf + TS_OUTPUT_HDR +
                     (size_t)row * TS_VIDEO_ACTIVE * 4,
                     TS_VIDEO_ACTIVE * 4);
    }

    /*
     * The audio the application mixed into this frame, straight after the
     * picture. audioCount is whatever the host left; clamp it to the plane
     * because renderd does not (see notes/product-flavors-and-audio.md) and
     * a bad value here would read past the frame.
     */
    {
        uint8_t raw[4];

        pci_dma_read(PCI_DEVICE(s), base + TS_AUDIO_COUNT_OFF, raw, sizeof(raw));
        pairs = ldl_le_p(raw);
        if (pairs > TS_AUDIO_SAMPLES / 2) {
            pairs = TS_AUDIO_SAMPLES / 2;
        }
        if (pairs) {
            pci_dma_read(PCI_DEVICE(s), base + TS_AUDIO_OFF,
                         s->output_buf + TS_OUTPUT_HDR + video,
                         (size_t)pairs * 2 * 4);
        }
    }
    total = TS_OUTPUT_HDR + video + (size_t)pairs * 2 * 4;

    stl_le_p(s->output_buf + 0, TS_OUTPUT_MAGIC);
    stl_le_p(s->output_buf + 4, TS_VIDEO_ACTIVE);
    stl_le_p(s->output_buf + 8, TS_VIDEO_LINES);
    stl_le_p(s->output_buf + 12, (uint32_t)s->output_frames);
    stl_le_p(s->output_buf + 16, pairs);
    stl_le_p(s->output_buf + 20, TS_OUTPUT_RATE);
    stl_le_p(s->output_buf + 24, 0);
    stl_le_p(s->output_buf + 28, 0);

    s->output_off = 0;
    s->output_pending = total;
    if (!ts_stream_flush(s)) {
        qemu_set_fd_handler(s->output_fd, NULL, ts_stream_write, s);
    }
}

static void ts_frame_present(ThunderstormState *s, const uint8_t *vb)
{
    DisplaySurface *surf;
    uint32_t index = ldl_le_p(vb + TS_VB_INDEX);
    hwaddr base;
    uint8_t *dst;
    int row, stride;

    if (!s->con || !s->agp_stride) {
        return;
    }
    if (s->agp_count && index >= s->agp_count) {
        return;
    }

    /*
     * The console is a view for us, not an output the guest depends on, so it
     * does not need every frame. Reading one costs 1.4 MB of DMA plus a full
     * surface update, and doing that 30 times a second in the main loop is a
     * large part of why the frame clock used to jitter.
     */
    if (s->display_rate) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        int64_t period = NANOSECONDS_PER_SECOND / s->display_rate;

        if (s->display_last && now - s->display_last < period) {
            return;
        }
        s->display_last = now;
    }

    base = (hwaddr)s->agp_base + s->agp_offset +
           (hwaddr)index * s->agp_stride;

    surf = qemu_console_surface(s->con);
    if (!surf) {
        return;
    }
    dst = surface_data(surf);
    stride = surface_stride(surf);

    /*
     * Row at a time: the source line is 4096 bytes but only the first 720
     * pixels are active picture, so a single bulk read would drag in 30% more
     * than we show. Alpha is the key signal rather than transparency against
     * anything here, so it is dropped for display.
     */
    for (row = 0; row < TS_VIDEO_LINES; row++) {
        pci_dma_read(PCI_DEVICE(s), base + (hwaddr)row * TS_VIDEO_STRIDE,
                     dst + (size_t)row * stride, TS_VIDEO_ACTIVE * 4);
    }

    s->frames_shown++;
    if (!(s->frames_shown % 300)) {
        trace_thunderstorm_present(s->frames_shown, index, (uint64_t)base,
                                   ldl_le_p(dst + (size_t)(TS_VIDEO_LINES / 2) *
                                            stride + 4 * (TS_VIDEO_ACTIVE / 2)));
    }
    qemu_console_update_full(s->con);
}

/*
 * Look at the audio half of a frame the host has just finished with.
 *
 * Unlike presenting, this is not rate-limited: four bytes per frame is
 * nothing, and the whole point is to catch the first frame that carries
 * anything. The 16 KB audio plane is only read when audioCount says there is
 * something in it.
 */
static void ts_frame_audio(ThunderstormState *s, const uint8_t *vb)
{
    uint32_t index = ldl_le_p(vb + TS_VB_INDEX);
    uint32_t samples[TS_AUDIO_SAMPLES];
    uint32_t count, pairs, peak = 0, i;
    hwaddr base;
    uint8_t raw[4];

    if (!s->agp_stride) {
        return;
    }
    if (s->agp_count && index >= s->agp_count) {
        return;
    }
    base = (hwaddr)s->agp_base + s->agp_offset + (hwaddr)index * s->agp_stride;

    pci_dma_read(PCI_DEVICE(s), base + TS_AUDIO_COUNT_OFF, raw, sizeof(raw));
    count = ldl_le_p(raw);

    /*
     * Where does the app actually put things?
     *
     * The offsets above are arithmetic out of ThunderstormHost.h, and the
     * positive control only proves the DMA read reaches the frame - not that
     * 2074624 is the field the app calls audioCount. Since renderd has now
     * been shown to read its vocal WAVs off disk while audioCount stays zero,
     * the useful question is no longer "is audioCount set" but "is there
     * anything at all past the picture, and where".
     *
     * So once per run, sweep everything between the end of the video plane and
     * the end of the per-frame stride and report which 4 KB blocks are not
     * zero. It costs one 128 KB read per frame until it finds something, and
     * nothing at all afterwards.
     */
    if (!s->audio_mapped) {
        uint8_t *tail = g_malloc(TS_FRAME_TAIL_BYTES);
        size_t off, first = TS_FRAME_TAIL_BYTES, last = 0, nonzero = 0;

        pci_dma_read(PCI_DEVICE(s), base + TS_VIDEO_BYTES, tail,
                     TS_FRAME_TAIL_BYTES);
        for (off = 0; off + 4 <= TS_FRAME_TAIL_BYTES; off += 4) {
            if (ldl_le_p(tail + off)) {
                if (off < first) {
                    first = off;
                }
                last = off;
                nonzero++;
            }
        }
        if (nonzero) {
            s->audio_mapped = true;
            warn_report("thunderstorm: frame tail is not empty: %zu non-zero "
                        "dwords between +%zu and +%zu (frame-relative +%zu..+%zu); "
                        "audio plane is +%d, audioCount +%d",
                        nonzero, first, last,
                        (size_t)TS_VIDEO_BYTES + first,
                        (size_t)TS_VIDEO_BYTES + last,
                        TS_AUDIO_OFF, TS_AUDIO_COUNT_OFF);
        }
        g_free(tail);
    }

    if (!s->audio_probed) {
        /*
         * A positive control, once per run. This probe has only ever read
         * zero, and a probe that has never returned anything else is exactly
         * the kind of thing this project has already been caught by. So on the
         * first frame, read a pixel from the middle of the picture through the
         * same frame-relative DMA path: it is non-zero whenever there is a
         * picture on the card's console, which confirms the read reaches the
         * frame rather than reading past it. Only the offsets differ after
         * that, and those are arithmetic out of ThunderstormHost.h.
         */
        uint8_t px[4];
        uint32_t centre;

        pci_dma_read(PCI_DEVICE(s),
                     base + (hwaddr)(TS_VIDEO_LINES / 2) * TS_VIDEO_STRIDE +
                     4 * (TS_VIDEO_ACTIVE / 2), px, sizeof(px));
        centre = ldl_le_p(px);
        /*
         * Keep trying until there is a picture: the first frames the app
         * queues are black, and a black centre pixel proves nothing.
         */
        if (centre) {
            s->audio_probed = true;
            warn_report("thunderstorm: audio probe live: frame at 0x%" PRIx64
                        ", centre pixel 0x%08x, audioCount %u",
                        (uint64_t)base, centre, count);
        }
    }

    if (!count) {
        return;
    }

    /* count is A/B pairs, so twice that many dwords - clamped to the buffer. */
    pairs = count > TS_AUDIO_SAMPLES / 2 ? TS_AUDIO_SAMPLES / 2 : count;
    pci_dma_read(PCI_DEVICE(s), base + TS_AUDIO_OFF, samples, pairs * 2 * 4);
    for (i = 0; i < pairs * 2; i++) {
        int32_t v = (int32_t)ldl_le_p(&samples[i]);
        uint32_t mag = v < 0 ? -(uint32_t)v : (uint32_t)v;

        if (mag > peak) {
            peak = mag;
        }
    }

    s->audio_frames++;
    if (count > s->audio_count_max) {
        s->audio_count_max = count;
    }
    if (peak > s->audio_peak) {
        s->audio_peak = peak;
    }

    if (!s->audio_seen) {
        s->audio_seen = true;
        /*
         * Loud on purpose. This has been zero for the entire life of the
         * project and a trace point would be missed; a run that produces this
         * line answers the question by itself.
         */
        warn_report("thunderstorm: audio: first non-zero frame - "
                    "audioCount %u pairs, peak sample %u, v4l2 type %d, "
                    "at output frame %" PRIu64, count, peak,
                    (int32_t)ldl_le_p(vb + TS_VB_TYPE), s->frames_shown);
    }

    /*
     * And separately, the first frame whose samples are actually non-zero.
     *
     * The report above fires on the first frame with a non-zero audioCount,
     * which since the model started delivering capture audio is frame 2 of
     * every run - so it says nothing about whether the application ever mixed
     * anything. This is the event that matters for todo #8: renderd writing
     * audio of its own into the frame.
     *
     * v4l2_buffer.type is carried because -1 in that field makes renderd
     * memset the plane unconditionally, bus or no bus.
     */
    if (peak && !s->audio_heard) {
        s->audio_heard = true;
        warn_report("thunderstorm: AUDIO OUT: first frame with non-zero "
                    "samples - %u pairs, peak %u, v4l2 type %d, at output "
                    "frame %" PRIu64, count, peak,
                    (int32_t)ldl_le_p(vb + TS_VB_TYPE), s->frames_shown);
    }
    trace_thunderstorm_audio(s->audio_frames, count, peak);
}

static bool ts_gfx_update(void *opaque)
{
    /* Frames arrive from the guest, not from a refresh tick; nothing to do. */
    return true;
}

static void ts_gfx_invalidate(void *opaque)
{
    ThunderstormState *s = opaque;

    if (s->con) {
        qemu_console_update_full(s->con);
    }
}

static const GraphicHwOps ts_gfx_ops = {
    .invalidate = ts_gfx_invalidate,
    .gfx_update = ts_gfx_update,
};

/* ---------------------------------------------------------- frame loop */

static void ts_frame_push(ThunderstormState *s, uint32_t device, const uint8_t *vb)
{
    TsFrameQueue *q;

    if (device >= TS_MINORS) {
        return;
    }
    q = &s->frames[device];
    if (q->count == TS_QUEUED_MAX) {
        /*
         * The host has more buffers outstanding than the card would hold. Drop
         * the oldest: a real card would simply never have accepted it, and
         * silently growing is worse than visibly losing one.
         */
        q->head = (q->head + 1) % TS_QUEUED_MAX;
        q->count--;
    }
    memcpy(q->buf[(q->head + q->count) % TS_QUEUED_MAX], vb, 64);
    q->count++;
}

/*
 * Stamp v4l2_buffer.timecode, which is the application's clock.
 *
 * renderd turns any non-zero activation time into a packed SMPTE timecode
 * (HH:MM:SS:FF in UTC, one byte each) and fires a layer only when that
 * matches the card's current timecode - same hour and not in the future, or
 * exactly one hour behind. Its current timecode comes from here and nowhere
 * else: it copies these four bytes out of every dequeued capture buffer.
 *
 * Leaving them zero, which this model did until 2026-09-18, pins the
 * application's "now" at 00:00:00:00 forever. Nothing scheduled ever becomes
 * ready, so the entire local segment had to be signalled with time=0 to
 * render at all, and the stock run.pyc path rendered nothing.
 *
 * The polarity is worth stating because it reads backwards: renderd uses
 * these bytes when V4L2_BUF_FLAG_TIMECODE is CLEAR, and falls back to
 * seeding from its own gmtime() and free-running when it is SET.
 *
 *   utc  (default) the machine's RTC clock every frame; renderd stays locked
 *                  to it and cannot drift
 *   drop           the same clock, numbered SMPTE drop-frame, which is what a
 *                  29.97 Hz signal is supposed to carry. Written to remove
 *                  the "frame time drift detected" beat and MEASURED NOT TO:
 *                  9 warnings in 300 s against utc's 7 under matched
 *                  conditions. Kept because it is correct in its own right
 *                  and makes the comparison reproducible, exactly as "free"
 *                  is. notes/drift-warnings.md
 *   free           set the flag instead, so renderd seeds once from its own
 *                  gmtime() and free-runs. See the warnings below.
 *   off            leave it zero - the behaviour before this existed, kept
 *                  so the bug can be reproduced on demand
 *
 * "free" is MEASURED NOT TO WORK - 20 distinct Local_* products against
 * utc's 28, i.e. the LDL-only baseline - and is kept only to make the
 * comparison reproducible. Two reasons from disassembly, and the first is
 * almost certainly why it fails here:
 *
 *  - renderd's `seeded` flag is a ONE-WAY LATCH, cleared only in the
 *    TStormFrameSource constructor. Both branches set it. So if the very
 *    first delivered frame has the flag clear, renderd copies whatever is in
 *    the buffer - zeros - latches, and free-runs from 00:00:00:00 forever;
 *    the wall-clock seed is then unreachable for the life of the process.
 *    That is why this is a device property fixed at realize and must not
 *    become runtime-switchable.
 *  - the free-run increment is 29.97 drop-frame and entirely open-loop: it
 *    advances one frame per delivered frame with no resync, so it runs fast
 *    if frames arrive at exactly 30.000 and slow if they arrive below 29.97.
 *    During start-up the card delivers far fewer than 30 a second, so the
 *    clock seeds and then falls behind the guest's wall clock immediately -
 *    which puts a command stamped "now" in the card's FUTURE, where neither
 *    gate clause can help (the one-hour window only forgives commands that
 *    are behind). That matches what was measured, though it was not
 *    confirmed directly: renderd's "switching input source - time" log line
 *    is emitted before the first frame arrives and so reads 00:00:00-00 in
 *    every mode, working or not. It is not a readout of the card's clock.
 */
static void ts_stamp_timecode(ThunderstormState *s, uint8_t *vb)
{
    const char *mode = s->timecode ? s->timecode : "utc";
    uint32_t flags = ldl_le_p(vb + TS_VB_FLAGS);
    struct timespec ts;
    struct tm g;

    if (!strcmp(mode, "off")) {
        return;
    }
    if (!strcmp(mode, "free")) {
        stl_le_p(vb + TS_VB_FLAGS, flags | V4L2_BUF_FLAG_TIMECODE);
        return;
    }

    /*
     * Use the machine's RTC basis, not raw UTC.
     *
     * renderd builds its command timecode with the GUEST's gmtime(), and the
     * guest's clock comes from the RTC - which run/boot-is1.sh starts with
     * base=localtime, so the guest's "UTC" is the host's local time. Stamping
     * real UTC put us four hours ahead of every command renderd generated,
     * the hour byte never matched, and nothing fired. qemu_get_timedate()
     * honours -rtc base= and keeps the two ends on the same clock whatever it
     * is set to.
     */
    clock_gettime(CLOCK_REALTIME, &ts);
    qemu_get_timedate(&g, 0);

    if (!strcmp(mode, "drop")) {
        /*
         * SMPTE 12M drop-frame numbering, from the same clock.
         *
         * "utc" labels a 29.970 Hz stream on a 30.000 Hz grid, so once per
         * thousand frames the frames byte has to advance by two. Drop-frame
         * is the standard answer to exactly that ratio: number the frames
         * sequentially and skip 00 and 01 at the top of nine minutes in ten,
         * which is 1798 frames a minute, 29.9667 fps, the rate the card
         * actually runs at. Consecutive frames then advance by exactly one
         * everywhere except at the nine defined drops - simulated over an
         * hour of frames, 54 defined drops and no unexpected skips, against
         * utc's 107 unplanned double-steps.
         *
         * IT DOES NOT STOP THE DRIFT WARNINGS. That was the point of writing
         * it and it did not work, so the double-step is not what renderd is
         * complaining about. See notes/drift-warnings.md for the table of
         * runs. The mode is kept because it is a truthful description of a
         * 29.97 Hz signal and because the comparison should stay runnable.
         *
         * The hours/minutes/seconds this produces still track the wall clock
         * - that is the point of drop-frame, and it matters here because
         * renderd's activation gate compares the hour byte against a command
         * timecode it builds from the guest's own gmtime().
         */
        uint64_t ns = ((uint64_t)((g.tm_hour * 60 + g.tm_min) * 60 + g.tm_sec)
                       * NANOSECONDS_PER_SECOND) + (uint64_t)ts.tv_nsec;
        /* real frames since midnight, at 30000/1001 */
        uint64_t f = ns * 30000 / (1001 * (uint64_t)NANOSECONDS_PER_SECOND);
        uint64_t d = f / 17982;            /* whole ten-minute blocks */
        uint64_t m = f % 17982;
        uint64_t n = f + 18 * d + (m >= 2 ? 2 * ((m - 2) / 1798) : 0);

        vb[TS_VB_TC_HOURS]   = (uint8_t)((n / 108000) % 24);
        vb[TS_VB_TC_MINUTES] = (uint8_t)((n % 108000) / 1800);
        vb[TS_VB_TC_SECONDS] = (uint8_t)((n % 1800) / 30);
        vb[TS_VB_TC_FRAMES]  = (uint8_t)(n % 30);
        stl_le_p(vb + TS_VB_TC_TYPE, V4L2_TC_TYPE_30FPS);
        stl_le_p(vb + TS_VB_TC_FLAGS, V4L2_TC_FLAG_DROPFRAME);
        stl_le_p(vb + TS_VB_FLAGS, flags & ~(uint32_t)V4L2_BUF_FLAG_TIMECODE);
        return;
    }

    vb[TS_VB_TC_HOURS]   = (uint8_t)g.tm_hour;
    vb[TS_VB_TC_MINUTES] = (uint8_t)g.tm_min;
    vb[TS_VB_TC_SECONDS] = (uint8_t)g.tm_sec;
    vb[TS_VB_TC_FRAMES]  = (uint8_t)((ts.tv_nsec * 30) / 1000000000);
    stl_le_p(vb + TS_VB_TC_TYPE, V4L2_TC_TYPE_30FPS);
    stl_le_p(vb + TS_VB_TC_FLAGS, 0);
    stl_le_p(vb + TS_VB_FLAGS, flags & ~(uint32_t)V4L2_BUF_FLAG_TIMECODE);
}

/*
 * The value handed back in v4l2_buffer.timestamp.
 *
 * stamp_t is a bare s64 in videodev.h with no stated unit or epoch, and the
 * model's original choice - microseconds since the VM started - is certainly
 * not what a card genlocked to house time would report. Since renderd's
 * activation clock reads zero (notes/local-segment-activation.md) and this is
 * the only time value the card gives it, make the choice switchable so the
 * question can be answered by experiment rather than by guessing once.
 */
static int64_t ts_stamp_now(ThunderstormState *s)
{
    const char *mode = s->stamp ?: "host-ns";

    if (!strcmp(mode, "virtual-us")) {
        return qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
    }
    if (!strcmp(mode, "host-us")) {
        return qemu_clock_get_ns(QEMU_CLOCK_HOST) / 1000;
    }
    if (!strcmp(mode, "host-s")) {
        return qemu_clock_get_ns(QEMU_CLOCK_HOST) / NANOSECONDS_PER_SECOND;
    }
    if (!strcmp(mode, "zero")) {
        return 0;
    }
    /* host-ns: nanoseconds since the epoch, the default. */
    return qemu_clock_get_ns(QEMU_CLOCK_HOST);
}

/* Hand one buffer back per minor per frame period. */
static void ts_frame_tick(void *opaque)
{
    ts_stream_flush((ThunderstormState *)opaque);

    ThunderstormState *s = opaque;
    unsigned device;
    bool any = false;
    uint32_t queued_total = 0;
    int64_t tick_ns;

    for (device = 0; device < TS_MINORS; device++) {
        TsFrameQueue *q = &s->frames[device];
        uint8_t vb[64];
        int64_t now;

        if (!q->count) {
            continue;
        }
        queued_total += q->count;
        memcpy(vb, q->buf[q->head], sizeof(vb));
        q->head = (q->head + 1) % TS_QUEUED_MAX;
        q->count--;

        /*
         * Mark it done and stamp it. RESTART (0x1000) is deliberately never
         * set: it means the DVB-ASI input underflowed, and we have no input.
         */
        stl_le_p(vb + TS_VB_FLAGS,
                 (ldl_le_p(vb + TS_VB_FLAGS) & ~(uint32_t)V4L2_BUF_FLAG_QUEUED) |
                 V4L2_BUF_FLAG_DONE | V4L2_BUF_FLAG_MAPPED);
        stl_le_p(vb + TS_VB_SEQUENCE, q->sequence++);
        now = ts_stamp_now(s);
        stl_le_p(vb + TS_VB_TIMESTAMP, (uint32_t)now);
        stl_le_p(vb + TS_VB_TIMESTAMP + 4, (uint32_t)(now >> 32));
        ts_stamp_timecode(s, vb);

        ts_send(s, TS_CMD_QBUF, device, 0, 0, vb, sizeof(vb));
        s->frames_returned++;
        any = true;
        trace_thunderstorm_frame_return(device, ldl_le_p(vb + TS_VB_INDEX),
                                        q->sequence, q->count);
    }

    if (!any) {
        s->frames_starved++;
    }
    ts_update_irq(s);

    /*
     * Advance the deadline by exactly one frame rather than scheduling from
     * "now". Rescheduling relative to now adds the handler's latency to every
     * period, and this is a genlocked clock: an extra few ms per frame showed
     * up as 24.5 fps against a 29.97 Hz timer. If we have fallen more than a
     * frame behind, give up on catching up and resynchronise, so a stall does
     * not turn into a burst.
     */
    tick_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    trace_thunderstorm_frame_tick(tick_ns - s->last_tick, queued_total);
    if (s->frame_log_f) {
        /*
         * One row per tick. This is the right place to measure the guest from:
         * outside it, on a wall clock, seeing every frame it produces. Short
         * samples have twice produced frame-rate claims that did not survive
         * repetition, because the scene's cost varies enormously with which
         * product is on screen - so log the whole run and take distributions.
         */
        fprintf(s->frame_log_f, "%" PRIu64 ",%" PRId64 ",%" PRId64 ",%u,%d\n",
                s->frames_returned, tick_ns, tick_ns - s->last_tick,
                queued_total, any ? 1 : 0);
    }
    s->last_tick = tick_ns;

    s->next_tick += TS_FRAME_NS;
    if (s->next_tick <= tick_ns) {
        s->next_tick = tick_ns + TS_FRAME_NS;
    }
    timer_mod_ns(s->frame_timer, s->next_tick);
}

static void ts_stream_start(ThunderstormState *s)
{
    if (s->streaming) {
        return;
    }
    s->streaming = true;
    trace_thunderstorm_stream(1, TS_FRAME_NS);
    s->last_tick = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->next_tick = s->last_tick + TS_FRAME_NS;
    timer_mod_ns(s->frame_timer, s->next_tick);
}

/* ------------------------------------------------------------------ tscReg */

static uint64_t ts_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    ThunderstormState *s = opaque;
    uint64_t val = 0;
    uint32_t mfa;

    switch (addr) {
    case TS_REG_DOORBELL_OUT:
        /*
         * The whole 0x8000-block handshake, and this magic, belong to the card's
         * flash boot ROM rather than to vxWorks.bin, so there is nothing to copy
         * and we synthesise it. Leaving it clear is what selects the degraded
         * attach path.
         */
        val = s->present ? TS_MAGIC_TSC7 : 0;
        break;
    case TS_REG_CAUSE:
        val = s->cause;
        break;
    case TS_REG_MASK:
        val = s->mask;
        break;
    case TS_REG_QUEUE_IN: /* inbound free: give the host an MFA to fill */
        if (!s->present || !ts_q_pop(&s->in_free, &mfa)) {
            trace_thunderstorm_i2o_starved("inbound");
            val = TS_QUEUE_EMPTY; /* the driver turns this into EAGAIN */
        } else {
            val = mfa;
            trace_thunderstorm_i2o_get(mfa, s->in_free.count);
        }
        break;
    case TS_REG_QUEUE_OUT: /* outbound post: hand over a message from the card */
        if (!s->present || !ts_q_pop(&s->out_post, &mfa)) {
            val = TS_QUEUE_EMPTY;
        } else {
            val = mfa;
            trace_thunderstorm_i2o_recv(mfa, s->out_post.count);
            ts_update_irq(s); /* draining the port is what deasserts bit 3 */
        }
        break;
    case TS_SDRAM_BOOT_STATUS:
        val = s->boot_status;
        break;
    case TS_SDRAM_VERSION:
        /* SDRAM is byte-transparent, so pre-swap for the big-endian card. */
        val = s->present ? bswap32(s->version) : 0;
        break;
    default:
        if (addr + size <= TS_REG_SHADOW_SIZE) {
            memcpy(&val, s->shadow + addr, size);
        }
        break;
    }

    trace_thunderstorm_reg_read(addr, val, size);
    return val;
}

static void ts_reg_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ThunderstormState *s = opaque;

    if (addr >= TS_FW_LOAD_OFFSET) {
        /*
         * Firmware blob landing area. We accept and discard it — the model does
         * not execute PowerPC code. Count it so bring-up can confirm the driver
         * really pushed ~653 KB through here.
         */
        s->fw_bytes += size;
        if (s->fw_bytes == size) {
            trace_thunderstorm_fw_first_write(addr, val);
        }
        return;
    }

    trace_thunderstorm_reg_write(addr, val, size);

    switch (addr) {
    case TS_REG_DOORBELL_IN:
        /*
         * This is what actually boots the image on real hardware. Report the
         * boot ROM's "running" status so the driver's poll of TS_SDRAM_BOOT_STATUS
         * succeeds. 0x05000000 is big-endian 5, again because SDRAM is
         * byte-transparent.
         */
        trace_thunderstorm_doorbell(val, s->fw_bytes);
        s->boot_status = 0x05000000;
        /*
         * The firmware is now running, and one of the things it does on the
         * way up is bring up the GT-64260's PCI master interface -
         * pci1MapMemory1space() and friends, programmed over the card's local
         * bus. The host driver never touches bus mastering: the guest's
         * PCI_COMMAND for this function reads 0x0103, memory and I/O but no
         * master bit, which would leave every pci_dma_read() returning zeros.
         * So enable it from the card side, which is where it really happens.
         */
        pci_default_write_config(PCI_DEVICE(s), PCI_COMMAND,
                                 pci_get_word(PCI_DEVICE(s)->config + PCI_COMMAND) |
                                 PCI_COMMAND_MASTER, 2);
        break;
    case TS_REG_CAUSE:
        s->cause &= ~(uint32_t)val;  /* R/W1C */
        ts_update_irq(s);
        /* The ack frees OMR0; send the next queued character straight away. */
        ts_vterm_kick(s);
        break;
    case TS_REG_MASK:
        s->mask = val;               /* 1 = MASKED, inverted sense */
        ts_update_irq(s);
        break;
    case TS_REG_DOORBELL_OUT:
    case TS_SDRAM_VERSION:
        /* release_cpu clears both before ringing the doorbell. */
        break;
    case TS_SDRAM_HOST_READY:
        /* Host clears it and never reads it back. Nothing reads it card-side. */
        break;
    case TS_REG_QUEUE_IN: /* inbound post: the host has filled this MFA */
        ts_q_push(&s->in_post, val);
        qemu_bh_schedule(s->msg_bh);
        break;
    case TS_REG_QUEUE_OUT: /* outbound free: the host is done with this MFA */
        ts_q_push(&s->out_free, val);
        trace_thunderstorm_i2o_return(val, s->out_free.count);
        break;
    default:
        if (addr + size <= TS_REG_SHADOW_SIZE) {
            memcpy(s->shadow + addr, &val, size);
        }
        break;
    }
}

static const MemoryRegionOps ts_reg_ops = {
    .read = ts_reg_read,
    .write = ts_reg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

/* ---------------------------------------------------------------- tscGalio */

static uint64_t ts_galio_read(void *opaque, hwaddr addr, unsigned size)
{
    ThunderstormState *s = opaque;
    uint64_t val = (addr == TS_GALIO_INT_GATE) ? s->galio_gate : 0;

    trace_thunderstorm_galio_read(addr, val, size);
    return val;
}

static void ts_galio_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ThunderstormState *s = opaque;

    trace_thunderstorm_galio_write(addr, val, size);
    if (addr == TS_GALIO_INT_GATE) {
        s->galio_gate = val;
        trace_thunderstorm_int_gate((val & TS_GALIO_GATE_BITS) == TS_GALIO_GATE_BITS);
        ts_update_irq(s);
    }
}

static const MemoryRegionOps ts_galio_ops = {
    .read = ts_galio_read,
    .write = ts_galio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

/* ------------------------------------------------------- function 1 regions */

static uint64_t ts_fpga_read(void *opaque, hwaddr addr, unsigned size)
{
    trace_thunderstorm_fpga_read(addr, size);
    return 0;
}

static void ts_fpga_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    trace_thunderstorm_fpga_write(addr, val, size);
}

static const MemoryRegionOps ts_fpga_ops = {
    .read = ts_fpga_read,
    .write = ts_fpga_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

/* The CPLD carries the PPC reset line: a byte at 0x13, pulsed 0 -> 1 -> 0. */
#define TS_CPLD_RESET 0x13

static uint64_t ts_cpld_read(void *opaque, hwaddr addr, unsigned size)
{
    ThunderstormFn1State *s = opaque;
    uint64_t val = 0;

    if (addr + size <= TS_BAR_CPLD_SIZE) {
        memcpy(&val, s->cpld_regs + addr, size);
    }
    trace_thunderstorm_cpld_read(addr, val, size);
    return val;
}

static void ts_cpld_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ThunderstormFn1State *s = opaque;

    trace_thunderstorm_cpld_write(addr, val, size);
    if (addr == TS_CPLD_RESET) {
        trace_thunderstorm_cpu_reset(val & 1);
    }
    if (addr + size <= TS_BAR_CPLD_SIZE) {
        memcpy(s->cpld_regs + addr, &val, size);
    }
}

static const MemoryRegionOps ts_cpld_ops = {
    .read = ts_cpld_read,
    .write = ts_cpld_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static uint64_t ts_flash_read(void *opaque, hwaddr addr, unsigned size)
{
    trace_thunderstorm_flash_read(addr, size);
    /*
     * The driver falls back to copying firmware out of flash if it cannot read
     * the file, and aborts if the first word reads 0x00FFFFFF (erased). We have
     * no flash image, so report erased and let it give up cleanly.
     */
    return (addr == 0x100000) ? 0x00FFFFFF : 0;
}

static void ts_flash_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    trace_thunderstorm_flash_write(addr, val, size);
}

static const MemoryRegionOps ts_flash_ops = {
    .read = ts_flash_read,
    .write = ts_flash_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

/* ----------------------------------------------------------------- realize */

static void thunderstorm_fn1_realize(PCIDevice *pdev, Error **errp)
{
    ThunderstormFn1State *s = THUNDERSTORM_FN1(pdev);

    memory_region_init_io(&s->fpga, OBJECT(s), &ts_fpga_ops, s,
                          "thunderstorm.fpga", TS_BAR_FPGA_SIZE);
    memory_region_init_io(&s->cpld, OBJECT(s), &ts_cpld_ops, s,
                          "thunderstorm.cpld", TS_BAR_CPLD_SIZE);
    memory_region_init_io(&s->flash, OBJECT(s), &ts_flash_ops, s,
                          "thunderstorm.flash", TS_BAR_FLASH_SIZE);

    pci_register_bar(pdev, TS_BAR_0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->fpga);
    pci_register_bar(pdev, TS_BAR_1, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->cpld);
    pci_register_bar(pdev, TS_BAR_4, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->flash);
}

static void thunderstorm_realize(PCIDevice *pdev, Error **errp)
{
    ERRP_GUARD();
    ThunderstormState *s = THUNDERSTORM(pdev);
    PCIBus *bus = pci_get_bus(pdev);
    Error *local_err = NULL;

    if (PCI_FUNC(pdev->devfn) != 0) {
        error_setg(errp, "thunderstorm must be function 0 of its slot");
        return;
    }

    memory_region_init_io(&s->reg, OBJECT(s), &ts_reg_ops, s,
                          "thunderstorm.reg", TS_BAR_REG_SIZE);
    /*
     * tscMem is the card's second SDRAM bank: the message pool, the IPC arena,
     * and whatever userspace mmaps. Backing it with RAM rather than MMIO
     * callbacks is both the only way it can be fast enough and the only way it
     * can be mmapped; the interesting traffic is traced at the message level
     * instead, which is the layer that actually means something.
     */
    memory_region_init_ram(&s->mem, OBJECT(s), "thunderstorm.mem",
                           TS_BAR_MEM_SIZE, errp);
    if (*errp) {
        return;
    }
    s->mem_ram = memory_region_get_ram_ptr(&s->mem);
    memory_region_init_io(&s->galio, OBJECT(s), &ts_galio_ops, s,
                          "thunderstorm.galio", TS_BAR_GALIO_SIZE);

    pci_register_bar(pdev, TS_BAR_0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->reg);
    pci_register_bar(pdev, TS_BAR_1, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mem);
    pci_register_bar(pdev, TS_BAR_4, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->galio);

    pci_config_set_interrupt_pin(pdev->config, 1);

    s->msg_bh = qemu_bh_new_guarded(ts_msg_bh, s,
                                    &DEVICE(s)->mem_reentrancy_guard);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ts_frame_tick, s);

    /*
     * Live capture input. The pipes are opened O_RDONLY, which BLOCKS on a
     * FIFO until a writer appears - so they are opened on the reader thread,
     * not here. Opening them in realize would hang the whole VM start until
     * ffmpeg turned up.
     */
    s->in_fd = s->in_afd = -1;
    if (s->input_pipe) {
        size_t vbytes = (size_t)TS_VIDEO_IN_DWORDS * TS_VIDEO_LINES * 4;

        s->in_front = g_malloc0(vbytes);
        s->in_back = g_malloc0(vbytes);
        s->in_stage = g_malloc0(vbytes);
        /* Half a second of slack: long enough to ride out host scheduling
         * and the producer's burstiness, short enough not to be heard as
         * latency against the video. */
        s->in_aring_cap = (size_t)48000 / 2 * 2 * 4;
        s->in_aring = g_malloc0(s->in_aring_cap);
        s->in_audio_stage = g_malloc0(TS_AUDIO_SAMPLES * 4);
        qemu_mutex_init(&s->in_lock);
        s->in_stopping = false;
        qemu_thread_create(&s->in_thread, "tsc-input", ts_input_open_thread, s,
                           QEMU_THREAD_JOINABLE);
        s->in_running = true;
    }

    /*
     * vterm. Harmless when no chardev is attached: the guest's writes are
     * counted and dropped exactly as before, and nothing ever raises cause
     * bit 0, which is the state every run of this project has been in.
     */
    qemu_chr_fe_set_handlers(&s->vterm_chr, ts_vterm_can_receive,
                             ts_vterm_receive, NULL, NULL, s, NULL, true);

    /*
     * A second console showing what the card would be putting to air. Only
     * when the board reports present: the degraded path has no frames, and an
     * extra blank display would just be confusing.
     */
    if (s->present) {
        ts_pattern_init(s, errp);
        if (*errp) {
            return;
        }
    }

    if (s->frame_log) {
        s->frame_log_f = fopen(s->frame_log, "w");
        if (!s->frame_log_f) {
            error_setg_errno(errp, errno, "thunderstorm: cannot open frame-log '%s'",
                             s->frame_log);
            return;
        }
        /* Buffer generously: a write per tick must not become the jitter. */
        setvbuf(s->frame_log_f, NULL, _IOFBF, 1 << 16);
        fprintf(s->frame_log_f, "frame,tick_ns,delta_ns,queued,returned\n");
    }

    if (s->present && s->display) {
        s->con = qemu_graphic_console_create(DEVICE(s), 0, &ts_gfx_ops, s);
        qemu_console_resize(s->con, TS_VIDEO_ACTIVE, TS_VIDEO_LINES);
    }

    /*
     * tsc_attach runs per function and returns immediately while either function
     * is missing, so a single-function device probes but never attaches. Create
     * function 1 here rather than making the user remember to.
     */
    /*
     * QEMU gates "may this slot hold other functions?" on function 0's
     * cap_present bit, not on the config-space header byte, and it checks when
     * function 1 realizes. Set both: the bit for QEMU's check, the header byte
     * so the guest's PCI scan actually looks at function 1.
     */
    pdev->cap_present |= QEMU_PCI_CAP_MULTIFUNCTION;
    pdev->config[PCI_HEADER_TYPE] |= PCI_HEADER_TYPE_MULTI_FUNCTION;

    s->fn1 = pci_new(pdev->devfn + 1, TYPE_THUNDERSTORM_FN1);
    pci_realize_and_unref(s->fn1, bus, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    s->cmds_seen = g_hash_table_new(g_direct_hash, g_direct_equal);

    /*
     * The output stream. A FIFO is created if the path does not exist, so
     * starting a viewer is one command either side and in either order.
     *
     * O_RDWR rather than O_WRONLY: opening the write end of a FIFO with no
     * reader fails with ENXIO, and holding a read end ourselves also means a
     * viewer that quits cannot deliver EPIPE mid-frame. Writes then simply
     * fill the pipe and fail with EAGAIN, which is counted and dropped.
     */
    s->output_fd = -1;
    if (s->output && *s->output) {
        struct stat st;

        if (stat(s->output, &st) < 0 && mkfifo(s->output, 0666) < 0) {
            error_setg_errno(errp, errno, "thunderstorm: cannot create %s",
                             s->output);
            return;
        }
        s->output_fd = open(s->output, O_RDWR | O_NONBLOCK);
        if (s->output_fd < 0) {
            error_setg_errno(errp, errno, "thunderstorm: cannot open %s",
                             s->output);
            return;
        }
        s->output_buf = g_malloc(TS_OUTPUT_HDR +
                                 TS_VIDEO_ACTIVE * TS_VIDEO_LINES * 4 +
                                 TS_AUDIO_SAMPLES * 4);
        /*
         * Make the pipe big enough to hold a whole frame with room to spare,
         * or every frame is dropped by the space check above. The cap is
         * /proc/sys/fs/pipe-max-size; take whatever we are given.
         */
        {
#if defined(F_SETPIPE_SZ) && defined(F_GETPIPE_SZ)
            /*
             * Take the biggest pipe the kernel will give us, down to the
             * 64 KiB default. /proc/sys/fs/pipe-max-size caps this at 1 MiB
             * on a stock Linux, which is less than one 1.38 MB frame - so a
             * frame always takes more than one write and the remainder is
             * carried, see ts_frame_stream().
             *
             * Getting the loop bound wrong here is expensive and silent: an
             * earlier version stopped before trying 1 MiB, left the pipe at
             * its 64 KiB default, and delivered 1.4 frames a second while the
             * card was doing thirty.
             */
            size_t want;

            for (want = 4 * 1024 * 1024; want >= 64 * 1024; want /= 2) {
                if (fcntl(s->output_fd, F_SETPIPE_SZ, (int)want) >= 0) {
                    break;
                }
            }
            s->output_pipe_sz = fcntl(s->output_fd, F_GETPIPE_SZ);
            if (s->output_pipe_sz < 0) {
                s->output_pipe_sz = 0;
            }
#else
            /* Pipe-size fcntls are Linux-specific. */
            s->output_pipe_sz = 0;
#endif
            trace_thunderstorm_output_pipe(s->output_pipe_sz);
        }
        trace_thunderstorm_output_open(s->output, TS_VIDEO_ACTIVE,
                                       TS_VIDEO_LINES);
    }

    trace_thunderstorm_realize(PCI_SLOT(pdev->devfn), s->present, s->version);
}

static void thunderstorm_reset(DeviceState *dev)
{
    ThunderstormState *s = THUNDERSTORM(dev);
    int i;

    memset(s->shadow, 0, sizeof(s->shadow));
    s->cause = 0;
    s->mask = 0xFFFFFFFF; /* everything masked; 1 = MASKED */
    s->boot_status = 0;
    s->galio_gate = 0;
    s->fw_bytes = 0;
    s->irq_level = false;
    s->streaming = false;
    s->next_tick = 0;
    s->last_tick = 0;
    s->frames_returned = 0;
    s->frames_starved = 0;
    s->audio_frames = 0;
    s->audio_count_max = 0;
    s->audio_peak = 0;
    s->audio_seen = false;
    s->audio_probed = false;
    s->audio_mapped = false;
    memset(s->frames, 0, sizeof(s->frames));
    if (s->frame_timer) {
        timer_del(s->frame_timer);
    }

    ts_q_reset(&s->in_free);
    ts_q_reset(&s->in_post);
    ts_q_reset(&s->out_free);
    ts_q_reset(&s->out_post);
    for (i = 0; i < TS_MSG_COUNT; i++) {
        ts_q_push(&s->in_free, TS_POOL_IN_BASE + i * TS_MSG_SIZE);
        ts_q_push(&s->out_free, TS_POOL_OUT_BASE + i * TS_MSG_SIZE);
    }
}

static const Property thunderstorm_properties[] = {
    /*
     * Default off: report the board as not ready, which takes the degraded
     * attach path — all nine /dev nodes, no AGP, no interrupt, no I2O.
     */
    DEFINE_PROP_BOOL("present", ThunderstormState, present, false),
    /*
     * 1.26.18, encoded major:minor:patch16 the way tsc.ko decodes 0x8008.
     *
     * This is not a free choice. /usr/local/etc/rc.d/yyy_flash.sh compares
     * hw.tsc.version.* against /usr/local/tsc/version and, on a mismatch,
     * reflashes the card and reboots - so reporting anything else puts the
     * unit in a reflash loop at every boot. (0.0.0 is the other safe value:
     * the script reads that as "no card" and exits.) 1.26.18 is also the
     * truthful answer, since /usr/local/etc/vxWorks.bin - the blob the driver
     * uploads - is byte-identical to the 1.26.18 package's.
     */
    DEFINE_PROP_UINT32("version", ThunderstormState, version, 0x011a0012),
    /* Present the card's video output as a second QEMU console. */
    DEFINE_PROP_BOOL("display", ThunderstormState, display, true),
    /*
     * What the capture side hands the host, since there is no DVB-ASI feed:
     * "bars", "ramp" and "grid" (orientation and alignment probes),
     * "black", or "none".
     *
     * Default "none" until the X-server crash noted in notes/phase5-result.md
     * is understood: filling is correct in principle and demonstrably reaches
     * memory, but with it on the session dies shortly after a render script
     * loads, and a default that takes the app down is the wrong default.
     */
    DEFINE_PROP_STRING("input", ThunderstormState, input),
    DEFINE_PROP_STRING("input-file", ThunderstormState, input_file),
    /*
     * Live capture input. See notes/av-io-design.md. Video is raw yuyv422,
     * 720x480, already flipped bottom-up, one frame per 30000/1001 period;
     * audio is raw s32le 48 kHz stereo, paired by count. Colour bars (or
     * whatever input= selects) show whenever a frame has not arrived.
     */
    DEFINE_PROP_STRING("input-pipe", ThunderstormState, input_pipe),
    DEFINE_PROP_STRING("input-audio", ThunderstormState, input_audio),
    /*
     * Console refresh rate. The card runs at 29.97; the view does not need to.
     * 0 means every frame, which is what used to destabilise the frame clock.
     */
    DEFINE_PROP_UINT32("display-rate", ThunderstormState, display_rate, 10),
    /* CSV of frame-clock intervals; see notes/frame-clock.md. */
    DEFINE_PROP_STRING("stamp", ThunderstormState, stamp),
    DEFINE_PROP_STRING("timecode", ThunderstormState, timecode),
    DEFINE_PROP_STRING("audio", ThunderstormState, audio),
    DEFINE_PROP_STRING("tstamp", ThunderstormState, tstamp),
    DEFINE_PROP_STRING("frame-log", ThunderstormState, frame_log),
    DEFINE_PROP_STRING("output", ThunderstormState, output),
    /* The card's debug console (minor 2). Any QEMU chardev. */
    DEFINE_PROP_CHR("vterm", ThunderstormState, vterm_chr),
};

static const VMStateDescription vmstate_ts_queue = {
    .name = "thunderstorm/queue",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(mfa, TsQueue, TS_MSG_COUNT),
        VMSTATE_UINT32(head, TsQueue),
        VMSTATE_UINT32(count, TsQueue),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ts_frame_queue = {
    .name = "thunderstorm/frames",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_2DARRAY(buf, TsFrameQueue, TS_QUEUED_MAX, 64),
        VMSTATE_UINT32(head, TsFrameQueue),
        VMSTATE_UINT32(count, TsFrameQueue),
        VMSTATE_UINT32(sequence, TsFrameQueue),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_thunderstorm = {
    .name = "thunderstorm",
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, ThunderstormState),
        VMSTATE_UINT32(cause, ThunderstormState),
        VMSTATE_UINT32(mask, ThunderstormState),
        VMSTATE_UINT32(boot_status, ThunderstormState),
        VMSTATE_UINT32(galio_gate, ThunderstormState),
        VMSTATE_BOOL(irq_level, ThunderstormState),
        VMSTATE_STRUCT(in_free, ThunderstormState, 1, vmstate_ts_queue, TsQueue),
        VMSTATE_STRUCT(in_post, ThunderstormState, 1, vmstate_ts_queue, TsQueue),
        VMSTATE_STRUCT(out_free, ThunderstormState, 1, vmstate_ts_queue, TsQueue),
        VMSTATE_STRUCT(out_post, ThunderstormState, 1, vmstate_ts_queue, TsQueue),
        VMSTATE_BOOL(streaming, ThunderstormState),
        VMSTATE_TIMER_PTR(frame_timer, ThunderstormState),
        VMSTATE_STRUCT_ARRAY(frames, ThunderstormState, TS_MINORS, 1,
                             vmstate_ts_frame_queue, TsFrameQueue),
        VMSTATE_UINT32(agp_count, ThunderstormState),
        VMSTATE_UINT32(agp_base, ThunderstormState),
        VMSTATE_UINT32(agp_size, ThunderstormState),
        VMSTATE_UINT32(agp_offset, ThunderstormState),
        VMSTATE_UINT32(agp_stride, ThunderstormState),
        VMSTATE_UINT32_ARRAY(av_route, ThunderstormState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void thunderstorm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = thunderstorm_realize;
    k->vendor_id = TS_VENDOR_ID;
    k->device_id = TS_DEVICE_ID;
    k->class_id = PCI_CLASS_MULTIMEDIA_VIDEO;
    k->revision = 1;

    dc->desc = "TWC/Wind River Thunderstorm video overlay card";
    device_class_set_legacy_reset(dc, thunderstorm_reset);
    device_class_set_props(dc, thunderstorm_properties);
    dc->vmsd = &vmstate_thunderstorm;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static void thunderstorm_fn1_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = thunderstorm_fn1_realize;
    k->vendor_id = TS_VENDOR_ID;
    k->device_id = TS_DEVICE_ID;
    k->class_id = PCI_CLASS_MULTIMEDIA_VIDEO;
    k->revision = 1;

    dc->desc = "Thunderstorm function 1 (FPGA/CPLD/flash)";
    /* Created automatically by function 0; not independently useful. */
    dc->user_creatable = false;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo thunderstorm_types[] = {
    {
        .name          = TYPE_THUNDERSTORM,
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(ThunderstormState),
        .class_init    = thunderstorm_class_init,
        .interfaces    = (const InterfaceInfo[]) {
            { INTERFACE_CONVENTIONAL_PCI_DEVICE },
            { },
        },
    },
    {
        .name          = TYPE_THUNDERSTORM_FN1,
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(ThunderstormFn1State),
        .class_init    = thunderstorm_fn1_class_init,
        .interfaces    = (const InterfaceInfo[]) {
            { INTERFACE_CONVENTIONAL_PCI_DEVICE },
            { },
        },
    },
};

DEFINE_TYPES(thunderstorm_types)
