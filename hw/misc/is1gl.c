/*
 * is1gl - GL command transport for the virtualised IntelliStar 1.
 *
 * The IntelliStar renders with fixed-function OpenGL through a 2003 Mesa,
 * in software, and cannot reach NTSC rate: a busy frame costs ~156 ms
 * against a 33.37 ms budget, of which ~112 ms is software fragment work and
 * ~44 ms is the framebuffer readback plus indirect-GLX marshalling.
 *
 * This device is the host half of the answer. A replacement libGL in the
 * guest writes GL calls as records into a ring in AGP aperture memory and
 * rings a doorbell here; this device replays them against the host's GL and
 * writes the finished frame straight into the Thunderstorm's TSH_FRAME, so
 * the readback never crosses a socket. See notes/gl-acceleration-plan.md.
 *
 * Phase 1: the transport, carrying nothing. Ring mapping, a render thread,
 * the record parser and NOP/WRAP/FENCE. No GL yet - Phase 2 adds the opcode
 * table, and both halves of it are generated from one source so the guest
 * and host cannot drift apart.
 *
 * Two things here are not arbitrary and should not be "tidied":
 *
 *  - The doorbell handler does no work beyond storing the head and waking
 *    the thread. It runs on the vCPU thread under the BQL, and the guest is
 *    libc_r - one kernel thread for all of renderd's threads - so a stalled
 *    vCPU stalls the application's data ingest and product decode too, not
 *    just the thread that made the GL call.
 *
 *  - Completion is published in the ring header, in guest RAM, and not in a
 *    register. Phase 0a measured an MMIO read from this guest at 8.02 us;
 *    the same word in the AGP aperture is an ordinary load. The guest polls
 *    the header.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/thread.h"
#include "qemu/atomic.h"
#include "hw/isa/isa.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "monitor/monitor.h"
#include "monitor/hmp.h"
#include "qobject/qdict.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qom/object.h"
#include "trace.h"

#ifdef CONFIG_PNG
#include <png.h>
#endif

/*
 * Host GL comes from OSMesa: off-screen software rendering with no window
 * system behind it, and a genuine compatibility profile, which is what this
 * replay needs - glBegin, glClipPlane, the matrix stack. EGL was the original
 * backend and was dropped once OSMesa proved equivalent, because EGL does not
 * exist on macOS and carrying two copies of Mesa in one process lets the
 * dynamic linker interpose one's glBegin on the other's context.
 *
 * libOSMesa exports the whole GL entry point set itself on Unix, including
 * the FBO calls, so this links it directly rather than going through epoxy.
 * The Windows DLL follows the platform convention and exposes post-1.1 GL
 * entry points through OSMesaGetProcAddress(); those few calls are resolved
 * below after the context is current.
 */
#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/osmesa.h>

#ifdef _WIN32
static PFNGLBINDFRAMEBUFFERPROC is1gl_glBindFramebuffer;
static PFNGLBINDRENDERBUFFERPROC is1gl_glBindRenderbuffer;
static PFNGLCHECKFRAMEBUFFERSTATUSPROC is1gl_glCheckFramebufferStatus;
static PFNGLCLEARBUFFERFVPROC is1gl_glClearBufferfv;
static PFNGLFRAMEBUFFERRENDERBUFFERPROC is1gl_glFramebufferRenderbuffer;
static PFNGLGENFRAMEBUFFERSPROC is1gl_glGenFramebuffers;
static PFNGLGENRENDERBUFFERSPROC is1gl_glGenRenderbuffers;
static PFNGLRENDERBUFFERSTORAGEPROC is1gl_glRenderbufferStorage;

#define glBindFramebuffer is1gl_glBindFramebuffer
#define glBindRenderbuffer is1gl_glBindRenderbuffer
#define glCheckFramebufferStatus is1gl_glCheckFramebufferStatus
#define glClearBufferfv is1gl_glClearBufferfv
#define glFramebufferRenderbuffer is1gl_glFramebufferRenderbuffer
#define glGenFramebuffers is1gl_glGenFramebuffers
#define glGenRenderbuffers is1gl_glGenRenderbuffers
#define glRenderbufferStorage is1gl_glRenderbufferStorage

static bool is1gl_load_gl_extensions(void)
{
#define IS1GL_LOAD_GL(name, type)                                      \
    do {                                                               \
        is1gl_##name = (type)OSMesaGetProcAddress(#name);              \
        if (!is1gl_##name) {                                           \
            error_report("is1gl: OSMesa entry point %s is unavailable", \
                         #name);                                       \
            return false;                                              \
        }                                                              \
    } while (0)

    IS1GL_LOAD_GL(glBindFramebuffer, PFNGLBINDFRAMEBUFFERPROC);
    IS1GL_LOAD_GL(glBindRenderbuffer, PFNGLBINDRENDERBUFFERPROC);
    IS1GL_LOAD_GL(glCheckFramebufferStatus,
                  PFNGLCHECKFRAMEBUFFERSTATUSPROC);
    IS1GL_LOAD_GL(glClearBufferfv, PFNGLCLEARBUFFERFVPROC);
    IS1GL_LOAD_GL(glFramebufferRenderbuffer,
                  PFNGLFRAMEBUFFERRENDERBUFFERPROC);
    IS1GL_LOAD_GL(glGenFramebuffers, PFNGLGENFRAMEBUFFERSPROC);
    IS1GL_LOAD_GL(glGenRenderbuffers, PFNGLGENRENDERBUFFERSPROC);
    IS1GL_LOAD_GL(glRenderbufferStorage, PFNGLRENDERBUFFERSTORAGEPROC);

#undef IS1GL_LOAD_GL
    return true;
}
#endif

#include "is1gl_ops.h"

#define TYPE_IS1GL "is1gl"
OBJECT_DECLARE_SIMPLE_TYPE(Is1glState, IS1GL)

/*
 * 'IS1G'. The guest reads this first and does nothing further if it does not
 * match, so a kernel that maps the wrong page cannot be mistaken for a
 * working transport.
 */
#define IS1GL_MAGIC        0x49533147
#define IS1GL_VERSION      1

/* MMIO register file, 4 KiB. */
#define IS1GL_REG_MAGIC    0x00   /* ro  */
#define IS1GL_REG_VERSION  0x04   /* ro  */
#define IS1GL_REG_RING_LO  0x08   /* rw  guest-physical base of the ring   */
#define IS1GL_REG_RING_HI  0x0c   /* rw  */
#define IS1GL_REG_RING_SZ  0x10   /* rw  data bytes; writing it arms the ring */
#define IS1GL_REG_DOORBELL 0x14   /* wo  new ring head                     */
#define IS1GL_REG_DONE_SEQ 0x18   /* ro  completed fence (debug: use the header) */
#define IS1GL_REG_BELLS    0x1c   /* ro  doorbells seen                    */
#define IS1GL_REG_STATUS   0x20   /* ro  1 = ring armed                    */
#define IS1GL_REG_APER_LO  0x24   /* rw  AGP aperture base                 */
#define IS1GL_REG_APER_HI  0x28   /* rw  */
#define IS1GL_REG_APER_SZ  0x2c   /* rw  writing it maps the aperture      */
#define IS1GL_REG_CAPS     0x30   /* ro  optional operations              */
#define IS1GL_CAP_QT_PNG   0x01
#define IS1GL_REG_SIZE     0x1000

/* I/O port window. Root-only in the guest (Phase 0a) - test use only. */
#define IS1GL_IO_DOORBELL  0
#define IS1GL_IO_BELLS     4
#define IS1GL_IO_SIZE      8

/*
 * The ring ABI. Phase 2's tools/is1gl/protocol.py becomes the single source
 * for this and generates both sides; until then the guest's copy in
 * tools/is1gl/is1gl_ring.h must be kept in step by hand.
 *
 * Layout: a 4 KiB header, then `size` bytes of records. A record is
 *
 *      u32 opcode; u32 length; payload...
 *
 * with `length` the whole record including the 8-byte head, rounded up to 8.
 * A record never straddles the end of the data area: the producer writes
 * IS1GL_OP_WRAP instead and restarts at 0.
 */
#define IS1GL_RING_MAGIC   0x49533152   /* 'IS1R' */
#define IS1GL_RING_HDR     4096

#define RH_MAGIC      0x00
#define RH_VERSION    0x04
#define RH_SIZE       0x08
#define RH_HEAD       0x10   /* guest writes */
#define RH_TAIL       0x14   /* host writes  */
#define RH_DONE_SEQ   0x18   /* host writes  */
#define RH_GUEST_SEQ  0x1c   /* guest writes */
#define RH_ERRORS     0x20   /* host writes  */

/*
 * A host context standing in for one guest GLX context.
 *
 * renderd has two: a loader thread creates textures, glyph pages and display
 * lists in one, and the renderer draws in the other. The guest library puts
 * a MAKE_CURRENT on the ring whenever the next call comes from the other
 * context, and each guest context is replayed in a host context of its own,
 * so everything GL keeps per context - bindings, enables, matrices, the
 * current colour, the attribute stacks, display-list compile mode - stays
 * apart exactly as it does under GLX. They are all in one share group, as
 * the guest asked, so textures and display lists are common to all of them.
 *
 * This device used to replay every guest context in one host context, and
 * keep only the viewport apart. That let the loader's glBindTexture change
 * what the renderer drew with, and a glyph page overwritten with an image
 * turned text into solid blocks.
 *
 * Framebuffer objects are the one kind of object contexts do not share, so
 * each has its own, all attached to one shared colour renderbuffer: in the
 * guest, both contexts are bound to the same X window.
 */
typedef struct Is1glHostCtx {
    uint32_t      id;
    OSMesaContext ctx;
    GLuint        fbo;
    void         *buf;          /* default framebuffer it was made current on */
    uint32_t      buf_w, buf_h;
    GLint         viewport[4];  /* a copy, for `info is1gl` */
} Is1glHostCtx;

struct Is1glState {
    ISADevice parent_obj;

    MemoryRegion mmio;
    MemoryRegion io;

    uint32_t mmio_base;
    uint32_t io_base;

    /* Ring, as the guest described it. */
    uint64_t ring_base;
    uint32_t ring_size;

    /*
     * The mapped ring. Taken once when the guest arms the ring, under the
     * BQL, and held until reset or unrealize. The AGP aperture is a RAM
     * MemoryRegion, so this is a direct pointer into the guest's memory
     * rather than a bounce buffer - which is the entire point, and is
     * checked for below.
     */
    uint8_t *ring;
    hwaddr   ring_mapped_len;

    QemuThread thread;
    QemuMutex  lock;
    QemuCond   cond;
    bool       thread_running;
    bool       stopping;
    bool       replaying;

    uint32_t head;        /* guest's producer offset, from the doorbell */
    uint32_t tail;        /* our consumer offset */
    uint32_t done_seq;

    /*
     * The GL contexts and everything reached through them live on the render
     * thread and are touched from nowhere else.
     */
    GHashTable   *host_ctxs;      /* guest context id -> Is1glHostCtx */
    Is1glHostCtx *cur;            /* current on the render thread, or NULL */
    OSMesaContext share_root;     /* the first context; the rest share it */
    void         *osmesa_buf;     /* backs the default framebuffer, unused */
    size_t        osmesa_buf_sz;
    uint32_t      osmesa_w, osmesa_h;
    bool       gl_failed;
    bool       gl_reset;          /* drop every context before replaying on */
    GLuint     colour_rb;
    uint32_t   draw_w, draw_h;

    uint32_t    guest_ctx;
    uint64_t    context_switches;

    /*
     * The AGP aperture, mapped once. Every readback destination is inside
     * it, so this is O(1) and cannot run out - unlike the fixed cache of
     * individual destinations this replaces, which held eight while renderd
     * cycles through twenty-eight frame buffers 2 MB apart and so failed
     * every readback after the eighth.
     */
    uint64_t aper_base;
    uint64_t aper_size;
    uint8_t *aper_ptr;
    hwaddr   aper_mapped;

    /* Counters, for `info is1gl`. */
    uint64_t bells_mmio, bells_io;
    uint64_t records, fences, wraps, unknown_ops, bad_records;
    uint64_t frames, readbacks, gl_errors;
    uint64_t replay_ns, readback_ns;
    uint64_t qt_png_decodes, qt_png_failures, qt_png_decode_ns;

    /*
     * Per-frame cost, and specifically its tail.
     *
     * replay_ns/frames was the only host-cost figure this device reported, and
     * a mean cannot answer the question that matters: renderd complains about
     * frame drift, and a single 200 ms frame is invisible in a mean taken over
     * ten thousand. So close out a frame at each SWAP and keep the maximum, a
     * histogram, and a count of frames that blew the NTSC budget.
     *
     * Note replay_ns already contains readback_ns - is1gl_readpixels() runs
     * inside is1gl_consume()'s loop - so the frame's host cost is the replay
     * accumulation alone, and readback is a component of it, not an addition.
     */
    uint64_t frame_acc_ns;      /* replay time since the last SWAP */
    uint64_t frame_rb_acc_ns;   /* readback within that */
    uint64_t frame_max_ns;
    uint64_t frame_over;        /* frames costing more than one NTSC frame */
    uint64_t frame_hist[8];
    /*
     * Per-opcode call counts. The last-64-record ring says what happened
     * immediately before a stall; this says which calls the application
     * actually uses at all, which is the question when a draw comes out
     * wrong rather than missing. See notes/is1gl-text-boxes.md.
     */
    uint64_t op_count[IS1GL_OP_COUNT];
    /* First few texture uploads, so "info is1gl" can say what formats the
     * application actually uses. See notes/is1gl-text-boxes.md. */
    struct {
        uint32_t sub, w, h, ifmt, fmt, type;
    } tex_log[24];
    int tex_log_n;
    char    *frame_log;
    FILE    *frame_log_f;
};

/* Upper edge of each histogram bucket, in microseconds. */
static const uint64_t is1gl_hist_edge_us[8] = {
    5000, 10000, 16683, 25000, 33367, 50000, 100000, UINT64_MAX
};
#define IS1GL_NTSC_FRAME_US 33367

/* ------------------------------------------------------------ ring access */

static uint32_t rh_get(Is1glState *s, unsigned off)
{
    return ldl_le_p(s->ring + off);
}

static void rh_put(Is1glState *s, unsigned off, uint32_t val)
{
    stl_le_p(s->ring + off, val);
}

/* ------------------------------------------------------------ wire reads
 *
 * memcpy, not casts, and the guest writes them the same way: neither side
 * then depends on the other's alignment inside a record.
 */
static uint32_t get_u32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static int32_t  get_i32(const uint8_t *p) { int32_t v;  memcpy(&v, p, 4); return v; }
static float    get_f32(const uint8_t *p) { float v;    memcpy(&v, p, 4); return v; }
static double   get_f64(const uint8_t *p) { double v;   memcpy(&v, p, 8); return v; }

/* ------------------------------------------------------------- GL set-up */

#ifndef GL_YCBCR_MESA
#define GL_YCBCR_MESA                  0x8757
#endif
#ifndef GL_UNSIGNED_SHORT_8_8_MESA
#define GL_UNSIGNED_SHORT_8_8_MESA     0x85BA
#define GL_UNSIGNED_SHORT_8_8_REV_MESA 0x85BB
#endif
#ifndef GL_PACK_INVERT_MESA
#define GL_PACK_INVERT_MESA            0x8758
#endif

/*
 * The whole design depends on the host speaking fixed-function GL natively,
 * so that nothing has to be translated into shaders. That is why the
 * contexts below ask for a compatibility profile explicitly rather than
 * going through QEMU's own helpers, which ask for core.
 *
 * OSMesa renders into a caller-supplied buffer and has no notion of a window
 * system, which is exactly what this device wants: the finished frame leaves
 * through glReadPixels either way, and there is nothing to present.
 *
 * The buffer backs the *default* framebuffer, which nothing ever draws to -
 * all rendering goes to the FBOs in is1gl_set_drawable(). It exists only
 * because OSMesaMakeCurrent() insists on one, and every context uses the same
 * one. It is kept at the drawable size so that a backend that did fall back
 * to the default framebuffer would still be correct rather than subtly
 * clipped.
 */
static OSMesaContext is1gl_osmesa_create(OSMesaContext share)
{
    /*
     * Ask for a compatibility profile explicitly. The whole design depends on
     * the host speaking fixed-function GL natively - glBegin, glClipPlane,
     * the matrix stack - so a core profile is useless here. OSMESA_PROFILE
     * needs Mesa 12 or later; the Ext form below is the old fallback and is
     * compatibility by definition.
     */
    static const int ctx_attr[] = {
        OSMESA_FORMAT,                OSMESA_RGBA,
        OSMESA_DEPTH_BITS,            24,
        OSMESA_STENCIL_BITS,          8,
        OSMESA_ACCUM_BITS,            0,
        OSMESA_PROFILE,               OSMESA_COMPAT_PROFILE,
        OSMESA_CONTEXT_MAJOR_VERSION, 3,
        OSMESA_CONTEXT_MINOR_VERSION, 0,
        0
    };
    OSMesaContext c;

    c = OSMesaCreateContextAttribs(ctx_attr, share);
    if (!c) {
        c = OSMesaCreateContextExt(OSMESA_RGBA, 24, 8, 0, share);
    }
    return c;
}

/* GL errors belong to a context, so collect them before leaving one. */
static void is1gl_drain_errors(Is1glState *s)
{
    GLenum e;

    while ((e = glGetError()) != GL_NO_ERROR) {
        s->gl_errors++;
        if (s->gl_errors <= 16) {
            trace_is1gl_gl_error(e, s->frames);
        }
    }
}

/*
 * The context for a guest context id, created on first use. The first one
 * created is the share root, and every later one shares with it.
 */
static Is1glHostCtx *is1gl_host_ctx(Is1glState *s, uint32_t id)
{
    Is1glHostCtx *c;
    OSMesaContext osm;

    c = g_hash_table_lookup(s->host_ctxs, GUINT_TO_POINTER(id));
    if (c || s->gl_failed) {
        return c;
    }
    osm = is1gl_osmesa_create(s->share_root);
    if (!osm) {
        /* Never retried: a failure here would repeat on every record. */
        error_report("is1gl: cannot create an OSMesa context");
        s->gl_failed = true;
        return NULL;
    }
    if (!s->share_root) {
        s->share_root = osm;
    }
    c = g_new0(Is1glHostCtx, 1);
    c->id = id;
    c->ctx = osm;
    g_hash_table_insert(s->host_ctxs, GUINT_TO_POINTER(id), c);
    return c;
}

/*
 * Drop every context, and with the last of them every texture and list.
 * Done when the guest arms a new ring - a restarted renderd, whose
 * predecessor's objects and state would otherwise still be here, under the
 * same names its successor is about to hand out again. Render thread only.
 */
static void is1gl_destroy_contexts(Is1glState *s)
{
    GHashTableIter it;
    gpointer v;

    if (s->cur) {
        is1gl_drain_errors(s);
        /* Newer OSMesa releases on NULL; older ones refuse, and destroying
         * the current context releases it anyway. */
        OSMesaMakeCurrent(NULL, NULL, 0, 0, 0);
        s->cur = NULL;
    }
    /* The share root last, although Mesa refcounts the shared state. */
    g_hash_table_iter_init(&it, s->host_ctxs);
    while (g_hash_table_iter_next(&it, NULL, &v)) {
        Is1glHostCtx *c = v;

        if (c->ctx != s->share_root) {
            OSMesaDestroyContext(c->ctx);
        }
    }
    if (s->share_root) {
        OSMesaDestroyContext(s->share_root);
        s->share_root = NULL;
    }
    g_hash_table_remove_all(s->host_ctxs);
    s->colour_rb = 0;
    s->draw_w = s->draw_h = 0;
}

/*
 * The drawable is an FBO, because nothing is ever presented: the finished
 * frame leaves through glReadPixels into the card's own memory. Its colour
 * renderbuffer is shared by every context, and each context has an FBO of
 * its own on it. Called with c current.
 */
static void is1gl_set_drawable(Is1glState *s, Is1glHostCtx *c,
                               uint32_t w, uint32_t h)
{
    bool resized = false;

    if (!s->colour_rb) {
        glGenRenderbuffers(1, &s->colour_rb);
    }
    if (s->draw_w != w || s->draw_h != h) {
        glBindRenderbuffer(GL_RENDERBUFFER, s->colour_rb);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, w, h);
        s->draw_w = w;
        s->draw_h = h;
        resized = true;
    }
    if (!c->fbo) {
        glGenFramebuffers(1, &c->fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, c->fbo);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                  GL_RENDERBUFFER, s->colour_rb);
    } else if (!resized) {
        return;
    }
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        error_report("is1gl: framebuffer incomplete at %ux%u", w, h);
        return;
    }
    if (!resized) {
        return;
    }

    /*
     * Renderbuffer storage starts with undefined contents.  renderd normally
     * paints only the regions needed by a product, and the card preview drops
     * the alpha/key channel, so untouched transparent pixels otherwise expose
     * allocator contents as high-entropy RGB noise.  This is especially easy
     * to see with llvmpipe on macOS, where reused tile memory is not
     * incidentally zeroed.
     *
     * glClearBufferfv ignores the current colour write mask and clear colour.
     * The scissor still applies, so disable just that state temporarily.  No
     * guest-visible GL state is changed by initializing the new attachment.
     */
    {
        static const GLfloat transparent_black[4] = { 0, 0, 0, 0 };
        GLboolean scissor = glIsEnabled(GL_SCISSOR_TEST);

        if (scissor) {
            glDisable(GL_SCISSOR_TEST);
        }
        glClearBufferfv(GL_COLOR, 0, transparent_black);
        if (scissor) {
            glEnable(GL_SCISSOR_TEST);
        }
    }
    trace_is1gl_drawable(w, h);
}

/*
 * MAKE_CURRENT: replay what follows in guest context `id`'s host context,
 * bound to a drawable of w x h.
 *
 * A current guest library sends one whenever the next call comes from the
 * other context. An older one sent one per context, when each thread first
 * made its context current, so everything after the second landed in one
 * host context - which is how this device used to behave anyway.
 */
static void is1gl_make_current(Is1glState *s, uint32_t id,
                               uint32_t w, uint32_t h)
{
    bool first = !s->share_root;
    Is1glHostCtx *c;

    if (!w || !h || w > 4096 || h > 4096) {
        return;
    }
    c = is1gl_host_ctx(s, id);
    if (!c) {
        return;
    }
    if (s->cur && s->cur != c) {
        is1gl_drain_errors(s);
    }

    if (w != s->osmesa_w || h != s->osmesa_h) {
        size_t need = (size_t)w * h * 4;

        /*
         * OSMesa keeps one buffer record per pixel format for every context,
         * pointing at whatever it was last given, so the old allocation
         * must outlive the next OSMesaMakeCurrent. Grow only.
         */
        if (need > s->osmesa_buf_sz) {
            void *old = s->osmesa_buf;

            s->osmesa_buf = g_malloc0(need);
            s->osmesa_buf_sz = need;
            if (!OSMesaMakeCurrent(c->ctx, s->osmesa_buf, GL_UNSIGNED_BYTE,
                                   w, h)) {
                error_report("is1gl: OSMesaMakeCurrent failed");
                s->cur = NULL;
                g_free(old);
                return;
            }
            g_free(old);
            c->buf = s->osmesa_buf;
            c->buf_w = w;
            c->buf_h = h;
            s->cur = c;
        }
        s->osmesa_w = w;
        s->osmesa_h = h;
    }
    if (s->cur != c || c->buf != s->osmesa_buf ||
        c->buf_w != w || c->buf_h != h) {
        if (!OSMesaMakeCurrent(c->ctx, s->osmesa_buf, GL_UNSIGNED_BYTE,
                               w, h)) {
            error_report("is1gl: OSMesaMakeCurrent failed");
            s->cur = NULL;
            return;
        }
        c->buf = s->osmesa_buf;
        c->buf_w = w;
        c->buf_h = h;
        s->cur = c;
    }

    if (first) {
#ifdef _WIN32
        if (!is1gl_load_gl_extensions()) {
            s->gl_failed = true;
            s->cur = NULL;
            return;
        }
#endif
        /*
         * Phase 0d confirmed the host offers 4.3 Compatibility and
         * GL_MESA_pack_invert; OSMesa supplies both.
         */
        trace_is1gl_gl_up((const char *)glGetString(GL_RENDERER),
                          (const char *)glGetString(GL_VERSION));
    }
    if (!c->fbo) {
        /* GL starts a context's viewport at its first drawable. */
        c->viewport[0] = 0;
        c->viewport[1] = 0;
        c->viewport[2] = w;
        c->viewport[3] = h;
    }
    is1gl_set_drawable(s, c, w, h);
    s->guest_ctx = id;
    s->context_switches++;
}

static void is1gl_contexts_reset(Is1glState *s)
{
    /* The contexts themselves belong to the render thread; it drops them. */
    s->gl_reset = true;
    s->guest_ctx = 0;
    s->context_switches = 0;
}

/* ------------------------------------------------------------ host hooks */

static void is1gl_host_glViewport(Is1glState *s, int32_t x, int32_t y,
                                  int32_t width, int32_t height)
{
    if (s->cur) {
        s->cur->viewport[0] = x;
        s->cur->viewport[1] = y;
        s->cur->viewport[2] = width;
        s->cur->viewport[3] = height;
    }
    glViewport(x, y, width, height);
}

static long is1gl_host_image_bytes(int32_t w, int32_t h, uint32_t format,
                                   uint32_t type)
{
    int bpp;

    if (w <= 0 || h <= 0) {
        return 0;
    }
    /*
     * The guest uses GL_NONE as an on-wire sentinel for a NULL pixels
     * argument to glTexImage2D().  There is deliberately no image payload in
     * that case.  This must be recognized here, before the generated replay
     * code validates the record length; otherwise it rejects the record and
     * the allocation path in is1gl_host_glTexImage2D() is unreachable.
     */
    if (type == GL_NONE) {
        return 0;
    }
    if (format == GL_YCBCR_MESA) {
        bpp = 2;
    } else {
        int comps;
        switch (type) {
        case GL_UNSIGNED_INT_8_8_8_8:
        case GL_UNSIGNED_INT_8_8_8_8_REV:
            return (long)w * h * 4;
        case GL_UNSIGNED_SHORT_5_6_5:
        case GL_UNSIGNED_SHORT_4_4_4_4:
        case GL_UNSIGNED_SHORT_5_5_5_1:
            return (long)w * h * 2;
        }
        switch (format) {
        case GL_RED: case GL_GREEN: case GL_BLUE: case GL_ALPHA:
        case GL_LUMINANCE:                          comps = 1; break;
        case GL_LUMINANCE_ALPHA:                    comps = 2; break;
        case GL_RGB: case GL_BGR:                   comps = 3; break;
        case GL_RGBA: case GL_BGRA:                 comps = 4; break;
        case GL_NONE:                               return 0;
        default:                                    return -1;
        }
        switch (type) {
        case GL_BYTE: case GL_UNSIGNED_BYTE:        bpp = comps; break;
        case GL_SHORT: case GL_UNSIGNED_SHORT:      bpp = comps * 2; break;
        case GL_INT: case GL_UNSIGNED_INT:
        case GL_FLOAT:                              bpp = comps * 4; break;
        default:                                    return -1;
        }
    }
    return (long)w * h * bpp;
}

static inline int clamp255(int v)
{
    return v < 0 ? 0 : (v > 255 ? 255 : v);
}

/*
 * The video layer arrives as 4:2:2 YCbCr - GL_MESA_ycbcr_texture, which this
 * host's driver does not offer and which killed the guest's own X server
 * besides (notes/gl-ycbcr-and-the-x-crash.md). Convert here: it is the same
 * BT.601 arithmetic libglfix was doing in the guest, moved to the side of
 * the wire that has cycles to spare.
 *
 * GL_UNSIGNED_SHORT_8_8_APPLE is 'yuvs', Y0 Cb Y1 Cr in memory; the _REV
 * form is '2vuy'. Rows arrive tightly packed, because the guest applies its
 * own unpack state when it packs the record.
 */
static uint8_t *is1gl_ycbcr_to_rgb(const uint8_t *src, int32_t w, int32_t h,
                                   uint32_t type)
{
    bool rev = (type == GL_UNSIGNED_SHORT_8_8_REV_MESA);
    uint8_t *rgb = g_malloc((size_t)w * h * 3);
    int32_t x, y;

    for (y = 0; y < h; y++) {
        const uint8_t *sr = src + (size_t)y * w * 2;
        uint8_t *dr = rgb + (size_t)y * w * 3;

        for (x = 0; x < w; x += 2) {
            int y0, y1, cb, cr, c, d, e;

            if (rev) {
                cb = sr[0]; y0 = sr[1]; cr = sr[2]; y1 = sr[3];
            } else {
                y0 = sr[0]; cb = sr[1]; y1 = sr[2]; cr = sr[3];
            }
            d = cb - 128;
            e = cr - 128;
            c = y0 - 16;
            dr[0] = clamp255((298 * c + 409 * e + 128) >> 8);
            dr[1] = clamp255((298 * c - 100 * d - 208 * e + 128) >> 8);
            dr[2] = clamp255((298 * c + 516 * d + 128) >> 8);
            c = y1 - 16;
            dr[3] = clamp255((298 * c + 409 * e + 128) >> 8);
            dr[4] = clamp255((298 * c - 100 * d - 208 * e + 128) >> 8);
            dr[5] = clamp255((298 * c + 516 * d + 128) >> 8);
            sr += 4;
            dr += 6;
        }
    }
    return rgb;
}

static void is1gl_unpack_tight(void)
{
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
    glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
}

static void is1gl_host_glTexImage2D(uint32_t target, int32_t level,
                                    int32_t internalFormat, int32_t width,
                                    int32_t height, int32_t border,
                                    uint32_t format, uint32_t type,
                                    const void *pixels)
{
    is1gl_unpack_tight();
    if (format == GL_YCBCR_MESA) {
        uint8_t *rgb = is1gl_ycbcr_to_rgb(pixels, width, height, type);
        glTexImage2D(target, level, GL_RGB, width, height, border,
                     GL_RGB, GL_UNSIGNED_BYTE, rgb);
        g_free(rgb);
        return;
    }
    /* type GL_NONE is the guest saying "allocate, do not initialise". */
    glTexImage2D(target, level, internalFormat, width, height, border,
                 format, type == GL_NONE ? GL_UNSIGNED_BYTE : type,
                 type == GL_NONE ? NULL : pixels);
}

/*
 * Dump the currently bound 2D texture as a PGM/PPM, for looking at a glyph
 * atlas that is coming out wrong on screen. IS1GL_TEXDUMP=<path-prefix> only;
 * costs nothing otherwise. See notes/is1gl-text-boxes.md.
 */
static void is1gl_dump_bound_texture(const char *tag)
{
    const char *prefix = getenv("IS1GL_TEXDUMP");
    GLint w = 0, h = 0, ifmt = 0;
    uint8_t *buf;
    char path[512];
    FILE *f;

    if (!prefix) {
        return;
    }
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &w);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &h);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_INTERNAL_FORMAT,
                             &ifmt);
    if (w <= 0 || h <= 0 || (int64_t)w * h > (int64_t)64 * 1024 * 1024) {
        return;
    }
    buf = g_malloc0((size_t)w * h);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_ALPHA, GL_UNSIGNED_BYTE, buf);
    snprintf(path, sizeof path, "%s-%s-%dx%d-ifmt%04x.pgm",
             prefix, tag, w, h, (unsigned)ifmt);
    f = fopen(path, "wb");
    if (f) {
        fprintf(f, "P5\n%d %d\n255\n", w, h);
        fwrite(buf, 1, (size_t)w * h, f);
        fclose(f);
    }
    g_free(buf);
}

static void is1gl_host_glTexSubImage2D(uint32_t target, int32_t level,
                                       int32_t xoffset, int32_t yoffset,
                                       int32_t width, int32_t height,
                                       uint32_t format, uint32_t type,
                                       const void *pixels)
{
    is1gl_unpack_tight();
    if (format == GL_YCBCR_MESA) {
        uint8_t *rgb = is1gl_ycbcr_to_rgb(pixels, width, height, type);
        glTexSubImage2D(target, level, xoffset, yoffset, width, height,
                        GL_RGB, GL_UNSIGNED_BYTE, rgb);
        g_free(rgb);
        return;
    }
    glTexSubImage2D(target, level, xoffset, yoffset, width, height,
                    format, type, pixels);
    if (format == GL_ALPHA) {
        static int n;
        if (++n % 150 == 0) {
            is1gl_dump_bound_texture("alpha");
        }
    }
}

static void is1gl_host_glDrawPixels(int32_t width, int32_t height,
                                    uint32_t format, uint32_t type,
                                    const void *pixels)
{
    is1gl_unpack_tight();
    glDrawPixels(width, height, format, type, pixels);
}

#include "is1gl_replay.h"

/*
 * The readback, and the reason the whole design pays: the host writes the
 * finished picture straight into the AGP frame the Thunderstorm DMAs from,
 * so the 1.38 MB never crosses a socket and nobody has to reverse 480 rows
 * by hand - GL_MESA_pack_invert is native here.
 */
static uint8_t *is1gl_map_dst(Is1glState *s, uint64_t phys, uint64_t len)
{
    uint64_t off;

    if (!s->aper_ptr) {
        return NULL;
    }
    if (phys < s->aper_base) {
        return NULL;
    }
    off = phys - s->aper_base;
    if (off + len > s->aper_size) {
        return NULL;
    }
    return s->aper_ptr + off;
}

/* Decode one PNG-compressed QuickTime frame into a guest AGP scratch buffer.
 * The first four destination bytes are a completion status (1 = success).
 * The guest waits for the following FENCE before reading either status or
 * RGBA pixels. Keeping this outside the vCPU thread lets it continue running
 * while the host does the PNG work. */
static void is1gl_qt_png_decode(Is1glState *s, const uint8_t *a, uint32_t alen)
{
    uint32_t w, h, phys, png_len;
    uint64_t output_len;
    uint8_t *dst;
    int64_t start = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    bool ok = false;

    if (alen < 16) {
        s->bad_records++;
        return;
    }
    w = ldl_le_p(a);
    h = ldl_le_p(a + 4);
    phys = ldl_le_p(a + 8);
    png_len = ldl_le_p(a + 12);
    if (!w || !h || w > 2048 || h > 2048 || png_len > 2 * 1024 * 1024 ||
        png_len > alen - 16) {
        s->qt_png_failures++;
        return;
    }
    output_len = 4 + (uint64_t)w * h * 4;
    dst = is1gl_map_dst(s, phys, output_len);
    if (!dst) {
        s->qt_png_failures++;
        return;
    }
    stl_le_p(dst, 0);
#ifdef CONFIG_PNG
    {
        png_image image = { 0 };

        image.version = PNG_IMAGE_VERSION;
        if (png_image_begin_read_from_memory(&image, a + 16, png_len)) {
            if (image.width == w && image.height == h) {
                image.format = PNG_FORMAT_RGBA;
                ok = png_image_finish_read(&image, NULL, dst + 4,
                                           w * 4, NULL);
            }
            png_image_free(&image);
        }
    }
#endif
    if (ok) {
        stl_le_p(dst, 1);
        s->qt_png_decodes++;
        s->qt_png_decode_ns += qemu_clock_get_ns(QEMU_CLOCK_REALTIME) - start;
    } else {
        s->qt_png_failures++;
    }
}

static void is1gl_readpixels(Is1glState *s, const uint8_t *a, uint32_t alen)
{
    int32_t x, y, w, h, row_length, invert;
    uint32_t format, type, phys;
    int64_t t0 = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    uint64_t len;
    uint8_t *dst;
    long bpp;

    if (alen < 36 || !s->cur) {
        return;
    }
    x = get_i32(a + 0);
    y = get_i32(a + 4);
    w = get_i32(a + 8);
    h = get_i32(a + 12);
    format = get_u32(a + 16);
    type = get_u32(a + 20);
    phys = get_u32(a + 24);
    row_length = get_i32(a + 28);
    invert = get_i32(a + 32);

    bpp = is1gl_host_image_bytes(1, 1, format, type);
    if (bpp <= 0 || w <= 0 || h <= 0) {
        s->bad_records++;
        return;
    }
    if (row_length < w) {
        row_length = w;
    }
    len = (uint64_t)row_length * bpp * h;

    dst = is1gl_map_dst(s, phys, len);
    if (!dst) {
        s->bad_records++;
        trace_is1gl_bad_record("readback destination not mappable", 0, phys, len);
        return;
    }

    glPixelStorei(GL_PACK_ROW_LENGTH, row_length);
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glPixelStorei(GL_PACK_INVERT_MESA, invert ? GL_TRUE : GL_FALSE);
    glReadPixels(x, y, w, h, format, type, dst);
    glPixelStorei(GL_PACK_INVERT_MESA, GL_FALSE);

    s->readbacks++;
    {
        uint64_t d = qemu_clock_get_ns(QEMU_CLOCK_REALTIME) - t0;

        s->readback_ns += d;
        s->frame_rb_acc_ns += d;
    }
}

/*
 * Consume records from tail to head. Called only on the render thread, with
 * s->lock NOT held: the guest may keep producing while we work, and head is
 * re-read under the lock by the caller.
 */
static void is1gl_consume(Is1glState *s, uint32_t head)
{
    uint8_t *data = s->ring + IS1GL_RING_HDR;
    uint32_t tail = s->tail;
    int64_t t0 = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    int64_t last_mark = t0;

    while (tail != head) {
        uint32_t opcode, length;

        /* A record head must fit before the end of the data area. */
        if (tail + 8 > s->ring_size) {
            s->bad_records++;
            trace_is1gl_bad_record("head past end", tail, 0, 0);
            tail = 0;
            continue;
        }

        opcode = ldl_le_p(data + tail);
        length = ldl_le_p(data + tail + 4);

        if (opcode == IS1GL_OP_WRAP) {
            s->wraps++;
            tail = 0;
            continue;
        }

        if (length < 8 || (length & 7) || tail + length > s->ring_size) {
            /*
             * A malformed length is the guest library's bug, and continuing
             * would read arbitrary guest memory as commands. Stop consuming
             * and let `info is1gl` show it rather than guessing.
             */
            s->bad_records++;
            trace_is1gl_bad_record("bad length", tail, opcode, length);
            break;
        }

        switch (opcode) {
        case IS1GL_OP_NOP:
            break;
        case IS1GL_OP_FENCE:
            if (length >= 12) {
                s->done_seq = ldl_le_p(data + tail + 8);
                s->fences++;
            } else {
                s->bad_records++;
            }
            break;
        case IS1GL_OP_MAKE_CURRENT:
            if (length >= 20) {
                is1gl_make_current(s, ldl_le_p(data + tail + 8),
                                   ldl_le_p(data + tail + 12),
                                   ldl_le_p(data + tail + 16));
            }
            break;
        case IS1GL_OP_SWAP: {
            int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
            uint64_t frame_us, rb_us;
            int b;

            s->frame_acc_ns += now - last_mark;
            last_mark = now;
            s->frames++;

            frame_us = s->frame_acc_ns / 1000;
            rb_us = s->frame_rb_acc_ns / 1000;
            if (s->frame_acc_ns > s->frame_max_ns) {
                s->frame_max_ns = s->frame_acc_ns;
            }
            if (frame_us > IS1GL_NTSC_FRAME_US) {
                s->frame_over++;
            }
            for (b = 0; b < 8; b++) {
                if (frame_us < is1gl_hist_edge_us[b]) {
                    s->frame_hist[b]++;
                    break;
                }
            }
            if (s->frame_log_f) {
                /* Wall clock, so this can be lined up against the card's
                 * frame-log and renderd's drift warnings in the clientlog. */
                fprintf(s->frame_log_f,
                        "%" PRIu64 ",%" PRId64 ",%" PRIu64 ",%" PRIu64 "\n",
                        s->frames, now / 1000, frame_us, rb_us);
            }
            s->frame_acc_ns = 0;
            s->frame_rb_acc_ns = 0;
            break;
        }
        case IS1GL_OP_READPIXELS:
            is1gl_readpixels(s, data + tail + 8, length - 8);
            break;
        case IS1GL_OP_QT_PNG_DECODE:
            is1gl_qt_png_decode(s, data + tail + 8, length - 8);
            break;
        default:
            if (!s->cur ||
                !is1gl_replay_one(s, opcode, data + tail + 8, length - 8)) {
                s->unknown_ops++;
                if (s->unknown_ops <= 16) {
                    trace_is1gl_bad_record("unknown opcode", tail, opcode,
                                           length);
                }
            }
            break;
        }

        if (opcode < IS1GL_OP_COUNT) {
            s->op_count[opcode]++;
        }
        if ((opcode == IS1GL_OP_glTexImage2D ||
             opcode == IS1GL_OP_glTexSubImage2D) &&
            s->tex_log_n < (int)ARRAY_SIZE(s->tex_log) && length >= 40) {
            const uint8_t *r = data + tail + 8;
            int sub = (opcode == IS1GL_OP_glTexSubImage2D);
            int i = s->tex_log_n++;

            /* glTexImage2D:    target,level,ifmt,w,h,border,fmt,type
             * glTexSubImage2D: target,level,x,y,w,h,fmt,type          */
            /* The capture plane re-uploads a 720x480 GL_YCBCR_MESA frame
             * every field and would fill this log before anything else got
             * a slot. It is not what we are looking at. */
            if (ldl_le_p(r + 24) == 0x8757) {
                s->tex_log_n--;
                goto tex_logged;
            }
            s->tex_log[i].sub  = sub;
            s->tex_log[i].w    = ldl_le_p(r + (sub ? 16 : 12));
            s->tex_log[i].h    = ldl_le_p(r + (sub ? 20 : 16));
            s->tex_log[i].ifmt = sub ? 0 : ldl_le_p(r + 8);
            s->tex_log[i].fmt  = ldl_le_p(r + 24);
            s->tex_log[i].type = ldl_le_p(r + 28);
tex_logged: ;
        }
        s->records++;
        tail += length;
    }

    s->tail = tail;
    {
        int64_t end = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);

        s->replay_ns += end - t0;
        /* Whatever followed the last SWAP belongs to the frame being built. */
        s->frame_acc_ns += end - last_mark;
    }

    /*
     * Ask once per batch rather than per call. Per call it would be tens of
     * thousands of synchronous queries a frame; per batch it still names the
     * frame an error appeared in.
     */
    if (s->cur) {
        is1gl_drain_errors(s);
    }

    /*
     * Publish progress. The store to done_seq must not be visible before the
     * work it reports, because the guest spins on exactly this word.
     */
    rh_put(s, RH_TAIL, s->tail);
    rh_put(s, RH_ERRORS, (uint32_t)(s->bad_records + s->unknown_ops));
    smp_wmb();
    rh_put(s, RH_DONE_SEQ, s->done_seq);
}

static void *is1gl_render_thread(void *opaque)
{
    Is1glState *s = opaque;

    qemu_mutex_lock(&s->lock);
    while (!s->stopping) {
        uint32_t head;
        bool reset;

        if (!s->ring || s->head == s->tail) {
            qemu_cond_wait(&s->cond, &s->lock);
            continue;
        }
        head = s->head;
        reset = s->gl_reset;
        s->gl_reset = false;
        s->replaying = true;
        qemu_mutex_unlock(&s->lock);

        if (reset) {
            is1gl_destroy_contexts(s);
        }

        /*
         * Records were written before the doorbell, and the doorbell is a VM
         * exit, but be explicit: nothing below may be hoisted above the read
         * of head.
         */
        smp_rmb();
        is1gl_consume(s, head);

        qemu_mutex_lock(&s->lock);
        s->replaying = false;
        qemu_cond_broadcast(&s->cond);
    }
    qemu_mutex_unlock(&s->lock);
    return NULL;
}

/* --------------------------------------------------------------- arming */

/*
 * A replay batch uses raw pointers returned by address_space_map().  Mapping
 * changes are rare (guest-library initialization/restart and device reset),
 * so wait for the active batch instead of putting the mutex around every GL
 * call and making ordinary doorbells contend with the renderer.
 *
 * Called with s->lock held.
 */
static void is1gl_wait_replay_idle(Is1glState *s)
{
    while (s->replaying) {
        qemu_cond_wait(&s->cond, &s->lock);
    }
}

static void is1gl_unmap_aperture(Is1glState *s)
{
    if (s->aper_ptr) {
        address_space_unmap(&address_space_memory, s->aper_ptr,
                            s->aper_mapped, true, s->aper_mapped);
        s->aper_ptr = NULL;
        s->aper_mapped = 0;
    }
}

/* Called with the BQL and s->lock held, or after the render thread stopped. */
static void is1gl_map_aperture(Is1glState *s)
{
    hwaddr len;
    void *p;

    is1gl_unmap_aperture(s);
    if (!s->aper_size) {
        return;
    }
    len = s->aper_size;
    p = address_space_map(&address_space_memory, s->aper_base, &len, true,
                          MEMTXATTRS_UNSPECIFIED);
    if (!p || len != s->aper_size) {
        if (p) {
            address_space_unmap(&address_space_memory, p, len, true, 0);
        }
        qemu_log_mask(LOG_GUEST_ERROR,
                      "is1gl: cannot map the AGP aperture at 0x%" PRIx64
                      " size 0x%" PRIx64 "\n", s->aper_base, s->aper_size);
        return;
    }
    s->aper_ptr = p;
    s->aper_mapped = len;
    trace_is1gl_aperture(s->aper_base, s->aper_size);
}

static void is1gl_unmap_ring(Is1glState *s)
{
    if (s->ring) {
        address_space_unmap(&address_space_memory, s->ring,
                            s->ring_mapped_len, true, s->ring_mapped_len);
        s->ring = NULL;
        s->ring_mapped_len = 0;
    }
}

/* Called with the BQL and s->lock held, or after the render thread stopped. */
static void is1gl_arm_ring(Is1glState *s)
{
    hwaddr len;
    void *p;

    is1gl_unmap_ring(s);
    /* A new transport may reuse context ids from a restarted renderd. */
    is1gl_contexts_reset(s);
    s->tail = 0;
    s->head = 0;

    if (!s->ring_size) {
        return;
    }
    if (s->ring_size & 7) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "is1gl: ring size %u is not a multiple of 8\n",
                      s->ring_size);
        return;
    }

    len = IS1GL_RING_HDR + (hwaddr)s->ring_size;
    p = address_space_map(&address_space_memory, s->ring_base, &len, true,
                          MEMTXATTRS_UNSPECIFIED);
    if (!p || len != IS1GL_RING_HDR + (hwaddr)s->ring_size) {
        /*
         * A short map means the range is not one contiguous piece of RAM -
         * a bounce buffer, or straddling the end of the aperture. Either way
         * the design's premise (a raw pointer into the guest's own memory)
         * does not hold, so refuse rather than limp.
         */
        if (p) {
            address_space_unmap(&address_space_memory, p, len, true, 0);
        }
        qemu_log_mask(LOG_GUEST_ERROR,
                      "is1gl: cannot map ring at 0x%" PRIx64 " size %u\n",
                      s->ring_base, s->ring_size);
        return;
    }

    s->ring = p;
    s->ring_mapped_len = len;

    rh_put(s, RH_MAGIC, IS1GL_RING_MAGIC);
    rh_put(s, RH_VERSION, IS1GL_VERSION);
    rh_put(s, RH_SIZE, s->ring_size);
    rh_put(s, RH_HEAD, 0);
    rh_put(s, RH_TAIL, 0);
    rh_put(s, RH_DONE_SEQ, 0);
    rh_put(s, RH_ERRORS, 0);

    trace_is1gl_arm(s->ring_base, s->ring_size);
}

/* ----------------------------------------------------------------- MMIO */

static uint64_t is1gl_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    Is1glState *s = opaque;
    uint32_t val;

    switch (addr) {
    case IS1GL_REG_MAGIC:    val = IS1GL_MAGIC; break;
    case IS1GL_REG_VERSION:  val = IS1GL_VERSION; break;
    case IS1GL_REG_RING_LO:  val = (uint32_t)s->ring_base; break;
    case IS1GL_REG_RING_HI:  val = (uint32_t)(s->ring_base >> 32); break;
    case IS1GL_REG_RING_SZ:  val = s->ring_size; break;
    case IS1GL_REG_DONE_SEQ: val = qatomic_read(&s->done_seq); break;
    case IS1GL_REG_BELLS:    val = s->bells_mmio + s->bells_io; break;
    case IS1GL_REG_STATUS:   val = s->ring ? 1 : 0; break;
    case IS1GL_REG_APER_LO:  val = (uint32_t)s->aper_base; break;
    case IS1GL_REG_APER_HI:  val = (uint32_t)(s->aper_base >> 32); break;
    case IS1GL_REG_APER_SZ:  val = (uint32_t)s->aper_size; break;
    case IS1GL_REG_CAPS:
#ifdef CONFIG_PNG
        val = IS1GL_CAP_QT_PNG;
#else
        val = 0;
#endif
        break;
    default:                 val = 0; break;
    }
    trace_is1gl_mmio_read(addr, val, size);
    return val;
}

static void is1gl_doorbell(Is1glState *s, uint32_t head, const char *via)
{
    qemu_mutex_lock(&s->lock);
    s->head = head;
    if (via[0] == 'm') {
        s->bells_mmio++;
    } else {
        s->bells_io++;
    }
    qemu_cond_signal(&s->cond);
    qemu_mutex_unlock(&s->lock);
    trace_is1gl_doorbell(via, head, s->bells_mmio + s->bells_io);
}

static void is1gl_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    Is1glState *s = opaque;

    trace_is1gl_mmio_write(addr, val, size);
    switch (addr) {
    case IS1GL_REG_RING_LO:
        qemu_mutex_lock(&s->lock);
        is1gl_wait_replay_idle(s);
        is1gl_unmap_ring(s);
        s->ring_base = (s->ring_base & ~0xffffffffULL) | (uint32_t)val;
        qemu_mutex_unlock(&s->lock);
        break;
    case IS1GL_REG_RING_HI:
        qemu_mutex_lock(&s->lock);
        is1gl_wait_replay_idle(s);
        is1gl_unmap_ring(s);
        s->ring_base = (s->ring_base & 0xffffffffULL) | ((uint64_t)val << 32);
        qemu_mutex_unlock(&s->lock);
        break;
    case IS1GL_REG_RING_SZ:
        qemu_mutex_lock(&s->lock);
        is1gl_wait_replay_idle(s);
        s->ring_size = val;
        is1gl_arm_ring(s);
        qemu_mutex_unlock(&s->lock);
        break;
    case IS1GL_REG_APER_LO:
        qemu_mutex_lock(&s->lock);
        is1gl_wait_replay_idle(s);
        is1gl_unmap_aperture(s);
        s->aper_base = (s->aper_base & ~0xffffffffULL) | (uint32_t)val;
        qemu_mutex_unlock(&s->lock);
        break;
    case IS1GL_REG_APER_HI:
        qemu_mutex_lock(&s->lock);
        is1gl_wait_replay_idle(s);
        is1gl_unmap_aperture(s);
        s->aper_base = (s->aper_base & 0xffffffffULL) | ((uint64_t)val << 32);
        qemu_mutex_unlock(&s->lock);
        break;
    case IS1GL_REG_APER_SZ:
        qemu_mutex_lock(&s->lock);
        is1gl_wait_replay_idle(s);
        s->aper_size = val;
        is1gl_map_aperture(s);
        qemu_mutex_unlock(&s->lock);
        break;
    case IS1GL_REG_DOORBELL:
        is1gl_doorbell(s, val, "mmio");
        break;
    }
}

static uint64_t is1gl_io_read(void *opaque, hwaddr addr, unsigned size)
{
    Is1glState *s = opaque;

    switch (addr) {
    case IS1GL_IO_DOORBELL: return IS1GL_MAGIC;
    case IS1GL_IO_BELLS:    return s->bells_mmio + s->bells_io;
    }
    return 0;
}

static void is1gl_io_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    if (addr == IS1GL_IO_DOORBELL) {
        is1gl_doorbell(opaque, val, "ioport");
    }
}

/*
 * Both windows are dword-only. The guest side is ours, so there is no reason
 * to accept anything else, and refusing narrow access makes a mistake in the
 * guest library loud rather than subtle.
 */
static const MemoryRegionOps is1gl_mmio_ops = {
    .read = is1gl_mmio_read,
    .write = is1gl_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const MemoryRegionOps is1gl_io_ops = {
    .read = is1gl_io_read,
    .write = is1gl_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* ------------------------------------------------------------- lifecycle */

static void is1gl_reset(DeviceState *dev)
{
    Is1glState *s = IS1GL(dev);

    qemu_mutex_lock(&s->lock);
    is1gl_wait_replay_idle(s);
    is1gl_unmap_ring(s);
    is1gl_unmap_aperture(s);
    s->aper_base = 0;
    s->aper_size = 0;
    s->ring_base = 0;
    s->ring_size = 0;
    s->head = s->tail = 0;
    s->done_seq = 0;
    s->bells_mmio = s->bells_io = 0;
    s->records = s->fences = s->wraps = 0;
    s->unknown_ops = s->bad_records = 0;
    is1gl_contexts_reset(s);
    qemu_mutex_unlock(&s->lock);
}

static void is1gl_realize(DeviceState *dev, Error **errp)
{
    Is1glState *s = IS1GL(dev);
    ISADevice *isa = ISA_DEVICE(dev);

    qemu_mutex_init(&s->lock);
    qemu_cond_init(&s->cond);
    s->host_ctxs = g_hash_table_new_full(g_direct_hash, g_direct_equal,
                                               NULL, g_free);

    /*
     * The MMIO page goes in the ISA bus's memory space, which on the pc
     * machine is system memory. 0xfed10000 by default: above the window
     * SeaBIOS assigns PCI BARs from, and clear of the HPET at 0xfed00000.
     */
    memory_region_init_io(&s->mmio, OBJECT(s), &is1gl_mmio_ops, s,
                          "is1gl-mmio", IS1GL_REG_SIZE);
    memory_region_add_subregion(isa_address_space(isa), s->mmio_base,
                                &s->mmio);

    memory_region_init_io(&s->io, OBJECT(s), &is1gl_io_ops, s,
                          "is1gl-io", IS1GL_IO_SIZE);
    memory_region_add_subregion(isa_address_space_io(isa), s->io_base,
                                &s->io);

    if (s->frame_log && *s->frame_log) {
        s->frame_log_f = fopen(s->frame_log, "w");
        if (!s->frame_log_f) {
            error_setg_errno(errp, errno, "is1gl: cannot open %s",
                             s->frame_log);
            return;
        }
        setvbuf(s->frame_log_f, NULL, _IOFBF, 1 << 16);
        fprintf(s->frame_log_f, "frame,wall_us,frame_us,readback_us\n");
    }

    s->stopping = false;
    qemu_thread_create(&s->thread, "is1gl", is1gl_render_thread, s,
                       QEMU_THREAD_JOINABLE);
    s->thread_running = true;

    trace_is1gl_realize(s->mmio_base, s->io_base);
}

static void is1gl_unrealize(DeviceState *dev)
{
    Is1glState *s = IS1GL(dev);

    if (s->thread_running) {
        qemu_mutex_lock(&s->lock);
        s->stopping = true;
        qemu_cond_signal(&s->cond);
        qemu_mutex_unlock(&s->lock);
        qemu_thread_join(&s->thread);
        s->thread_running = false;
    }
    is1gl_unmap_ring(s);
    is1gl_unmap_aperture(s);
    /* After the thread is joined, so nothing can still be writing to it. */
    if (s->frame_log_f) {
        fclose(s->frame_log_f);
        s->frame_log_f = NULL;
    }
    /*
     * The OSMesa contexts are current on a thread that no longer exists and
     * go with the process; only the bookkeeping is freed here.
     */
    g_clear_pointer(&s->host_ctxs, g_hash_table_destroy);
    qemu_cond_destroy(&s->cond);
    qemu_mutex_destroy(&s->lock);
}

/* ------------------------------------------------------------------ HMP */

static Is1glState *is1gl_find(void)
{
    Object *o = object_resolve_path_type("", TYPE_IS1GL, NULL);

    return o ? IS1GL(o) : NULL;
}

void hmp_info_is1gl(Monitor *mon, const QDict *qdict)
{
    Is1glState *s = is1gl_find();

    if (!s) {
        monitor_printf(mon, "is1gl: no device\n");
        return;
    }
    monitor_printf(mon, "mmio       0x%08x   ioport 0x%04x\n",
                   s->mmio_base, s->io_base);
    monitor_printf(mon, "aperture   0x%" PRIx64 " size 0x%" PRIx64 "  %s\n",
                   s->aper_base, s->aper_size,
                   s->aper_ptr ? "mapped" : "NOT MAPPED");
    monitor_printf(mon, "ring       0x%" PRIx64 " size %u  %s\n",
                   s->ring_base, s->ring_size,
                   s->ring ? "armed" : "not armed");
    monitor_printf(mon, "head/tail  %u / %u\n", s->head, s->tail);
    if (s->ring) {
        /*
         * The header as the guest sees it, beside our own copies. If these
         * two columns ever disagree, the guest library and this device have
         * drifted apart, which is the failure this transport is most likely
         * to have and the hardest to see any other way.
         */
        monitor_printf(mon, "hdr        magic 0x%08x size %u head %u tail %u "
                       "done %u errors %u\n",
                       rh_get(s, RH_MAGIC), rh_get(s, RH_SIZE),
                       rh_get(s, RH_HEAD), rh_get(s, RH_TAIL),
                       rh_get(s, RH_DONE_SEQ), rh_get(s, RH_ERRORS));
    }
    monitor_printf(mon, "doorbells  %" PRIu64 " mmio, %" PRIu64 " ioport\n",
                   s->bells_mmio, s->bells_io);
    monitor_printf(mon, "records    %" PRIu64 "  fences %" PRIu64
                   "  wraps %" PRIu64 "\n", s->records, s->fences, s->wraps);
    monitor_printf(mon, "gl         %s  drawable %ux%u  frames %" PRIu64
                   "  readbacks %" PRIu64 "\n",
                   s->gl_failed ? "FAILED"
                                : (s->share_root ? "up" : "not started"),
                   s->draw_w, s->draw_h, s->frames, s->readbacks);
    /*
     * Read from the monitor while the render thread may be switching, as
     * the counters above are; good enough for a debugging aid.
     */
    monitor_printf(mon, "contexts   %u host  current %u  switches %" PRIu64
                   "\n", g_hash_table_size(s->host_ctxs), s->guest_ctx,
                   s->context_switches);
    {
        Is1glHostCtx *c = qatomic_read(&s->cur);

        if (c) {
            monitor_printf(mon, "viewport   %d,%d %dx%d\n",
                           c->viewport[0], c->viewport[1],
                           c->viewport[2], c->viewport[3]);
        }
    }
    if (s->frames) {
        static const char *labels[8] = {
            "   <5ms", " 5-10ms", "10-17ms", "17-25ms",
            "25-33ms", "33-50ms", "50-100ms", " >100ms"
        };
        int b;

        /* replay includes readback; readback is shown as a component of it. */
        monitor_printf(mon, "per frame  replay %.3f ms  (readback %.3f ms of "
                       "that)\n",
                       s->replay_ns / 1e6 / s->frames,
                       s->readback_ns / 1e6 / s->frames);
        monitor_printf(mon, "frame tail max %.3f ms   over 33.37ms budget: "
                       "%" PRIu64 " of %" PRIu64 "  (%.3f%%)\n",
                       s->frame_max_ns / 1e6, s->frame_over, s->frames,
                       100.0 * s->frame_over / s->frames);
        monitor_printf(mon, "frame cost");
        for (b = 0; b < 8; b++) {
            if (s->frame_hist[b]) {
                monitor_printf(mon, "  %s %" PRIu64, labels[b],
                               s->frame_hist[b]);
            }
        }
        monitor_printf(mon, "\n");
    }
    monitor_printf(mon, "gl errors  %" PRIu64 "\n", s->gl_errors);
    monitor_printf(mon, "qt png     %" PRIu64 " decoded, %" PRIu64
                   " failed, %.3f ms/decode\n", s->qt_png_decodes,
                   s->qt_png_failures,
                   s->qt_png_decodes ?
                   s->qt_png_decode_ns / 1e6 / s->qt_png_decodes : 0.0);
    monitor_printf(mon, "done_seq   %u\n", s->done_seq);
    monitor_printf(mon, "errors     %" PRIu64 " unknown opcode, %" PRIu64
                   " malformed\n", s->unknown_ops, s->bad_records);
    {
        static const char *const names[] = IS1GL_OP_NAMES;
        int i, shown = 0;

        monitor_printf(mon, "op counts  (calls actually replayed)\n");
        for (i = 0; i < IS1GL_OP_COUNT; i++) {
            if (!s->op_count[i]) {
                continue;
            }
            monitor_printf(mon, "    %-28s %" PRIu64 "\n",
                           names[i], s->op_count[i]);
            shown++;
        }
        if (!shown) {
            monitor_printf(mon, "    (none)\n");
        }
    }
    if (s->tex_log_n) {
        int i;

        monitor_printf(mon, "texture uploads (first %d)\n", s->tex_log_n);
        for (i = 0; i < s->tex_log_n; i++) {
            monitor_printf(mon,
                           "    %-4s %4ux%-4u ifmt 0x%04x fmt 0x%04x"
                           " type 0x%04x\n",
                           s->tex_log[i].sub ? "sub" : "full",
                           s->tex_log[i].w, s->tex_log[i].h,
                           s->tex_log[i].ifmt, s->tex_log[i].fmt,
                           s->tex_log[i].type);
        }
    }
}

/* --------------------------------------------------------------- object */

static const VMStateDescription vmstate_is1gl = {
    .name = "is1gl",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(ring_base, Is1glState),
        VMSTATE_UINT32(ring_size, Is1glState),
        VMSTATE_UINT32(head, Is1glState),
        VMSTATE_UINT32(tail, Is1glState),
        VMSTATE_UINT32(done_seq, Is1glState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property is1gl_properties[] = {
    DEFINE_PROP_UINT32("mmio", Is1glState, mmio_base, 0xfed10000),
    DEFINE_PROP_UINT32("iobase", Is1glState, io_base, 0x520),
    /* Per-frame host cost, one CSV row per SWAP. IS1GL_FRAMELOG= in the
     * boot script. Buffered, so it cannot become the jitter it measures. */
    DEFINE_PROP_STRING("frame-log", Is1glState, frame_log),
};

static void is1gl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = is1gl_realize;
    dc->unrealize = is1gl_unrealize;
    device_class_set_legacy_reset(dc, is1gl_reset);
    dc->vmsd = &vmstate_is1gl;
    dc->desc = "IntelliStar GL command transport";
    device_class_set_props(dc, is1gl_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo is1gl_info = {
    .name          = TYPE_IS1GL,
    .parent        = TYPE_ISA_DEVICE,
    .instance_size = sizeof(Is1glState),
    .class_init    = is1gl_class_init,
};

static void is1gl_register_types(void)
{
    type_register_static(&is1gl_info);
}

type_init(is1gl_register_types)
