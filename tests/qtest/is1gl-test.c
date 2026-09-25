/*
 * is1gl context-state tests
 *
 * Copyright (c) 2026 VirTSC contributors
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "hw/misc/is1gl_ops.h"
#include "libqtest.h"

#define IS1GL_MMIO          0xfed10000
#define IS1GL_REG_RING_LO   0x08
#define IS1GL_REG_RING_HI   0x0c
#define IS1GL_REG_RING_SZ   0x10
#define IS1GL_REG_DOORBELL  0x14
#define IS1GL_REG_DONE_SEQ  0x18
#define IS1GL_REG_STATUS    0x20
#define IS1GL_REG_APER_LO   0x24
#define IS1GL_REG_APER_HI   0x28
#define IS1GL_REG_APER_SZ   0x2c

#define IS1GL_RING_BASE     0x00100000
#define IS1GL_RING_HDR      4096
#define IS1GL_RING_SIZE     4096
#define IS1GL_FRAME_BASE    0x00200000
#define IS1GL_FRAME_SIZE    0x00200000

#define IS1GL_RECORD_SIZE     24

static void put_make_current(uint8_t *p, uint32_t context,
                             uint32_t width, uint32_t height)
{
    stl_le_p(p + 0, IS1GL_OP_MAKE_CURRENT);
    stl_le_p(p + 4, IS1GL_RECORD_SIZE);
    stl_le_p(p + 8, context);
    stl_le_p(p + 12, width);
    stl_le_p(p + 16, height);
}

static void put_viewport(uint8_t *p, int32_t x, int32_t y,
                         int32_t width, int32_t height)
{
    stl_le_p(p + 0, IS1GL_OP_glViewport);
    stl_le_p(p + 4, IS1GL_RECORD_SIZE);
    stl_le_p(p + 8, x);
    stl_le_p(p + 12, y);
    stl_le_p(p + 16, width);
    stl_le_p(p + 20, height);
}

static char *wait_for_info(QTestState *qts, const char *needle1,
                           const char *needle2)
{
    char *info = NULL;
    int i;

    for (i = 0; i < 100; i++) {
        g_free(info);
        info = qtest_hmp(qts, "info is1gl");
        if (strstr(info, needle1) && (!needle2 || strstr(info, needle2))) {
            return info;
        }
        g_usleep(10000);
    }
    g_test_message("last info is1gl output:\n%s", info);
    g_assert_not_reached();
}

static void test_context_viewport(void)
{
    QTestState *qts;
    uint8_t records[4 * IS1GL_RECORD_SIZE] = { 0 };
    g_autofree char *info = NULL;

    qts = qtest_init("-machine pc -nodefaults -display none "
                     "-device is1gl,mmio=0xfed10000,iobase=0x520");

    qtest_writel(qts, IS1GL_MMIO + IS1GL_REG_RING_LO, IS1GL_RING_BASE);
    qtest_writel(qts, IS1GL_MMIO + IS1GL_REG_RING_HI, 0);
    qtest_writel(qts, IS1GL_MMIO + IS1GL_REG_RING_SZ, IS1GL_RING_SIZE);
    g_assert_cmphex(qtest_readl(qts, IS1GL_MMIO + IS1GL_REG_STATUS), ==, 1);

    /* Context 1 selects a quarter-screen viewport. */
    /* Context 2 must not inherit it. */
    put_make_current(records + 0 * IS1GL_RECORD_SIZE, 1, 720, 480);
    put_viewport(records + 1 * IS1GL_RECORD_SIZE, 360, 240, 360, 240);
    put_make_current(records + 2 * IS1GL_RECORD_SIZE, 2, 720, 480);
    qtest_memwrite(qts, IS1GL_RING_BASE + IS1GL_RING_HDR,
                   records, 3 * IS1GL_RECORD_SIZE);
    qtest_writel(qts, IS1GL_MMIO + IS1GL_REG_DOORBELL,
                 3 * IS1GL_RECORD_SIZE);

    info = wait_for_info(qts,
                         "contexts   2 host  current 2  switches 2",
                         "viewport   0,0 720x480");
    g_clear_pointer(&info, g_free);

    /* Switching back restores context 1's viewport rather than a default. */
    put_make_current(records + 3 * IS1GL_RECORD_SIZE, 1, 720, 480);
    qtest_memwrite(qts,
                   IS1GL_RING_BASE + IS1GL_RING_HDR + 3 * IS1GL_RECORD_SIZE,
                   records + 3 * IS1GL_RECORD_SIZE, IS1GL_RECORD_SIZE);
    qtest_writel(qts, IS1GL_MMIO + IS1GL_REG_DOORBELL,
                 4 * IS1GL_RECORD_SIZE);

    info = wait_for_info(qts,
                         "contexts   2 host  current 1  switches 3",
                         "viewport   360,240 360x240");

    qtest_quit(qts);
}

static void append_record(uint8_t *records, size_t *used, uint32_t opcode,
                          const void *payload, size_t payload_len)
{
    size_t length = (8 + payload_len + 7) & ~7;

    g_assert_cmpuint(*used + length, <=, IS1GL_RING_SIZE);
    stl_le_p(records + *used, opcode);
    stl_le_p(records + *used + 4, length);
    if (payload_len) {
        memcpy(records + *used + 8, payload, payload_len);
    }
    *used += length;
}

static void append_u32(uint8_t *records, size_t *used, uint32_t opcode,
                       uint32_t value)
{
    uint8_t payload[4];

    stl_le_p(payload, value);
    append_record(records, used, opcode, payload, sizeof(payload));
}

static void append_vertex(uint8_t *records, size_t *used, float x, float y)
{
    float point[2] = { x, y };

    append_record(records, used, IS1GL_OP_glVertex2f, point, sizeof(point));
}

static void append_texcoord(uint8_t *records, size_t *used, float s, float t)
{
    float point[2] = { s, t };

    append_record(records, used, IS1GL_OP_glTexCoord2f, point, sizeof(point));
}

static void test_context_frame(bool textured, bool padded_inverted,
                               bool loader_projection,
                               bool interleave_loader_binding)
{
    QTestState *qts;
    uint8_t records[IS1GL_RING_SIZE] = { 0 };
    uint8_t args[36] = { 0 };
    uint8_t pixel[4];
    float projection[16] = {
        2.0f / 720, 0, 0, 0,
        0, 2.0f / 480, 0, 0,
        0, 0, 1, 0,
        -1, -1, 0, 1,
    };
    float white[4] = { 1, 1, 1, 1 };
    uint8_t texture[44] = { 0 };
    uint8_t texture_param[12] = { 0 };
    uint8_t texture_binding[8] = { 0 };
    const uint32_t samples[][2] = {
        { 90, 60 }, { 630, 60 }, { 90, 420 }, { 630, 420 },
    };
    size_t used = 0;
    int i;

    qts = qtest_init("-machine pc -nodefaults -display none "
                     "-device is1gl,mmio=0xfed10000,iobase=0x520");

    qtest_writel(qts, IS1GL_MMIO + IS1GL_REG_APER_LO, IS1GL_FRAME_BASE);
    qtest_writel(qts, IS1GL_MMIO + IS1GL_REG_APER_HI, 0);
    qtest_writel(qts, IS1GL_MMIO + IS1GL_REG_APER_SZ, IS1GL_FRAME_SIZE);
    qtest_writel(qts, IS1GL_MMIO + IS1GL_REG_RING_LO, IS1GL_RING_BASE);
    qtest_writel(qts, IS1GL_MMIO + IS1GL_REG_RING_HI, 0);
    qtest_writel(qts, IS1GL_MMIO + IS1GL_REG_RING_SZ, IS1GL_RING_SIZE);

    /* A loader's quarter-screen viewport must not affect the renderer. */
    put_make_current(records + used, 1, 720, 480);
    used += IS1GL_RECORD_SIZE;
    put_viewport(records + used, 360, 240, 360, 240);
    used += IS1GL_RECORD_SIZE;
    if (textured) {
        /* The loader creates a texture in context 1. Context 2 must see it. */
        stl_le_p(texture_binding + 0, 0x0de1); /* GL_TEXTURE_2D */
        stl_le_p(texture_binding + 4, 1);
        append_record(records, &used, IS1GL_OP_glBindTexture,
                      texture_binding, sizeof(texture_binding));
        stl_le_p(texture_param + 0, 0x0de1);
        stl_le_p(texture_param + 4, 0x2801); /* GL_TEXTURE_MIN_FILTER */
        stl_le_p(texture_param + 8, 0x2600); /* GL_NEAREST */
        append_record(records, &used, IS1GL_OP_glTexParameteri,
                      texture_param, sizeof(texture_param));
        stl_le_p(texture + 0, 0x0de1); /* target */
        stl_le_p(texture + 4, 0);      /* level */
        stl_le_p(texture + 8, 0x1907); /* internal GL_RGB */
        stl_le_p(texture + 12, 2);
        stl_le_p(texture + 16, 2);
        stl_le_p(texture + 20, 0);     /* border */
        stl_le_p(texture + 24, 0x1907); /* format GL_RGB */
        stl_le_p(texture + 28, 0x1401); /* type GL_UNSIGNED_BYTE */
        memset(texture + 32, 0xff, 12);
        append_record(records, &used, IS1GL_OP_glTexImage2D,
                      texture, sizeof(texture));
    }
    if (loader_projection) {
        append_u32(records, &used, IS1GL_OP_glMatrixMode, 0x1701);
        append_record(records, &used, IS1GL_OP_glLoadIdentity, NULL, 0);
        append_record(records, &used, IS1GL_OP_glMultMatrixf,
                      projection, sizeof(projection));
        append_u32(records, &used, IS1GL_OP_glMatrixMode, 0x1700);
    }
    put_make_current(records + used, 2, 720, 480);
    used += IS1GL_RECORD_SIZE;
    if (!loader_projection) {
        append_u32(records, &used, IS1GL_OP_glMatrixMode, 0x1701);
        append_record(records, &used, IS1GL_OP_glLoadIdentity, NULL, 0);
        append_record(records, &used, IS1GL_OP_glMultMatrixf,
                      projection, sizeof(projection));
        append_u32(records, &used, IS1GL_OP_glMatrixMode, 0x1700);
    }
    append_record(records, &used, IS1GL_OP_glLoadIdentity, NULL, 0);
    append_u32(records, &used, IS1GL_OP_glClear, 0x4000);
    append_record(records, &used, IS1GL_OP_glColor4f, white, sizeof(white));
    if (textured) {
        append_u32(records, &used, IS1GL_OP_glEnable, 0x0de1);
        append_record(records, &used, IS1GL_OP_glBindTexture,
                      texture_binding, sizeof(texture_binding));
        if (interleave_loader_binding) {
            uint8_t other_binding[8];

            memcpy(other_binding, texture_binding, sizeof(other_binding));
            stl_le_p(other_binding + 4, 2);
            put_make_current(records + used, 1, 720, 480);
            used += IS1GL_RECORD_SIZE;
            append_record(records, &used, IS1GL_OP_glBindTexture,
                          other_binding, sizeof(other_binding));
            append_record(records, &used, IS1GL_OP_glTexParameteri,
                          texture_param, sizeof(texture_param));
            memset(texture + 32, 0, 12);
            append_record(records, &used, IS1GL_OP_glTexImage2D,
                          texture, sizeof(texture));
            put_make_current(records + used, 2, 720, 480);
            used += IS1GL_RECORD_SIZE;
        }
    }
    append_u32(records, &used, IS1GL_OP_glBegin, 0x0007);
    if (textured) {
        append_texcoord(records, &used, 0, 0);
    }
    append_vertex(records, &used, 0, 0);
    if (textured) {
        append_texcoord(records, &used, 1, 0);
    }
    append_vertex(records, &used, 720, 0);
    if (textured) {
        append_texcoord(records, &used, 1, 1);
    }
    append_vertex(records, &used, 720, 480);
    if (textured) {
        append_texcoord(records, &used, 0, 1);
    }
    append_vertex(records, &used, 0, 480);
    append_record(records, &used, IS1GL_OP_glEnd, NULL, 0);

    stl_le_p(args + 0, 0);       /* x */
    stl_le_p(args + 4, 0);       /* y */
    stl_le_p(args + 8, 720);
    stl_le_p(args + 12, 480);
    stl_le_p(args + 16, 0x80e1); /* GL_BGRA */
    stl_le_p(args + 20, 0x8367); /* GL_UNSIGNED_INT_8_8_8_8_REV */
    stl_le_p(args + 24, IS1GL_FRAME_BASE);
    stl_le_p(args + 28, padded_inverted ? 1024 : 720);
    stl_le_p(args + 32, padded_inverted ? 1 : 0);
    append_record(records, &used, IS1GL_OP_READPIXELS, args, sizeof(args));
    append_u32(records, &used, IS1GL_OP_FENCE, 1);

    qtest_memwrite(qts, IS1GL_RING_BASE + IS1GL_RING_HDR, records, used);
    qtest_writel(qts, IS1GL_MMIO + IS1GL_REG_DOORBELL, used);
    for (i = 0; i < 100 &&
         qtest_readl(qts, IS1GL_MMIO + IS1GL_REG_DONE_SEQ) != 1; i++) {
        g_usleep(10000);
    }
    g_assert_cmpuint(qtest_readl(qts, IS1GL_MMIO + IS1GL_REG_DONE_SEQ), ==, 1);
    for (i = 0; i < G_N_ELEMENTS(samples); i++) {
        uint32_t x = samples[i][0], y = samples[i][1];

        uint32_t row_length = padded_inverted ? 1024 : 720;

        qtest_memread(qts, IS1GL_FRAME_BASE + (y * row_length + x) * 4,
                      pixel, sizeof(pixel));
        g_assert_cmphex(pixel[0], ==, 0xff);
        g_assert_cmphex(pixel[1], ==, 0xff);
        g_assert_cmphex(pixel[2], ==, 0xff);
        g_assert_cmphex(pixel[3], ==, 0xff);
    }
    qtest_quit(qts);
}

static void test_context_full_frame(void)
{
    test_context_frame(false, false, false, false);
}

static void test_context_shared_texture(void)
{
    test_context_frame(true, false, false, false);
}

static void test_context_padded_readback(void)
{
    test_context_frame(true, true, false, false);
}

static void test_context_projection_handoff(void)
{
    test_context_frame(true, true, true, false);
}

static void test_context_binding_isolation(void)
{
    test_context_frame(true, true, true, true);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/is1gl/context-viewport", test_context_viewport);
    qtest_add_func("/is1gl/context-full-frame", test_context_full_frame);
    qtest_add_func("/is1gl/context-shared-texture",
                   test_context_shared_texture);
    qtest_add_func("/is1gl/context-padded-readback",
                   test_context_padded_readback);
    qtest_add_func("/is1gl/context-projection-handoff",
                   test_context_projection_handoff);
    qtest_add_func("/is1gl/context-binding-isolation",
                   test_context_binding_isolation);
    return g_test_run();
}
