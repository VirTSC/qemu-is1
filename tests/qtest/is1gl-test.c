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
#define IS1GL_REG_STATUS    0x20

#define IS1GL_RING_BASE     0x00100000
#define IS1GL_RING_HDR      4096
#define IS1GL_RING_SIZE     4096

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
                         "contexts   2 known  current 2  switches 2",
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
                         "contexts   2 known  current 1  switches 3",
                         "viewport   360,240 360x240");

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/is1gl/context-viewport", test_context_viewport);
    return g_test_run();
}
