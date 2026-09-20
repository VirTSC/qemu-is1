/*
 * is1gl stub, for builds without OpenGL. hmp-commands-info.hx references
 * hmp_info_is1gl unconditionally, so it has to exist in every target.
 */
#include "qemu/osdep.h"
#include "monitor/monitor.h"
#include "monitor/hmp.h"

void hmp_info_is1gl(Monitor *mon, const QDict *qdict)
{
    monitor_printf(mon, "is1gl: this QEMU was built without OpenGL\n");
}
