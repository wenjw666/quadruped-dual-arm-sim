#ifndef P85_REPORT_H
#define P85_REPORT_H
#include <stdint.h>
#include <stddef.h>
#include "Mcu.h"
/* R0 SRAM is cacheable and is read by Linux through a noncoherent /dev/mem
 * mapping. volatile alone does not publish dirty cache lines to the A core.
 * The pinned BSP cleans+invalidates each covering line and finishes with DSB.
 * Call only when IRQ callbacks cannot mutate this report. */
static inline void p85_report_publish(const volatile void *report, size_t size) {
    uint32_t start = (uint32_t)(uintptr_t)report;
    DcacheMaintRange(start, start + (uint32_t)size, DCACHE_CLEAN_INVAL_RANGE);
}
#endif
