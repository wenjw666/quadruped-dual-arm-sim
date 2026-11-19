#include "p85_r2_handoff.h"
#include "Can.h"
#include "Mcu.h"
#include <stddef.h>
typedef struct {
    uint32_t magic, version, core_id, can1_active;
    uint32_t stage, heartbeat, can2_start_result, can3_start_result;
    uint32_t can1_owner, can2_active, can3_active, reserved[5];
} p85_r2_handoff_type;
volatile p85_r2_handoff_type p85_r2_handoff_report __attribute__((aligned(32)));
_Static_assert(sizeof(p85_r2_handoff_type) == 64, "handoff report layout");
static void publish(void) {
    uint32_t start = (uint32_t)(uintptr_t)&p85_r2_handoff_report;
    DcacheMaintRange(start, start + sizeof p85_r2_handoff_report, DCACHE_CLEAN_INVAL_RANGE);
}
void p85_r2_handoff_init(void) {
    p85_r2_handoff_report.magic = 0x50324331u;
    p85_r2_handoff_report.version = 1u;
    p85_r2_handoff_report.core_id = 2u;
    p85_r2_handoff_report.can1_active = Can_Config.Can_ppController[CanController_1]->Can_bActivation;
    p85_r2_handoff_report.can2_active = Can_Config.Can_ppController[CanController_2]->Can_bActivation;
    p85_r2_handoff_report.can3_active = Can_Config.Can_ppController[CanController_3]->Can_bActivation;
    p85_r2_handoff_report.can1_owner = 0u;
    p85_r2_handoff_report.stage = 1u;
    publish();
}
void p85_r2_handoff_started(uint32_t can2_result, uint32_t can3_result) {
    p85_r2_handoff_report.can2_start_result = can2_result;
    p85_r2_handoff_report.can3_start_result = can3_result;
    p85_r2_handoff_report.stage = 2u;
    publish();
}
void p85_r2_handoff_tick(void) {
    p85_r2_handoff_report.heartbeat++;
    publish();
}
