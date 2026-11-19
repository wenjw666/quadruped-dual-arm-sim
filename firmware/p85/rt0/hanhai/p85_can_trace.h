#ifndef P85_CAN_TRACE_H
#define P85_CAN_TRACE_H
#include <stdint.h>
/* Diagnostic events are emitted only by R0 startup code, never IRQ callbacks.
 * No A-core reads of CAN MMIO are needed. Values are native FlexCAN statuses. */
#define P85_CAN_TRACE_CAPACITY 96u
#define P85_CAN_TRACE_WORDS (8u + 16u * P85_CAN_TRACE_CAPACITY)
enum {
    P85_TRACE_INIT_ENTER=1, P85_TRACE_FREEZE=2, P85_TRACE_DISABLE=3,
    P85_TRACE_RESET=4, P85_TRACE_FIFO=5, P85_TRACE_MAX_MB=6,
    P85_TRACE_INIT_RETURN=7, P85_TRACE_START_ENTER=8,
    P85_TRACE_START_RETURN=9, P85_TRACE_FINISH=10, P85_TRACE_EXIT_FREEZE=11
};
#if P85_BOOT_CAN_LINK
void p85_can_trace(uint32_t instance, uint32_t stage, uint32_t status, uint32_t aux);
void p85_can_trace_finish(uint32_t failure);
#else
static inline void p85_can_trace(uint32_t instance, uint32_t stage, uint32_t status, uint32_t aux) {
    (void)instance; (void)stage; (void)status; (void)aux;
}
static inline void p85_can_trace_finish(uint32_t failure) { (void)failure; }
#endif
#endif
