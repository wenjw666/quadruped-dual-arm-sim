#ifndef P85_CAN_LINK_H
#define P85_CAN_LINK_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define P85_LINK_TOKEN0 0xfffcu
#define P85_LINK_TOKEN1 0xfffdu
typedef struct {
    uint32_t submitted, confirmed, received, length_counts[16];
} p85_can_link_direction;
/* Startup-only report, immutable after state changes to 2 (PASS) or 3 (FAIL). */
typedef struct {
    uint32_t magic, version, state, failure, phase, requested_per_direction;
    uint32_t elapsed_ms, restored, bad_metadata, bad_payload, unexpected, timeouts;
    uint32_t faults[2];
    p85_can_link_direction direction[2];
    uint32_t before[2][8], tested[2][8], after[2][8];
} p85_can_link_report_type;
extern volatile p85_can_link_report_type p85_can_link_report;
bool p85_can_link_run(void);
bool p85_can_link_rx(uint32_t controller, uint32_t id, uint32_t cs,
                     const uint8_t *data, size_t length);
void p85_can_link_tx_done(uint32_t token);
void p85_can_link_fault(uint32_t controller, uint32_t status);
#endif
