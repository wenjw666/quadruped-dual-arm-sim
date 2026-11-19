#ifndef P85_CAN_LOOPBACK_H
#define P85_CAN_LOOPBACK_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define P85_LOOPBACK_TOKEN 0xfffeu
/* Immutable after startup. All fields are 32-bit for the A-core reader. */
typedef struct {
    uint32_t magic, version, state, failure, requested, submitted, confirmed;
    uint32_t received, bad_metadata, bad_payload, unexpected, timeouts, faults;
    uint32_t elapsed_ms, restored, length_counts[16];
    uint32_t before[8], loopback[8], after[8];
} p85_can_loopback_report_type;
extern volatile p85_can_loopback_report_type p85_can_loopback_report;
bool p85_can_loopback_run(void);
bool p85_can_loopback_rx(uint32_t id, uint32_t cs, const uint8_t *data, size_t n);
void p85_can_loopback_tx_done(void);
void p85_can_loopback_fault(void);
#endif
