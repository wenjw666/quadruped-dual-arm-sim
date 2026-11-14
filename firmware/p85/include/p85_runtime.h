#ifndef P85_RUNTIME_H
#define P85_RUNTIME_H
#include "p85_wire.h"
#define P85_EVENT_CAPACITY 16u
#define P85_COMMAND_MAX_AGE_MS 20u
/* The lock must work across every producer context (ISR/tasks/cores) using this
 * runtime and restore the prior interrupt state. No volatile-only lock is valid.
 * This object and its CAN owner must reside on exactly one R core.
 */
typedef struct {
    void *context;
    uintptr_t (*lock)(void *context);
    void (*unlock)(void *context, uintptr_t saved);
    bool (*can_send)(void *context, uint32_t id, bool fd, bool brs,
                     const uint8_t *data, size_t bytes);
    bool (*reply)(void *context, uint32_t cookie, const uint8_t *data, size_t bytes);
} p85_port;
typedef struct {
    uint32_t time_ms, key;
    uint8_t kind, length;
    bool fd, extended;
    uint8_t data[P85_WIRE_SIZE];
} p85_event;
typedef struct {
    p85_control control;
    p85_session session;
    p85_port port;
    p85_event queue[P85_EVENT_CAPACITY];
    unsigned read, count;
    bool overflow, bus_fault;
    uint32_t reply_failed, stale_commands;
} p85_runtime;
bool p85_runtime_init(p85_runtime *runtime, const p85_config *config,
                      const p85_port *port, uint32_t now_ms);
/* ISR/task ingress only: copies bounded data. No control or driver calls here.
 * cookie belongs to the IPC adapter; it must identify a retained reply route.
 */
bool p85_runtime_request(p85_runtime *runtime, const uint8_t *data, size_t length,
                         uint32_t cookie, uint32_t capture_ms);
bool p85_runtime_rx(p85_runtime *runtime, uint32_t arbitration_id, bool fd, bool extended,
                    const uint8_t *data, size_t length, uint32_t capture_ms);
void p85_runtime_bus_fault(p85_runtime *runtime);
/* Single 10ms task owner; never invoke from IPC or CAN callbacks. */
void p85_runtime_step(p85_runtime *runtime, uint32_t now_ms);
#endif
