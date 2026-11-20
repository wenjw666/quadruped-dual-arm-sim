#ifndef P85_WIRE_H
#define P85_WIRE_H
#include "p85_control.h"
#define P85_WIRE_SIZE 32u
#define P85_WIRE_VERSION 1u
#define P85_MSGBOX_CMD 0x85u
/* Exactly one 32-byte MsgBox payload, all integers explicitly little endian.
 * Request: magic P85C, version u8, opcode u8, seq u16, six int32 arguments.
 * Reply: magic P85R, version u8, kind u8, seq u16, result/state/fault/status u8,
 * now_ms u32, position/velocity/torque/reference int32 (micro SI units).
 * Diagnostics kind=0x82: tx_ok/rx_ok/rx_rejected/tx_failed u32, temp i16,
 * voltage u8, fresh u8, feedback_age_ms u32. No raw C structs go on wire.
 */
typedef enum { P85_OP_HELLO=0, P85_OP_HEARTBEAT=1, P85_OP_STATUS=2,
 P85_OP_CONFIG=3, P85_OP_PROBE=4, P85_OP_ENABLE=5, P85_OP_HOLD=6,
 P85_OP_MOVE=7, P85_OP_SINE=8, P85_OP_STOP=9, P85_OP_DISABLE=10,
 P85_OP_CLEAR=11, P85_OP_DIAG=12, P85_OP_DEMO=13 } p85_opcode;
typedef struct { bool session; uint16_t last_sequence; } p85_session;
p85_error p85_dispatch(p85_control *control, p85_session *session,
 const uint8_t *request, size_t length, uint32_t now_ms,
 uint8_t response[P85_WIRE_SIZE]);
void p85_telemetry(const p85_control *c,uint16_t seq,p85_error result,
 uint32_t now,uint8_t out[P85_WIRE_SIZE]);
void p85_diagnostics(const p85_control *c,uint16_t seq,uint32_t now,
 uint8_t out[P85_WIRE_SIZE]);
#endif
