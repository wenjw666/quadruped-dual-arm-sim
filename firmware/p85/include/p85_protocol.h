#ifndef P85_PROTOCOL_H
#define P85_PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define P85_TX_SIZE 64u
#define P85_RX_SIZE 12u
#define P85_MOTORS 6u
#define P85_CAN_HOST_ID 0u
#define P85_STATUS_ENABLED 0x01u
#define P85_STATUS_ERRORS 0x7eu
#define P85_SPECIAL_ENABLE 0xfcu
#define P85_SPECIAL_DISABLE 0xfdu
#define P85_SPECIAL_ZERO 0xefu

typedef struct { float position, velocity, kp, kd, torque; } p85_command;
typedef struct {
    uint16_t msgid;
    float position, velocity, torque;
    int16_t temperature;
    uint8_t status, voltage;
} p85_feedback;
/* Reject nonfinite/out-of-range fields; never silently clip a command. */
bool p85_encode_run(uint8_t out[P85_TX_SIZE], uint16_t msgid,
                    uint8_t motor_id, const p85_command *command);
bool p85_encode_special(uint8_t out[P85_TX_SIZE], uint16_t msgid,
                        uint8_t motor_id, uint8_t special);
bool p85_decode_feedback(p85_feedback *out, uint32_t can_id, bool fd,
                         bool extended, const uint8_t *data, size_t length,
                         uint8_t motor_id);
uint16_t p85_get_u16(const uint8_t *p);
void p85_put_u16(uint8_t *p, uint16_t value);
#endif
