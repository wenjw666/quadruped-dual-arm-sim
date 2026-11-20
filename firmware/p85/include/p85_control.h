#ifndef P85_CONTROL_H
#define P85_CONTROL_H
#include "p85_protocol.h"
#define P85_HISTORY 32u
#define P85_TICK_MS 10u

typedef enum { P85_OFF, P85_PROBING, P85_DISABLED, P85_ENABLING, P85_HOLD,
               P85_MOVE, P85_SINE, P85_STOPPING, P85_DISABLING, P85_FAULT,
               P85_DEMO_NEGATIVE, P85_DEMO_CENTER_OUT, P85_DEMO_POSITIVE,
               P85_DEMO_CENTER_RETURN, P85_DEMO_POSITION_SINE, P85_SPEED_SINE } p85_state;
typedef enum { P85_OK, P85_E_STATE, P85_E_VALUE, P85_E_NO_FEEDBACK, P85_E_CONFIG,
               P85_E_MOTOR, P85_E_RX_TIMEOUT, P85_E_HOST_TIMEOUT, P85_E_ACK_TIMEOUT,
               P85_E_TRACKING, P85_E_SPEED, P85_E_TEMPERATURE, P85_E_VOLTAGE,
               P85_E_TX, P85_E_TICK, P85_E_SEQUENCE, P85_E_WIRE, P85_E_POSITION,
               P85_E_TORQUE, P85_E_STOP_TIMEOUT, P85_E_QUEUE, P85_E_SETTLE_TIMEOUT } p85_error;
typedef struct {
    uint8_t motor_id;
    float kp, kd, max_speed, max_torque, max_excursion, max_tracking_error;
    int16_t max_temperature;
    uint8_t min_voltage, max_voltage;
    uint32_t feedback_timeout_ms, heartbeat_timeout_ms, ack_timeout_ms;
    bool gains_set;
} p85_config;
typedef struct {
    uint16_t msgid; uint32_t time_ms, ordinal; bool valid;
} p85_sent;
typedef struct {
    p85_config config;
    p85_state state;
    p85_error fault;
    p85_feedback feedback;
    bool has_feedback, have_origin, tx_pending, got_tick, got_echo;
    uint32_t feedback_ms, host_ms, state_ms, last_tick_ms;
    uint32_t tx_ok, tx_failed, rx_ok, rx_rejected, tx_ordinal, echo_ordinal;
    uint16_t next_msgid, pending_msgid;
    uint8_t tx_streak;
    p85_sent history[P85_HISTORY];
    float origin, reference, reference_velocity;
    float start_position, start_velocity, goal, duration_s;
    float amplitude, period_s, ramp_s;
    uint16_t cycles;
    float demo_amplitude, demo_move_s, demo_speed_amplitude, demo_speed_period_s;
    uint16_t demo_speed_cycles, demo_repeats, demo_completed;
    float demo_sine_origin;
    uint32_t settled_ms, state_tx_floor;
    bool settling;
} p85_control;
void p85_default_config(p85_config *cfg, uint8_t motor_id);
bool p85_init(p85_control *c, const p85_config *cfg, uint32_t now_ms);
void p85_heartbeat(p85_control *c, uint32_t now_ms);
p85_error p85_configure(p85_control *c, float kp, float kd, float speed,
                         float torque, float excursion, int16_t temperature);
p85_error p85_probe(p85_control *c, uint32_t now_ms);
p85_error p85_enable(p85_control *c, uint32_t now_ms);
p85_error p85_hold(p85_control *c, uint32_t now_ms);
p85_error p85_move(p85_control *c, float offset_rad, float seconds, uint32_t now_ms);
p85_error p85_sine(p85_control *c, float amplitude, float period_s, uint16_t cycles,
                    float ramp_s, uint32_t now_ms);
p85_error p85_demo(p85_control *c, float position_amplitude, float move_seconds,
                    float speed_amplitude, float speed_period_s,
                    uint16_t speed_cycles, uint16_t repeats, uint32_t now_ms);
p85_error p85_stop(p85_control *c, uint32_t now_ms);
p85_error p85_disable(p85_control *c, uint32_t now_ms);
p85_error p85_clear_fault(p85_control *c, uint32_t now_ms);
bool p85_receive(p85_control *c, uint32_t can_id, bool fd, bool extended,
                  const uint8_t *data, size_t length, uint32_t now_ms);
/* Call in a single task exactly every 10ms; RX/IPC must be queued to that task. */
bool p85_tick(p85_control *c, uint32_t now_ms, uint8_t out[P85_TX_SIZE]);
bool p85_state_is_active(p85_state state);
bool p85_state_has_reference(p85_state state);
/* Must report driver acceptance before any new tick. TX errors cannot refresh RX. */
void p85_tx_result(p85_control *c, bool accepted, uint32_t now_ms);
void p85_trip(p85_control *c, p85_error fault, uint32_t now_ms);
const char *p85_state_name(p85_state state);
const char *p85_error_name(p85_error error);
#endif
