#include "p85_protocol.h"
#include <math.h>
#include <string.h>
uint16_t p85_get_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1]<<8)); }
void p85_put_u16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static bool in_range(float x, float lo, float hi) { return isfinite(x) && x>=lo && x<=hi; }
static uint16_t quantize(float x, float lo, float hi) {
    if (x == 0.0f && lo == -hi) return 0x8000u;
    return (uint16_t)floor(((double)x-lo)*65535.0/((double)hi-lo)+0.5);
}
static float decode(uint16_t x, float lo, float hi) {
    /* Manual names 0x8000 as zero despite its endpoint-linear equation. */
    if (lo == -hi && x == 0x8000u) return 0.0f;
    return lo+(float)x*((hi-lo)/65535.0f);
}
static bool init_frame(uint8_t *out, uint16_t msgid, uint8_t id) {
    unsigned i;
    if (!out || id<1u || id>P85_MOTORS) return false;
    memset(out,0,P85_TX_SIZE); p85_put_u16(out,msgid);
    /* This single-motor foundation explicitly disables every other slot. */
    for (i=0;i<P85_MOTORS;i++) {
        memset(out+2u+10u*i,0xff,9); out[11u+10u*i]=P85_SPECIAL_DISABLE;
    }
    return true;
}
bool p85_encode_run(uint8_t out[P85_TX_SIZE], uint16_t msgid, uint8_t id, const p85_command *c) {
    uint8_t *p;
    if (!c || !in_range(c->position,-12.5f,12.5f) || !in_range(c->velocity,-65,65) ||
        !in_range(c->kp,0,500) || !in_range(c->kd,0,50) || !in_range(c->torque,-200,200) ||
        !init_frame(out,msgid,id)) return false;
    p=out+2u+10u*(id-1u);
    p85_put_u16(p,quantize(c->position,-12.5f,12.5f));
    p85_put_u16(p+2,quantize(c->velocity,-65,65));
    p85_put_u16(p+4,quantize(c->kp,0,500));
    p85_put_u16(p+6,quantize(c->kd,0,50));
    p85_put_u16(p+8,quantize(c->torque,-200,200));
    return true;
}
bool p85_encode_special(uint8_t out[P85_TX_SIZE], uint16_t msgid, uint8_t id, uint8_t special) {
    if ((special!=P85_SPECIAL_ENABLE && special!=P85_SPECIAL_DISABLE && special!=P85_SPECIAL_ZERO) ||
        !init_frame(out,msgid,id)) return false;
    out[11u+10u*(id-1u)]=special; return true;
}
bool p85_decode_feedback(p85_feedback *out, uint32_t id, bool fd, bool ext,
                         const uint8_t *p, size_t n, uint8_t motor) {
    if (!out || !p || motor<1u || motor>6u || id!=motor || !fd || ext || n!=P85_RX_SIZE) return false;
    out->msgid=p85_get_u16(p);
    out->position=decode(p85_get_u16(p+2),-12.5f,12.5f);
    out->velocity=decode(p85_get_u16(p+4),-65,65);
    out->torque=decode(p85_get_u16(p+6),-200,200);
    out->temperature=(int16_t)p[8]-40;
    out->status=p[9]; out->voltage=p[10];
    return true;
}
