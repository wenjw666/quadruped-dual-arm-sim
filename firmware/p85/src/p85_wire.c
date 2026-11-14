#include "p85_wire.h"
#include <limits.h>
#include <math.h>
#include <string.h>
static uint32_t get32(const uint8_t *p) {
 return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
}
static void put32(uint8_t *p,uint32_t x) {
 p[0]=(uint8_t)x;p[1]=(uint8_t)(x>>8);p[2]=(uint8_t)(x>>16);p[3]=(uint8_t)(x>>24);
}
static int32_t arg(const uint8_t *p,unsigned i) {
 uint32_t u=get32(p+8+4*i);int32_t x;memcpy(&x,&u,4);return x;
}
static int32_t micro(float x) {
 double v=round((double)x*1000000.0);
 if(v>INT32_MAX)return INT32_MAX;
 if(v<INT32_MIN)return INT32_MIN;
 return (int32_t)v;
}
static void header(uint8_t *p,uint8_t kind,uint16_t seq) {
 memset(p,0,P85_WIRE_SIZE);memcpy(p,"P85R",4);p[4]=P85_WIRE_VERSION;p[5]=kind;p85_put_u16(p+6,seq);
}
void p85_telemetry(const p85_control *c,uint16_t seq,p85_error e,uint32_t now,uint8_t out[P85_WIRE_SIZE]) {
 header(out,0x80,seq);out[8]=(uint8_t)e;out[9]=(uint8_t)c->state;
 out[10]=(uint8_t)c->fault;out[11]=c->has_feedback?c->feedback.status:0;
 put32(out+12,now);put32(out+16,(uint32_t)micro(c->feedback.position));
 put32(out+20,(uint32_t)micro(c->feedback.velocity));put32(out+24,(uint32_t)micro(c->feedback.torque));
 put32(out+28,(uint32_t)micro(c->reference));
}
void p85_diagnostics(const p85_control *c,uint16_t seq,uint32_t now,uint8_t out[P85_WIRE_SIZE]) {
 uint32_t a=c->has_feedback?now-c->feedback_ms:UINT32_MAX;
 header(out,0x82,seq);put32(out+8,c->tx_ok);put32(out+12,c->rx_ok);
 put32(out+16,c->rx_rejected);put32(out+20,c->tx_failed);
 p85_put_u16(out+24,(uint16_t)c->feedback.temperature);out[26]=c->feedback.voltage;
 out[27]=(uint8_t)(c->has_feedback&&a<=c->config.feedback_timeout_ms);put32(out+28,a);
}
p85_error p85_dispatch(p85_control *c,p85_session *s,const uint8_t *r,size_t n,
 uint32_t now,uint8_t out[P85_WIRE_SIZE]) {
 p85_error e=P85_OK;uint16_t seq=n>=8?p85_get_u16(r+6):0;unsigned op=n>=6?r[5]:255;
 if(n!=P85_WIRE_SIZE||memcmp(r,"P85C",4)||r[4]!=P85_WIRE_VERSION) e=P85_E_WIRE;
 else if(op>P85_OP_DIAG)e=P85_E_WIRE;
 else if(op==P85_OP_HELLO) {
  if(c->state>=P85_ENABLING&&c->state<=P85_STOPPING)e=P85_E_STATE;
  else {s->session=true;s->last_sequence=seq;p85_heartbeat(c,now);}
 } else if(!s->session)e=P85_E_SEQUENCE;
 else if((uint16_t)(seq-s->last_sequence)==0 || (uint16_t)(seq-s->last_sequence)>32767u)e=P85_E_SEQUENCE;
 else {
  s->last_sequence=seq;
  /* Only a valid session command refreshes the lease. Rejected replays do not. */
  p85_heartbeat(c,now);
  switch(op) {
  case P85_OP_HEARTBEAT:case P85_OP_STATUS:case P85_OP_DIAG:break;
  case P85_OP_CONFIG:
   if(arg(r,5)<20||arg(r,5)>85)e=P85_E_VALUE;
   else e=p85_configure(c,arg(r,0)*0.001f,arg(r,1)*0.001f,arg(r,2)*0.000001f,
       arg(r,3)*0.001f,arg(r,4)*0.000001f,(int16_t)arg(r,5));
   break;
  case P85_OP_PROBE:e=p85_probe(c,now);break;
  case P85_OP_ENABLE:e=p85_enable(c,now);break;
  case P85_OP_HOLD:e=p85_hold(c,now);break;
  case P85_OP_MOVE:e=p85_move(c,arg(r,0)*0.000001f,arg(r,1)*0.001f,now);break;
  case P85_OP_SINE:
   if(arg(r,2)<1||arg(r,2)>100)e=P85_E_VALUE;
   else e=p85_sine(c,arg(r,0)*0.000001f,arg(r,1)*0.001f,(uint16_t)arg(r,2),arg(r,3)*0.001f,now);
   break;
  case P85_OP_STOP:e=p85_stop(c,now);break;
  case P85_OP_DISABLE:e=p85_disable(c,now);break;
  case P85_OP_CLEAR:e=p85_clear_fault(c,now);break;
  default:e=P85_E_WIRE;break;
  }
 }
 if(op==P85_OP_DIAG&&e==P85_OK)p85_diagnostics(c,seq,now,out);
 else p85_telemetry(c,seq,e,now,out);
 return e;
}
