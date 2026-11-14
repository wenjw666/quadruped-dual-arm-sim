#include "p85_wire.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
static unsigned checks=0,failures=0;
#define CHECK(x) do {checks++;if(!(x)){failures++;fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x);}} while(0)
static uint16_t q(float x,float lo,float hi) {return (uint16_t)floor(((double)x-lo)*65535/(hi-lo)+0.5);}
static void feedback(uint8_t *fb,uint16_t seq,float p,float v,float torque,uint8_t status) {
 memset(fb,0,12);p85_put_u16(fb,seq);p85_put_u16(fb+2,q(p,-12.5f,12.5f));
 p85_put_u16(fb+4,q(v,-65,65));p85_put_u16(fb+6,q(torque,-200,200));fb[8]=65;fb[9]=status;fb[10]=48;
}
static void init(p85_control *c) {
 p85_config cfg;p85_default_config(&cfg,3);CHECK(p85_init(c,&cfg,0));
 CHECK(p85_configure(c,10,1,0.5f,2,0.2f,60)==P85_OK);
}
static void step(p85_control *c,uint32_t t,bool reply) {
 uint8_t tx[64],fb[12];p85_heartbeat(c,t);
 if(p85_tick(c,t,tx)) {
  p85_tx_result(c,true,t);
  if(reply) {
   float p=c->have_origin?c->reference:0,v=c->reference_velocity;
   uint8_t status=(c->state>=P85_ENABLING&&c->state<=P85_STOPPING)?1:0;
   feedback(fb,p85_get_u16(tx),p,v,0,status);CHECK(p85_receive(c,3,true,false,fb,12,t));
  }
 }
}
static void ready(p85_control *c) {
 init(c);CHECK(p85_probe(c,0)==P85_OK);step(c,0,true);CHECK(c->state==P85_DISABLED);
 CHECK(p85_enable(c,10)==P85_OK);step(c,10,true);CHECK(c->state==P85_HOLD);
}
static void test_protocol(void) {
 uint8_t tx[64],fb[12];p85_feedback result;p85_command c={0,0,0,0,0};
 const uint8_t zero_slot[10]={0,128,0,128,0,0,0,0,0,128};
 CHECK(p85_encode_run(tx,0x1234,3,&c));CHECK(tx[0]==0x34&&tx[1]==0x12);
 CHECK(!memcmp(tx+22,zero_slot,10));CHECK(tx[62]==0&&tx[63]==0);
 for(unsigned id=1;id<=6;id++)if(id!=3) {
  for(unsigned j=0;j<9;j++)CHECK(tx[2+(id-1)*10+j]==0xff);
  CHECK(tx[11+(id-1)*10]==0xfd);
 }
 c.position=-12.5f;c.velocity=65;c.kp=500;c.kd=50;c.torque=-200;
 CHECK(p85_encode_run(tx,0xffff,1,&c));CHECK(p85_get_u16(tx+2)==0);
 CHECK(p85_get_u16(tx+4)==65535);CHECK(p85_get_u16(tx+6)==65535);
 CHECK(p85_get_u16(tx+8)==65535);CHECK(p85_get_u16(tx+10)==0);
 c.position=NAN;CHECK(!p85_encode_run(tx,0,1,&c));c.position=12.501f;CHECK(!p85_encode_run(tx,0,1,&c));
 CHECK(!p85_encode_special(tx,0,0,0xfc));CHECK(!p85_encode_special(tx,0,7,0xfc));
 CHECK(!p85_encode_special(tx,0,3,0xfe));
 CHECK(p85_encode_special(tx,2,3,0xfc));for(unsigned j=0;j<9;j++)CHECK(tx[22+j]==0xff);CHECK(tx[31]==0xfc);
 feedback(fb,0xabcd,1.25f,-2.5f,43,0x25);fb[8]=100;fb[10]=60;
 CHECK(p85_decode_feedback(&result,3,true,false,fb,12,3));
 CHECK(result.msgid==0xabcd&&fabsf(result.position-1.25f)<0.0002f);
 CHECK(fabsf(result.velocity+2.5f)<0.0011f&&fabsf(result.torque-43)<0.0031f);
 CHECK(result.temperature==60&&result.status==0x25&&result.voltage==60);
 CHECK(!p85_decode_feedback(&result,3,true,false,fb,8,3));CHECK(!p85_decode_feedback(&result,2,true,false,fb,12,3));
 CHECK(!p85_decode_feedback(&result,3,false,false,fb,12,3));CHECK(!p85_decode_feedback(&result,3,true,true,fb,12,3));
 feedback(fb,0,0,0,0,0);CHECK(p85_decode_feedback(&result,3,true,false,fb,12,3));
 CHECK(result.position==0&&result.velocity==0&&result.torque==0);
}
static void test_state_and_timeouts(void) {
 p85_control c;uint8_t tx[64],fb[12];init(&c);
 CHECK(!p85_tick(&c,0,tx));CHECK(p85_enable(&c,0)==P85_E_STATE);
 CHECK(p85_probe(&c,0)==P85_OK);for(uint32_t t=0;t<=510;t+=10)step(&c,t,false);
 CHECK(c.state==P85_FAULT&&c.fault==P85_E_ACK_TIMEOUT);
 CHECK(p85_clear_fault(&c,510)==P85_E_NO_FEEDBACK);
 ready(&c);for(uint32_t t=20;t<=120;t+=10)step(&c,t,false);
 CHECK(c.state==P85_FAULT&&c.fault==P85_E_RX_TIMEOUT);
 step(&c,130,true);CHECK(p85_clear_fault(&c,130)==P85_OK);CHECK(c.state==P85_DISABLED);
 ready(&c);for(uint32_t t=20;t<=320;t+=10) {
  bool sent=p85_tick(&c,t,tx);CHECK(sent);p85_tx_result(&c,true,t);
  feedback(fb,p85_get_u16(tx),0,0,0,c.state==P85_FAULT?0:1);p85_receive(&c,3,true,false,fb,12,t);
 }
 CHECK(c.state==P85_FAULT&&c.fault==P85_E_HOST_TIMEOUT);
 ready(&c);p85_heartbeat(&c,40);CHECK(p85_tick(&c,40,tx));CHECK(c.fault==P85_E_TICK);p85_tx_result(&c,true,40);
 ready(&c);for(uint32_t t=20;t<=40;t+=10){p85_heartbeat(&c,t);CHECK(p85_tick(&c,t,tx));p85_tx_result(&c,false,t);}
 CHECK(c.state==P85_FAULT&&c.fault==P85_E_TX);
 ready(&c);CHECK(p85_disable(&c,20)==P85_OK);step(&c,20,true);CHECK(c.state==P85_DISABLED);
 init(&c);c.config.gains_set=false;p85_probe(&c,0);step(&c,0,true);CHECK(p85_enable(&c,10)==P85_E_CONFIG);
}
static void test_feedback_validation(void) {
 p85_control c;uint8_t tx[64],fb[12];ready(&c);
 feedback(fb,0x1234,0,0,0,1);CHECK(!p85_receive(&c,3,true,false,fb,12,20));
 CHECK(p85_tick(&c,20,tx));p85_tx_result(&c,true,20);feedback(fb,p85_get_u16(tx),0,0,0,1);
 CHECK(p85_receive(&c,3,true,false,fb,12,20));CHECK(!p85_receive(&c,3,true,false,fb,12,21));
 CHECK(p85_tick(&c,30,tx));p85_tx_result(&c,false,30);feedback(fb,p85_get_u16(tx),0,0,0,1);
 CHECK(!p85_receive(&c,3,true,false,fb,12,30));
 ready(&c);CHECK(p85_tick(&c,20,tx));p85_tx_result(&c,true,20);feedback(fb,p85_get_u16(tx),0,-0.6f,0,1);
 CHECK(p85_receive(&c,3,true,false,fb,12,20));step(&c,30,false);CHECK(c.fault==P85_E_SPEED);
 ready(&c);CHECK(p85_tick(&c,20,tx));p85_tx_result(&c,true,20);feedback(fb,p85_get_u16(tx),0,0,0,0x81);
 CHECK(p85_receive(&c,3,true,false,fb,12,20));step(&c,30,false);CHECK(c.state==P85_HOLD); /* reserved bit 7 */
 ready(&c);CHECK(p85_tick(&c,20,tx));p85_tx_result(&c,true,20);feedback(fb,p85_get_u16(tx),0,0,0,0x05);
 CHECK(p85_receive(&c,3,true,false,fb,12,20));step(&c,30,false);CHECK(c.fault==P85_E_MOTOR);
 ready(&c);CHECK(p85_tick(&c,20,tx));p85_tx_result(&c,true,20);feedback(fb,p85_get_u16(tx),0,0,0,1);fb[10]=23;
 CHECK(p85_receive(&c,3,true,false,fb,12,20));step(&c,30,false);CHECK(c.fault==P85_E_VOLTAGE);
 ready(&c);CHECK(p85_tick(&c,20,tx));p85_tx_result(&c,true,20);feedback(fb,p85_get_u16(tx),0,0,0,1);fb[8]=100;
 CHECK(p85_receive(&c,3,true,false,fb,12,20));step(&c,30,false);CHECK(c.fault==P85_E_TEMPERATURE);
}
static void test_trajectory(void) {
 p85_control c;float peak=0,previous,prev_velocity;ready(&c);
 CHECK(p85_move(&c,0.3f,2,10)==P85_E_VALUE);CHECK(p85_move(&c,0.1f,0.2f,10)==P85_E_VALUE);
 CHECK(p85_move(&c,0.1f,2,10)==P85_OK);
 for(uint32_t t=20;t<2500;t+=10){step(&c,t,true);CHECK(c.state!=P85_FAULT);}
 CHECK(c.state==P85_HOLD&&fabsf(c.reference-0.1f)<0.00001f);
 CHECK(p85_move(&c,0,2,2500)==P85_OK);for(uint32_t t=2500;t<=4900;t+=10)step(&c,t,true);
 CHECK(c.state==P85_HOLD);CHECK(p85_sine(&c,0.1f,4,3,2,4900)==P85_OK);
 previous=c.reference;prev_velocity=0;
 for(uint32_t t=4910;t<=21600;t+=10) {
  step(&c,t,true);CHECK(c.state!=P85_FAULT);CHECK(fabsf(c.reference_velocity)<=0.5f);
  CHECK(fabsf(c.reference-previous)<=0.006f);CHECK(fabsf(c.reference_velocity-prev_velocity)<0.02f);
  if(t>=6900&&t<=18900&&fabsf(c.reference)>peak)peak=fabsf(c.reference);
  if(t==7900)CHECK(fabsf(c.reference-0.1f)<0.00001f);
  if(t==9900)CHECK(fabsf(c.reference+0.1f)<0.00001f);
  previous=c.reference;prev_velocity=c.reference_velocity;
 }
 CHECK(fabsf(peak-0.1f)<0.00001f);CHECK(c.state==P85_HOLD);CHECK(fabsf(c.reference)<0.00001f);
 CHECK(p85_sine(&c,0.1f,4,3,2,21600)==P85_OK);for(uint32_t t=21600;t<=24700;t+=10)step(&c,t,true);
 previous=c.reference;prev_velocity=c.reference_velocity;CHECK(p85_stop(&c,24700)==P85_OK);
 step(&c,24700,true);CHECK(fabsf(c.reference-previous)<0.00001f&&fabsf(c.reference_velocity-prev_velocity)<0.00001f);
 for(uint32_t t=24710;t<=25700;t+=10){step(&c,t,true);CHECK(c.state!=P85_FAULT);}
 CHECK(c.state==P85_HOLD&&fabsf(c.reference_velocity)<0.00001f);
}
static void put32(uint8_t *p,int32_t v){uint32_t u=(uint32_t)v;for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(u>>(8*i));}
static void request(uint8_t *r,uint8_t op,uint16_t seq) {
 memset(r,0,32);memcpy(r,"P85C",4);r[4]=1;r[5]=op;p85_put_u16(r+6,seq);
}
static void test_wire_and_wrap(void) {
 p85_control c;p85_session s={false,0};uint8_t r[32],out[32],tx[64],fb[12];init(&c);
 request(r,P85_OP_ENABLE,1);CHECK(p85_dispatch(&c,&s,r,32,0,out)==P85_E_SEQUENCE);
 request(r,P85_OP_HELLO,65534);CHECK(p85_dispatch(&c,&s,r,32,0,out)==P85_OK);
 request(r,P85_OP_PROBE,65535);CHECK(p85_dispatch(&c,&s,r,32,0,out)==P85_OK);
 CHECK(p85_dispatch(&c,&s,r,32,1,out)==P85_E_SEQUENCE);CHECK(c.host_ms==0);
 request(r,P85_OP_HEARTBEAT,0);CHECK(p85_dispatch(&c,&s,r,32,2,out)==P85_OK);
 CHECK(!memcmp(out,"P85R",4)&&out[4]==1&&out[9]==P85_PROBING);
 CHECK(p85_dispatch(&c,&s,r,31,3,out)==P85_E_WIRE);
 request(r,P85_OP_SINE,1);put32(r+16,65537);CHECK(p85_dispatch(&c,&s,r,32,4,out)==P85_E_VALUE);
 ready(&c);s.session=true;s.last_sequence=10;request(r,P85_OP_HELLO,11);
 CHECK(p85_dispatch(&c,&s,r,32,10,out)==P85_E_STATE);
 /* MSGID wrap, monotonic tick wrap, duplicate and delayed feedback. */
 init(&c);c.next_msgid=65535;p85_probe(&c,0xfffffff0u);
 CHECK(p85_tick(&c,0xfffffff0u,tx));p85_tx_result(&c,true,0xfffffff0u);
 feedback(fb,65535,0,0,0,0);CHECK(p85_receive(&c,3,true,false,fb,12,0xfffffff0u));
 p85_heartbeat(&c,0xfffffffau);CHECK(p85_enable(&c,0xfffffffau)==P85_OK);
 CHECK(p85_tick(&c,0xfffffffau,tx));CHECK(p85_get_u16(tx)==0);p85_tx_result(&c,true,0xfffffffau);
 feedback(fb,0,0,0,0,1);CHECK(p85_receive(&c,3,true,false,fb,12,0xfffffffau));
 p85_heartbeat(&c,4);CHECK(p85_tick(&c,4,tx));p85_tx_result(&c,true,4);CHECK(c.state==P85_HOLD);
 CHECK(!p85_receive(&c,3,true,false,fb,12,105));
}
int main(void) {
 test_protocol();test_state_and_timeouts();test_feedback_validation();test_trajectory();test_wire_and_wrap();
 printf("P85: %u checks, %u failures\n",checks,failures);return failures?1:0;
}
