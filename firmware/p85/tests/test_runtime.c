#include "p85_runtime.h"
#include <stdio.h>
#include <string.h>
static unsigned failures,checks;
#define CHECK(x) do {checks++;if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);failures++;}}while(0)
typedef struct {unsigned sends,replies,locked;bool accept,reply_ok;uint32_t cookie;uint8_t tx[64],reply[32];} fixture;
static uintptr_t lock(void *ctx) {fixture *f=ctx;CHECK(f->locked==0);f->locked++;return 7;}
static void unlock(void *ctx,uintptr_t token) {fixture *f=ctx;CHECK(token==7&&f->locked==1);f->locked--;}
static bool send_can(void *ctx,uint32_t id,bool fd,bool brs,const uint8_t *data,size_t n) {
 fixture *f=ctx;CHECK(!f->locked&&id==0&&fd&&brs&&n==64);memcpy(f->tx,data,64);f->sends++;return f->accept;
}
static bool send_reply(void *ctx,uint32_t cookie,const uint8_t *data,size_t n) {
 fixture *f=ctx;CHECK(!f->locked&&n==32);f->replies++;f->cookie=cookie;memcpy(f->reply,data,32);return f->reply_ok;
}
static void init(p85_runtime *r,fixture *f) {
 p85_config cfg;p85_port port={f,lock,unlock,send_can,send_reply};
 memset(f,0,sizeof(*f));f->accept=true;f->reply_ok=true;
 p85_default_config(&cfg,3);cfg.kp=5;cfg.kd=0.1f;cfg.gains_set=true;
 CHECK(p85_runtime_init(r,&cfg,&port,0));
}
static void command(p85_runtime *r,unsigned op,uint16_t seq,uint32_t now) {
 uint8_t data[32]={0};memcpy(data,"P85C",4);data[4]=1;data[5]=(uint8_t)op;p85_put_u16(data+6,seq);
 CHECK(p85_runtime_request(r,data,32,123,now));
}
static void echo(p85_runtime *r,const fixture *f,uint8_t status,uint32_t now) {
 uint8_t fb[12]={0,0,0,128,0,128,0,128,65,0,48,0};
 memcpy(fb,f->tx,2);fb[9]=status;CHECK(p85_runtime_rx(r,3,true,false,fb,12,now));
}
static void ready(p85_runtime *r,fixture *f) {
 init(r,f);command(r,P85_OP_HELLO,1,0);command(r,P85_OP_PROBE,2,0);
 CHECK(f->sends==0&&f->replies==0);p85_runtime_step(r,0);
 CHECK(f->cookie==123&&f->sends==1&&f->replies==2);echo(r,f,0,1);p85_runtime_step(r,10);
 CHECK(r->control.state==P85_DISABLED);command(r,P85_OP_ENABLE,3,10);p85_runtime_step(r,20);
 echo(r,f,1,21);p85_runtime_step(r,30);CHECK(r->control.state==P85_HOLD);
}
int main(void) {
 p85_runtime r;fixture f;uint8_t data[32]={0};
 init(&r,&f);p85_runtime_step(&r,0);CHECK(f.sends==0);
 CHECK(!p85_runtime_request(&r,data,31,0,0));CHECK(!p85_runtime_rx(&r,3,true,false,data,32,0));
 ready(&r,&f);CHECK(r.control.rx_ok==2);
 p85_runtime_bus_fault(&r);CHECK(r.control.state==P85_HOLD);p85_runtime_step(&r,40);
 CHECK(r.control.state==P85_FAULT&&r.control.fault==P85_E_TX&&f.tx[31]==0xfd);
 ready(&r,&f);for(unsigned i=0;i<P85_EVENT_CAPACITY;i++)command(&r,P85_OP_STATUS,(uint16_t)(4+i),30);
 CHECK(!p85_runtime_request(&r,data,32,0,30));p85_runtime_step(&r,40);
 CHECK(r.control.fault==P85_E_QUEUE&&f.tx[31]==0xfd);
 init(&r,&f);command(&r,P85_OP_HELLO,1,0);p85_runtime_step(&r,21);
 CHECK(r.stale_commands==1&&!r.session.session&&f.reply[8]==P85_E_TICK&&f.sends==0);
 ready(&r,&f);f.reply_ok=false;command(&r,P85_OP_STATUS,4,30);p85_runtime_step(&r,40);
 CHECK(r.reply_failed==1&&r.control.fault==P85_E_HOST_TIMEOUT&&f.tx[31]==0xfd);
 ready(&r,&f);f.accept=false;for(uint32_t t=40;t<=60;t+=10)p85_runtime_step(&r,t);
 CHECK(r.control.fault==P85_E_TX&&r.control.tx_failed==3);
 ready(&r,&f);echo(&r,&f,1,31);p85_runtime_step(&r,200);
 CHECK(r.control.rx_rejected>0&&r.control.state==P85_FAULT);
 CHECK(!strcmp(p85_state_name((p85_state)-1),"invalid"));
 CHECK(!strcmp(p85_error_name((p85_error)-1),"invalid"));
 /* State handshake must also work across the 32-bit transmit ordinal wrap. */
 init(&r,&f);r.control.tx_ordinal=0xffffffffu;command(&r,P85_OP_HELLO,1,0);
 command(&r,P85_OP_PROBE,2,0);p85_runtime_step(&r,0);echo(&r,&f,0,1);p85_runtime_step(&r,10);
 CHECK(r.control.state==P85_DISABLED&&r.control.tx_ordinal==1);
 printf("P85 runtime: %u checks, %u failures\n",checks,failures);return failures?1:0;
}
