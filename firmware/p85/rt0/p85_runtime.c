#include "p85_runtime.h"
#include <string.h>
#define EVENT_REQUEST 1u
#define EVENT_RX 2u
bool p85_runtime_init(p85_runtime *r,const p85_config *cfg,const p85_port *port,uint32_t now) {
    if(!r||!port||!port->lock||!port->unlock||!port->can_send||!port->reply)return false;
    memset(r,0,sizeof(*r));r->port=*port;
    return p85_init(&r->control,cfg,now);
}
static bool enqueue(p85_runtime *r,const p85_event *event) {
    uintptr_t saved=r->port.lock(r->port.context);
    bool ok=r->count<P85_EVENT_CAPACITY;
    if(ok){r->queue[(r->read+r->count)%P85_EVENT_CAPACITY]=*event;r->count++;}
    else r->overflow=true;
    r->port.unlock(r->port.context,saved);return ok;
}
bool p85_runtime_request(p85_runtime *r,const uint8_t *data,size_t n,uint32_t cookie,uint32_t now) {
    p85_event e;
    if(!data||n!=P85_WIRE_SIZE)return false;
    memset(&e,0,sizeof(e));e.kind=EVENT_REQUEST;e.length=(uint8_t)n;e.time_ms=now;e.key=cookie;
    memcpy(e.data,data,n);return enqueue(r,&e);
}
bool p85_runtime_rx(p85_runtime *r,uint32_t id,bool fd,bool ext,const uint8_t *data,size_t n,uint32_t now) {
    p85_event e;
    if(!data||n!=P85_RX_SIZE)return false;
    memset(&e,0,sizeof(e));e.kind=EVENT_RX;e.length=(uint8_t)n;e.time_ms=now;e.key=id;e.fd=fd;e.extended=ext;
    memcpy(e.data,data,n);return enqueue(r,&e);
}
void p85_runtime_bus_fault(p85_runtime *r) {
    uintptr_t saved=r->port.lock(r->port.context);r->bus_fault=true;
    r->port.unlock(r->port.context,saved);
}
static bool pop(p85_runtime *r,p85_event *e,bool *overflow,bool *bus_fault) {
    uintptr_t saved=r->port.lock(r->port.context);
    bool present=r->count>0;
    *overflow=r->overflow;*bus_fault=r->bus_fault;r->overflow=false;r->bus_fault=false;
    if(present){*e=r->queue[r->read];r->read=(r->read+1u)%P85_EVENT_CAPACITY;r->count--;}
    r->port.unlock(r->port.context,saved);return present;
}
void p85_runtime_step(p85_runtime *r,uint32_t now) {
    uint8_t reply[P85_WIRE_SIZE],tx[P85_TX_SIZE];
    /* Snapshot budget bounds task execution even if a producer never stops. */
    for(unsigned i=0;i<P85_EVENT_CAPACITY;i++) {
        p85_event e;bool overflow,bus_fault;
        bool present=pop(r,&e,&overflow,&bus_fault);
        if(overflow)p85_trip(&r->control,P85_E_QUEUE,now);
        if(bus_fault)p85_trip(&r->control,P85_E_TX,now);
        if(!present)break;
        if(e.kind==EVENT_RX) {
            if(now-e.time_ms<=r->control.config.feedback_timeout_ms)
                p85_receive(&r->control,e.key,e.fd,e.extended,e.data,e.length,e.time_ms);
            else r->control.rx_rejected++;
        } else {
            if(now-e.time_ms>P85_COMMAND_MAX_AGE_MS) {
                r->stale_commands++;
                /* Expired commands cannot renew the lease or execute motion. */
                p85_telemetry(&r->control,p85_get_u16(e.data+6),P85_E_TICK,now,reply);
            } else p85_dispatch(&r->control,&r->session,e.data,e.length,now,reply);
            if(!r->port.reply(r->port.context,e.key,reply,sizeof(reply))) {
                r->reply_failed++;
                if(r->control.state>=P85_ENABLING&&r->control.state<=P85_STOPPING)
                    p85_trip(&r->control,P85_E_HOST_TIMEOUT,now);
            }
        }
    }
    if(p85_tick(&r->control,now,tx)) {
        bool accepted=r->port.can_send(r->port.context,P85_CAN_HOST_ID,true,true,tx,sizeof(tx));
        p85_tx_result(&r->control,accepted,now);
    }
}
