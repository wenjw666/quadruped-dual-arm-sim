#include "p85_control.h"
#include <math.h>
#include <string.h>
#define PI_F 3.14159265358979323846f
static uint32_t age(uint32_t now, uint32_t then) { return now-then; }
static bool finite_between(float x,float lo,float hi) { return isfinite(x)&&x>=lo&&x<=hi; }
static bool active(const p85_control *c) { return c->state>=P85_ENABLING && c->state<=P85_STOPPING; }
static bool fresh(const p85_control *c,uint32_t now) {
    return c->has_feedback && age(now,c->feedback_ms)<=c->config.feedback_timeout_ms;
}
static void state(p85_control *c,p85_state s,uint32_t now) {
    c->state=s; c->state_ms=now; c->state_tx_floor=c->tx_ordinal+1u; c->settling=false;
}
void p85_trip(p85_control *c,p85_error e,uint32_t now) {
    if(c->state!=P85_FAULT) { c->fault=e; state(c,P85_FAULT,now); }
}
void p85_default_config(p85_config *cfg,uint8_t id) {
    memset(cfg,0,sizeof(*cfg)); cfg->motor_id=id;
    cfg->max_speed=0.5f; cfg->max_torque=2.0f; cfg->max_excursion=0.2f;
    cfg->max_tracking_error=0.15f; cfg->max_temperature=60;
    cfg->min_voltage=24; cfg->max_voltage=60;
    cfg->feedback_timeout_ms=100; cfg->heartbeat_timeout_ms=300; cfg->ack_timeout_ms=500;
}
bool p85_init(p85_control *c,const p85_config *cfg,uint32_t now) {
    if(!c||!cfg||cfg->motor_id<1||cfg->motor_id>6 ||
       !finite_between(cfg->kp,0,500)||!finite_between(cfg->kd,0,50)||
       !finite_between(cfg->max_speed,0.01f,5)||!finite_between(cfg->max_torque,0.1f,20)||
       !finite_between(cfg->max_excursion,0.001f,1)||!finite_between(cfg->max_tracking_error,0.005f,0.5f)||
       cfg->min_voltage<24||cfg->max_voltage>60||cfg->min_voltage>=cfg->max_voltage||
       cfg->max_temperature<20||cfg->max_temperature>85||
       cfg->feedback_timeout_ms<20||cfg->feedback_timeout_ms>200||
       cfg->heartbeat_timeout_ms<50||cfg->heartbeat_timeout_ms>1000||
       cfg->ack_timeout_ms<100||cfg->ack_timeout_ms>2000) return false;
    memset(c,0,sizeof(*c)); c->config=*cfg; c->state=P85_OFF; c->next_msgid=1;
    c->host_ms=now; c->state_ms=now; return true;
}
void p85_heartbeat(p85_control *c,uint32_t now) { c->host_ms=now; }
p85_error p85_configure(p85_control *c,float kp,float kd,float v,float tor,float window,int16_t temp) {
    if(c->state!=P85_OFF && c->state!=P85_DISABLED) return P85_E_STATE;
    if(!finite_between(kp,0.01f,500)||!finite_between(kd,0,50)||!finite_between(v,0.01f,5)||
       !finite_between(tor,0.1f,20)||!finite_between(window,0.001f,1)||temp<20||temp>85) return P85_E_VALUE;
    c->config.kp=kp;c->config.kd=kd;c->config.max_speed=v;c->config.max_torque=tor;
    c->config.max_excursion=window;c->config.max_temperature=temp;c->config.gains_set=true;
    return P85_OK;
}
p85_error p85_probe(p85_control *c,uint32_t now) {
    if(c->state!=P85_OFF && c->state!=P85_DISABLED) return P85_E_STATE;
    state(c,P85_PROBING,now); return P85_OK;
}
static p85_error healthy(p85_control *c,uint32_t now) {
    if(!fresh(c,now)) return P85_E_NO_FEEDBACK;
    if(c->feedback.status&P85_STATUS_ERRORS) return P85_E_MOTOR;
    if(c->feedback.temperature>=c->config.max_temperature) return P85_E_TEMPERATURE;
    if(c->feedback.voltage<c->config.min_voltage||c->feedback.voltage>c->config.max_voltage) return P85_E_VOLTAGE;
    if(fabsf(c->feedback.velocity)>c->config.max_speed) return P85_E_SPEED;
    if(fabsf(c->feedback.torque)>c->config.max_torque) return P85_E_TORQUE;
    return P85_OK;
}
p85_error p85_enable(p85_control *c,uint32_t now) {
    p85_error e;
    if(c->state!=P85_DISABLED) return P85_E_STATE;
    if(!c->config.gains_set) return P85_E_CONFIG;
    if((e=healthy(c,now))!=P85_OK) return e;
    if(c->feedback.status&P85_STATUS_ENABLED) return P85_E_STATE;
    if(age(now,c->host_ms)>c->config.heartbeat_timeout_ms) return P85_E_HOST_TIMEOUT;
    c->origin=c->feedback.position; c->have_origin=true;
    c->reference=c->origin; c->reference_velocity=0;
    state(c,P85_ENABLING,now);return P85_OK;
}
/* Quintic Hermite segment: prescribed start velocity, zero end velocity/acceleration. */
static void segment(float p0,float v0,float p1,float T,float t,float *p,float *v) {
    float s=t/T,d=p1-p0,w=v0*T,a3=10*d-6*w,a4=-15*d+8*w,a5=6*d-3*w;
    if(s>=1) {*p=p1;*v=0;return;}
    if(s<0)s=0;
    *p=p0+w*s+a3*s*s*s+a4*s*s*s*s+a5*s*s*s*s*s;
    *v=(w+3*a3*s*s+4*a4*s*s*s+5*a5*s*s*s*s)/T;
}
static bool within(const p85_control *c,float p) {
    return finite_between(p,-12.5f,12.5f)&&fabsf(p-c->origin)<=c->config.max_excursion;
}
p85_error p85_move(p85_control *c,float offset,float seconds,uint32_t now) {
    float goal=c->origin+offset;
    if(c->state!=P85_HOLD)return P85_E_STATE;
    if(!isfinite(offset)||!finite_between(seconds,0.2f,120)||!within(c,goal)||
       1.875f*fabsf(goal-c->reference)/seconds>c->config.max_speed)return P85_E_VALUE;
    c->start_position=c->reference;c->start_velocity=0;c->goal=goal;c->duration_s=seconds;
    state(c,P85_MOVE,now);return P85_OK;
}
p85_error p85_sine(p85_control *c,float a,float period,uint16_t cycles,float ramp,uint32_t now) {
    if(c->state!=P85_HOLD)return P85_E_STATE;
    if(!finite_between(a,0.001f,c->config.max_excursion)||!finite_between(period,0.5f,60)||
       !finite_between(ramp,0.5f,30)||cycles<1||cycles>100||
       !within(c,c->reference-a)||!within(c,c->reference+a)||
       a*(2*PI_F/period+1.875f/ramp)>c->config.max_speed) return P85_E_VALUE;
    c->start_position=c->reference;c->amplitude=a;c->period_s=period;c->cycles=cycles;
    c->ramp_s=ramp;state(c,P85_SINE,now);return P85_OK;
}
p85_error p85_stop(p85_control *c,uint32_t now) {
    float T=0.5f,goal;
    if(c->state==P85_HOLD)return P85_OK;
    if(c->state==P85_STOPPING)return P85_OK;
    if(c->state!=P85_MOVE&&c->state!=P85_SINE)return P85_E_STATE;
    goal=c->reference+0.5f*c->reference_velocity*T;
    if(!within(c,goal)) {p85_trip(c,P85_E_POSITION,now);return P85_E_POSITION;}
    c->start_position=c->reference;c->start_velocity=c->reference_velocity;
    c->goal=goal;c->duration_s=T;state(c,P85_STOPPING,now);return P85_OK;
}
p85_error p85_hold(p85_control *c,uint32_t now) { return p85_stop(c,now); }
p85_error p85_disable(p85_control *c,uint32_t now) {
    if(c->state==P85_FAULT)return P85_OK; /* Preserve the latched fault. */
    state(c,P85_DISABLING,now);return P85_OK;
}
p85_error p85_clear_fault(p85_control *c,uint32_t now) {
    p85_error e;
    if(c->state!=P85_FAULT)return P85_E_STATE;
    if((e=healthy(c,now))!=P85_OK)return e;
    if(c->feedback.status&P85_STATUS_ENABLED)return P85_E_STATE;
    c->fault=P85_OK;c->tx_streak=0;c->have_origin=false;state(c,P85_DISABLED,now);return P85_OK;
}
bool p85_receive(p85_control *c,uint32_t id,bool fd,bool ext,const uint8_t *data,size_t n,uint32_t now) {
    p85_feedback fb;unsigned i;uint32_t ordinal=0;
    if(!p85_decode_feedback(&fb,id,fd,ext,data,n,c->config.motor_id)) {c->rx_rejected++;return false;}
    for(i=0;i<P85_HISTORY;i++) {
        p85_sent *h=c->history+i;
        if(h->valid&&h->msgid==fb.msgid&&age(now,h->time_ms)<=c->config.feedback_timeout_ms) {
            ordinal=h->ordinal;h->valid=false;break;
        }
    }
    if(i==P85_HISTORY || (c->got_echo&&(int32_t)(ordinal-c->echo_ordinal)<=0)) {
        c->rx_rejected++;return false;
    }
    c->echo_ordinal=ordinal;c->got_echo=true;c->feedback=fb;c->feedback_ms=now;
    c->has_feedback=true;c->rx_ok++;
    if((int32_t)(ordinal-c->state_tx_floor)>=0) {
        if((c->state==P85_PROBING||c->state==P85_DISABLING)&&!(fb.status&P85_STATUS_ENABLED))
            state(c,P85_DISABLED,now);
        else if(c->state==P85_ENABLING&&(fb.status&P85_STATUS_ENABLED)) {
            c->reference=fb.position;c->reference_velocity=0;state(c,P85_HOLD,now);
        }
    }
    return true;
}
static void smooth(float x,float *y,float *dy) {
    if(x<=0){*y=0;*dy=0;}else if(x>=1){*y=1;*dy=0;}
    else {*y=x*x*x*(10+x*(-15+6*x));*dy=30*x*x*(1-x)*(1-x);}
}
static void reference(p85_control *c,uint32_t now) {
    float t=0.001f*(float)age(now,c->state_ms);
    if(c->state==P85_MOVE||c->state==P85_STOPPING) {
        segment(c->start_position,c->start_velocity,c->goal,c->duration_s,t,&c->reference,&c->reference_velocity);
        if(t>=c->duration_s) {
            if(fabsf(c->feedback.velocity)<0.03f&&fabsf(c->reference-c->feedback.position)<0.02f) {
                if(!c->settling){c->settled_ms=now;c->settling=true;}
                if(age(now,c->settled_ms)>=200)state(c,P85_HOLD,now);
            } else c->settling=false;
            if(t>c->duration_s+3.0f)p85_trip(c,P85_E_STOP_TIMEOUT,now);
        }
    } else if(c->state==P85_SINE) {
        float total=2*c->ramp_s+c->period_s*c->cycles;
        float env=1,denv=0,w=2*PI_F/c->period_s;
        if(t>=total) {
            c->reference=c->start_position;c->reference_velocity=0;
            c->goal=c->reference;c->start_velocity=0;c->duration_s=0.2f;
            state(c,P85_STOPPING,now);return;
        }
        if(t<c->ramp_s) {smooth(t/c->ramp_s,&env,&denv);denv/=c->ramp_s;}
        else if(t>total-c->ramp_s){smooth((total-t)/c->ramp_s,&env,&denv);denv/=-c->ramp_s;}
        c->reference=c->start_position+c->amplitude*env*sinf(w*(t-c->ramp_s));
        c->reference_velocity=c->amplitude*(denv*sinf(w*(t-c->ramp_s))+env*w*cosf(w*(t-c->ramp_s)));
    }
}
bool p85_tick(p85_control *c,uint32_t now,uint8_t out[P85_TX_SIZE]) {
    p85_error e; p85_command cmd;uint8_t special=P85_SPECIAL_DISABLE;bool run=false;
    if(c->tx_pending) {c->tx_pending=false;p85_trip(c,P85_E_TX,now);}
    if(c->got_tick&&age(now,c->last_tick_ms)>2u*P85_TICK_MS&&active(c))p85_trip(c,P85_E_TICK,now);
    c->last_tick_ms=now;c->got_tick=true;
    if(c->state==P85_OFF)return false;
    if(active(c)) {
        if(age(now,c->host_ms)>c->config.heartbeat_timeout_ms)p85_trip(c,P85_E_HOST_TIMEOUT,now);
        else if((e=healthy(c,now))!=P85_OK)p85_trip(c,e==P85_E_NO_FEEDBACK?P85_E_RX_TIMEOUT:e,now);
        else if(c->have_origin&&!within(c,c->feedback.position))p85_trip(c,P85_E_POSITION,now);
        else if(c->state!=P85_ENABLING&&!(c->feedback.status&P85_STATUS_ENABLED))p85_trip(c,P85_E_MOTOR,now);
    }
    if((c->state==P85_ENABLING||c->state==P85_DISABLING||c->state==P85_PROBING)&&
       age(now,c->state_ms)>c->config.ack_timeout_ms) p85_trip(c,P85_E_ACK_TIMEOUT,now);
    if(c->state==P85_DISABLED&&fresh(c,now)&&(c->feedback.status&P85_STATUS_ENABLED))p85_trip(c,P85_E_MOTOR,now);
    if(c->state>=P85_HOLD&&c->state<=P85_STOPPING) {
        reference(c,now);
        if(c->state!=P85_FAULT) {
            float effort=c->config.kp*(c->reference-c->feedback.position)+
                         c->config.kd*(c->reference_velocity-c->feedback.velocity);
            if(!within(c,c->reference))p85_trip(c,P85_E_POSITION,now);
            else if(fabsf(c->reference_velocity)>c->config.max_speed+0.0001f)p85_trip(c,P85_E_SPEED,now);
            else if(fabsf(c->reference-c->feedback.position)>c->config.max_tracking_error)p85_trip(c,P85_E_TRACKING,now);
            else if(fabsf(effort)>c->config.max_torque)p85_trip(c,P85_E_TORQUE,now);
            else run=true;
        }
    }
    if(c->state==P85_ENABLING)special=P85_SPECIAL_ENABLE;
    c->pending_msgid=c->next_msgid++;
    if(run) {
        cmd.position=c->reference;cmd.velocity=c->reference_velocity;
        cmd.kp=c->config.kp;cmd.kd=c->config.kd;cmd.torque=0;
        if(!p85_encode_run(out,c->pending_msgid,c->config.motor_id,&cmd)) {p85_trip(c,P85_E_VALUE,now);return false;}
    } else if(!p85_encode_special(out,c->pending_msgid,c->config.motor_id,special))return false;
    c->tx_pending=true;return true;
}
void p85_tx_result(p85_control *c,bool ok,uint32_t now) {
    p85_sent *h;
    if(!c->tx_pending)return;
    c->tx_pending=false;
    if(!ok) {c->tx_failed++;if(++c->tx_streak>=3)p85_trip(c,P85_E_TX,now);return;}
    c->tx_streak=0;c->tx_ok++;c->tx_ordinal++;
    h=c->history+(c->tx_ordinal%P85_HISTORY);h->msgid=c->pending_msgid;
    h->time_ms=now;h->ordinal=c->tx_ordinal;h->valid=true;
}
const char *p85_state_name(p85_state s) {
    static const char *const names[]={"off","probing","disabled","enabling","hold","move","sine","stopping","disabling","fault"};
    return (unsigned)s<=P85_FAULT?names[(unsigned)s]:"invalid";
}
const char *p85_error_name(p85_error e) {
    static const char *const names[]={"ok","state","value","no_feedback","gains_not_set","motor_fault",
    "feedback_timeout","host_timeout","ack_timeout","tracking_error","overspeed","temperature","voltage",
    "can_tx","tick_overrun","sequence","wire","position_limit","torque_limit","stop_timeout","queue_overflow"};
    return (unsigned)e<=P85_E_QUEUE?names[(unsigned)e]:"invalid";
}
