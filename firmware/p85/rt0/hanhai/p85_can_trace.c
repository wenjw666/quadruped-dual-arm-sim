#include "p85_can_trace.h"
#if P85_BOOT_CAN_LINK
#include "p85_report.h"
#include "FlexCAN_Ip_DeviceReg.h"
#include <stddef.h>
typedef struct {
    uint32_t stage, instance, status, aux;
    uint32_t regs[8];
    uint32_t reserved[4];
} p85_can_trace_entry;
typedef struct {
    uint32_t magic, version, count, dropped, finalized, failure, capacity, entry_words;
    p85_can_trace_entry entries[P85_CAN_TRACE_CAPACITY];
} p85_can_trace_type;
volatile p85_can_trace_type p85_can_trace_report __attribute__((aligned(32)));
_Static_assert(sizeof(p85_can_trace_type) == P85_CAN_TRACE_WORDS * 4u, "trace layout");
void p85_can_trace(uint32_t instance, uint32_t stage, uint32_t status, uint32_t aux) {
    volatile p85_can_trace_type *r = &p85_can_trace_report;
    const FLEXCAN_Type *c;
    volatile p85_can_trace_entry *e;
    if (instance > 1u || r->finalized) return;
    if (!r->magic) {
        r->magic=0x50545231u; r->version=1u;
        r->capacity=P85_CAN_TRACE_CAPACITY; r->entry_words=16u;
    }
    if (r->count >= P85_CAN_TRACE_CAPACITY) {
        r->dropped++;
        p85_report_publish(r,32u);
        return;
    }
    c=(const FLEXCAN_Type *)(instance ? FLEXCAN_1_BASE : FLEXCAN_0_BASE);
    e=&r->entries[r->count];
    e->stage=stage; e->instance=instance; e->status=status; e->aux=aux;
    e->regs[0]=c->MCR; e->regs[1]=c->CTRL1;
    e->regs[2]=c->CBT; e->regs[3]=c->FDCBT;
    e->regs[4]=c->FDCTRL; e->regs[5]=c->CTRL2;
    e->regs[6]=c->ESR1; e->regs[7]=c->ECR;
    p85_report_publish(e,sizeof *e);
    r->count++;
    p85_report_publish(r,32u);
}
void p85_can_trace_finish(uint32_t failure) {
    p85_can_trace(1u,P85_TRACE_FINISH,failure,0u);
    p85_can_trace_report.failure=failure;
    p85_can_trace_report.finalized=1u;
    p85_report_publish(&p85_can_trace_report,sizeof p85_can_trace_report);
}
#endif
