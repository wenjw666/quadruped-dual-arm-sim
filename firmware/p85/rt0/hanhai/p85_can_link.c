/* CAN0 <-> CAN1 external physical test. R0 owns both only in this build.
 * Test IDs 0x5a5/0x5a6 carry patterns, never motor commands.
 * Both controllers MUST be normal, self reception disabled, TDC enabled.
 * Startup runs before IPC/motor control; failure prevents application startup. */
#include "p85_can_link.h"
#include "p85_report.h"
#include "p85_can_trace.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Can.h"
#include "SchM_Can.h"
#include "Can_Cfg.h"
#include "FlexCAN_Ip_DeviceReg.h"
#include "FlexCAN_Ip_HwAccess.h"
#include <stdio.h>
#include <string.h>
#define REPEATS 32u
#define WAIT_MS 50u
#define FD_CONFIG_MASK (~(FLEXCAN_FDCTRL_TDCVAL_MASK | FLEXCAN_FDCTRL_TDCFAIL_MASK))
static const uint8_t lengths[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
static const Can_HwHandleType tx_object[2] = {12u,67u};
static const uint32_t ids[2] = {0x5a5u,0x5a6u};
static FLEXCAN_Type * const hw[2] = {
    (FLEXCAN_Type *)FLEXCAN_0_BASE, (FLEXCAN_Type *)FLEXCAN_1_BASE
};
volatile p85_can_link_report_type p85_can_link_report __attribute__((aligned(32)));
static volatile bool active, pending, got_rx, got_tx;
static volatile uint32_t direction, length_index;
static uint8_t expected[64];
static size_t expected_length;
static uint32_t ms(void) { return xTaskGetTickCount() * portTICK_PERIOD_MS; }
static void snapshot(uint32_t controller, volatile uint32_t *p) {
    const FLEXCAN_Type *c=hw[controller];
    p[0]=c->MCR; p[1]=c->CTRL1; p[2]=c->CBT; p[3]=c->FDCBT;
    p[4]=c->FDCTRL; p[5]=c->CTRL2; p[6]=c->ESR1; p[7]=c->ECR;
}
static bool configured(uint32_t controller) {
    const FLEXCAN_Type *c=hw[controller];
    return !(c->CTRL1 & (FLEXCAN_CTRL1_LPB_MASK|FLEXCAN_CTRL1_LOM_MASK)) &&
        (c->MCR & FLEXCAN_MCR_SRXDIS_MASK) && (c->MCR & FLEXCAN_MCR_FDEN_MASK) &&
        c->CBT==0x80139a73u && c->FDCBT==0x00031c63u &&
        !(c->CTRL2 & FLEXCAN_CTRL2_BTE_MASK) &&
        (c->FDCTRL & (FLEXCAN_FDCTRL_FDRATE_MASK|FLEXCAN_FDCTRL_TDCEN_MASK|
                     FLEXCAN_FDCTRL_TDCOFF_MASK)) ==
        (FLEXCAN_FDCTRL_FDRATE_MASK|FLEXCAN_FDCTRL_TDCEN_MASK|FLEXCAN_FDCTRL_TDCOFF(12u));
}
bool p85_can_link_rx(uint32_t controller, uint32_t id, uint32_t cs,
                     const uint8_t *data, size_t n) {
    if (!active || controller>1u) return false;
    if (!pending || got_rx) {p85_can_link_report.unexpected++;return true;}
    if (controller!=(direction^1u) || id!=(ids[direction]|CAN_FD_FRAME_U32) ||
        n!=expected_length || (cs & (0xc0000000u|FLEXCAN_IP_CS_IDE_MASK|
        FLEXCAN_IP_CS_RTR_MASK))!=0xc0000000u) {
        p85_can_link_report.bad_metadata++;return true;
    }
    if (!data || memcmp(data,expected,n)) {p85_can_link_report.bad_payload++;return true;}
    p85_can_link_report.direction[direction].received++;
    p85_can_link_report.direction[direction].length_counts[length_index]++;
    got_rx=true;return true;
}
void p85_can_link_tx_done(uint32_t token) {
    if (!active) return;
    if (!pending || got_tx || token!=(direction ? P85_LINK_TOKEN1:P85_LINK_TOKEN0)) {
        p85_can_link_report.unexpected++;return;
    }
    p85_can_link_report.direction[direction].confirmed++;got_tx=true;
}
void p85_can_link_fault(uint32_t controller, uint32_t status) {
    if (!active || controller>1u) return;
    if (!p85_can_link_report.faults[controller]) {
        snapshot(controller,p85_can_link_report.tested[controller]);
        p85_can_link_report.tested[controller][6] |= status;
    }
    p85_can_link_report.faults[controller]++;
}
bool p85_can_link_run(void) {
    volatile p85_can_link_report_type *r=&p85_can_link_report;
    uint32_t start=ms(),seq=0u;
    bool started=false;
    r->magic=0x504c4b31u;r->version=1u;r->state=1u;r->requested_per_direction=512u;
    p85_report_publish(r, sizeof *r);
    /* CAN0 is already started by board startup; Can_Init left CAN1 stopped. */
    {
        Can_ControllerStateType mode=CAN_CS_UNINIT;
        Std_ReturnType mode_status=Can_GetControllerMode(1u,&mode);
        Std_ReturnType start_status;
        p85_can_trace(1u,P85_TRACE_START_ENTER,(uint32_t)mode_status,(uint32_t)mode);
        start_status=Can_SetControllerMode(1u,CAN_CS_STARTED);
        p85_can_trace(1u,P85_TRACE_START_RETURN,(uint32_t)start_status,0u);
        if (start_status!=E_OK) {r->failure=1u;goto restore;}
    }
    started=true;
    for (uint32_t c=0;c<2u;c++) {
        snapshot(c,r->before[c]);
        if (!configured(c)) {r->failure=2u;goto restore;}
    }
    active=true;
    for (direction=0u;direction<2u;direction++) {
        r->phase=direction+1u;
        p85_report_publish(r, sizeof *r);
        for (length_index=0u;length_index<16u;length_index++) {
            for (uint32_t rep=0u;rep<REPEATS;rep++,seq++) {
                Can_PduType pdu;
                expected_length=lengths[length_index];
                for (size_t j=0u;j<sizeof expected;j++)
                    expected[j]=(uint8_t)((seq*37u)^(j*19u)^(seq>>8u)^(direction*0x69u)^0xa5u);
                if (expected_length>=4u) {
                    expected[0]=(uint8_t)seq;expected[1]=(uint8_t)(seq>>8u);
                    expected[2]=lengths[length_index];expected[3]=(uint8_t)~seq;
                }
                got_rx=false;got_tx=false;pending=true;
                pdu.id=ids[direction]|CAN_FD_FRAME_U32;
                pdu.swPduHandle=direction ? P85_LINK_TOKEN1:P85_LINK_TOKEN0;
                pdu.length=lengths[length_index];pdu.sdu=expected;
                r->direction[direction].submitted++;
                if (Can_Write(tx_object[direction],&pdu)!=E_OK) {r->failure=3u;goto restore;}
                uint32_t sent=ms();
                while (!got_rx || !got_tx) {
                    Can_MainFunction_BusOff();Can_MainFunction_Mode();
                    for (uint32_t c=0u;c<2u;c++) {
                        Can_ErrorStateType error;
                        if (Can_GetControllerErrorState((uint8_t)c,&error)!=E_OK ||
                            error!=CAN_ERRORSTATE_ACTIVE) {
                            if (!r->faults[c]) p85_can_link_fault(c,0u);
                        }
                    }
                    if (r->faults[0] || r->faults[1] || r->bad_metadata ||
                        r->bad_payload || r->unexpected) {r->failure=4u;goto restore;}
                    if ((uint32_t)(ms()-sent)>=WAIT_MS) {r->timeouts++;r->failure=5u;goto restore;}
                    vTaskDelay(pdMS_TO_TICKS(1u));
                }
                pending=false;
                vTaskDelay(pdMS_TO_TICKS(1u));
                if (r->unexpected || r->faults[0] || r->faults[1]) {r->failure=4u;goto restore;}
            }
        }
    }
restore:
    pending=false;
    if (started) {
        for (uint32_t c=0u;c<2u;c++)
            if (!r->faults[c]) snapshot(c,r->tested[c]);
    }
    /* Keep hooks active until normal-mode reinitialization is complete. */
    if (Can_SetControllerMode(1u,CAN_CS_STOPPED)==E_OK &&
        Can_SetControllerMode(0u,CAN_CS_STOPPED)==E_OK &&
        Can_SetControllerMode(0u,CAN_CS_STARTED)==E_OK) {
        Can_ControllerStateType mode;
        snapshot(0u,r->after[0]);snapshot(1u,r->after[1]);
        if (configured(0u) && configured(1u) &&
            Can_GetControllerMode(1u,&mode)==E_OK && mode==CAN_CS_STOPPED &&
            (r->after[0][4]&FD_CONFIG_MASK)==(r->before[0][4]&FD_CONFIG_MASK) &&
            (r->after[1][4]&FD_CONFIG_MASK)==(r->before[1][4]&FD_CONFIG_MASK)) r->restored=1u;
    }
    active=false;
    if (!r->restored && !r->failure) r->failure=6u;
    r->elapsed_ms=ms()-start;
    bool counts=true;
    for (uint32_t d=0u;d<2u;d++) {
        if (r->direction[d].submitted!=512u || r->direction[d].confirmed!=512u ||
            r->direction[d].received!=512u) counts=false;
        for (uint32_t l=0u;l<16u;l++)
            if (r->direction[d].length_counts[l]!=REPEATS) counts=false;
    }
    r->state=(!r->failure && counts && !r->faults[0] && !r->faults[1] &&
        !r->timeouts && !r->bad_metadata && !r->bad_payload && !r->unexpected && r->restored) ? 2u:3u;
    p85_can_trace_finish(r->failure);
    p85_report_publish(r, sizeof *r);
    printf("P85 CAN0/CAN1 physical state=%lu failure=%lu phase=%lu tx=%lu/%lu rx=%lu/%lu restored=%lu ms=%lu\n",
        (unsigned long)r->state,(unsigned long)r->failure,(unsigned long)r->phase,
        (unsigned long)r->direction[0].confirmed,(unsigned long)r->direction[1].confirmed,
        (unsigned long)r->direction[0].received,(unsigned long)r->direction[1].received,
        (unsigned long)r->restored,(unsigned long)r->elapsed_ms);
    return r->state==2u;
}
