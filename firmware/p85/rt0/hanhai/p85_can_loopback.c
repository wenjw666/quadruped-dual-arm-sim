/* Startup-only CAN0 selftest, executed by its sole owner before IPC receive processing starts.
 * No motor command is constructed. MCAL stays responsible for TX/RX and IRQs.
 * Freeze-mode APIs below are vendor APIs, never called from the A core. */
#include "p85_can_loopback.h"
#include "p85_report.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Can.h"
#include "SchM_Can.h"
#include "Can_Cfg.h"
#include "FlexCAN_Ip.h"
#include "FlexCAN_Ip_HwAccess.h"
#include <stdio.h>
#include <string.h>
#define TEST_ID 0x5a5u
#define REPEATS 32u
#define WAIT_MS 50u
#define CS_EDL 0x80000000u
#define CS_BRS 0x40000000u
#define FD_CONFIG_MASK (~(FLEXCAN_FDCTRL_TDCVAL_MASK | FLEXCAN_FDCTRL_TDCFAIL_MASK))
static const uint8_t lengths[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
volatile p85_can_loopback_report_type p85_can_loopback_report __attribute__((aligned(32)));
static volatile bool active, pending, got_rx, got_tx;
static uint8_t expected[64];
static size_t expected_length;
static uint32_t length_index;
static uint32_t ms(void) { return xTaskGetTickCount() * portTICK_PERIOD_MS; }
static void snapshot(volatile uint32_t *p) {
    const FLEXCAN_Type *c = (const FLEXCAN_Type *)FLEXCAN_0_BASE;
    p[0]=c->MCR; p[1]=c->CTRL1; p[2]=c->CBT; p[3]=c->FDCBT;
    p[4]=c->FDCTRL; p[5]=c->CTRL2; p[6]=c->ESR1; p[7]=c->ECR;
}
bool p85_can_loopback_rx(uint32_t id, uint32_t cs, const uint8_t *data, size_t n) {
    if (!active) return false;
    if (!pending || got_rx) { p85_can_loopback_report.unexpected++; return true; }
    if (id != (TEST_ID | CAN_FD_FRAME_U32) || n != expected_length ||
        (cs & (CS_EDL | CS_BRS | FLEXCAN_IP_CS_IDE_MASK | FLEXCAN_IP_CS_RTR_MASK)) !=
        (CS_EDL | CS_BRS)) { p85_can_loopback_report.bad_metadata++; return true; }
    if (!data || memcmp(data, expected, n)) {
        p85_can_loopback_report.bad_payload++; return true;
    }
    p85_can_loopback_report.received++;
    p85_can_loopback_report.length_counts[length_index]++;
    got_rx = true;
    return true;
}
void p85_can_loopback_tx_done(void) {
    if (active && pending && !got_tx) {
        p85_can_loopback_report.confirmed++;
        got_tx = true;
    } else if (active) p85_can_loopback_report.unexpected++;
}
void p85_can_loopback_fault(void) {
    if (active) {
        /* Preserve error-time status before STOP/START clears the controller. */
        if (!p85_can_loopback_report.faults) {
            FLEXCAN_Type *c = (FLEXCAN_Type *)FLEXCAN_0_BASE;
            p85_can_loopback_report.loopback[6] = c->ESR1;
            p85_can_loopback_report.loopback[7] = c->ECR;
        }
        p85_can_loopback_report.faults++;
    }
}
bool p85_can_loopback_run(void) {
    volatile p85_can_loopback_report_type *r = &p85_can_loopback_report;
    FLEXCAN_Type *c = (FLEXCAN_Type *)FLEXCAN_0_BASE;
    Can_ErrorStateType error;
    uint32_t start = ms(), seq = 0u;
    r->magic=0x504c4231u; r->version=1u; r->state=1u; r->requested=16u*REPEATS;
    snapshot(r->before);
    active = true;
    if (FlexCAN_Ip_EnterFreezeMode(0u) != FLEXCAN_STATUS_SUCCESS) {
        r->failure=1u; goto restore;
    }
    /* FlexCAN requires TDC disabled in internal loopback (FDCTRL[TDCEN]).
     * Only the external-transceiver compensation changes; FD+BRS and both
     * bit timings remain at the production 1 Mbit/s / 5 Mbit/s values. */
    c->FDCTRL &= ~FLEXCAN_FDCTRL_TDCEN_MASK;
    FlexCAN_SetOperationMode(c, FLEXCAN_LOOPBACK_MODE);
    if (FlexCAN_Ip_ExitFreezeMode(0u) != FLEXCAN_STATUS_SUCCESS) {
        r->failure=2u; goto restore;
    }
    snapshot(r->loopback);
    if (!(c->CTRL1 & FLEXCAN_CTRL1_LPB_MASK) || (c->MCR & FLEXCAN_MCR_SRXDIS_MASK) ||
        (c->CTRL1 & FLEXCAN_CTRL1_LOM_MASK) || c->CBT != r->before[2] ||
        c->FDCBT != r->before[3] || (c->FDCTRL & FD_CONFIG_MASK) !=
        (r->before[4] & FD_CONFIG_MASK & ~FLEXCAN_FDCTRL_TDCEN_MASK)) {
        r->failure=3u; goto restore;
    }
    for (length_index=0u; length_index<16u; length_index++) {
        for (uint32_t rep=0u; rep<REPEATS; rep++,seq++) {
            Can_PduType pdu;
            expected_length=lengths[length_index];
            for (size_t j=0u;j<sizeof expected;j++)
                expected[j]=(uint8_t)((seq*37u) ^ (j*19u) ^ (seq>>8u) ^ 0xa5u);
            if (expected_length>=4u) {
                expected[0]=(uint8_t)seq; expected[1]=(uint8_t)(seq>>8u);
                expected[2]=lengths[length_index]; expected[3]=(uint8_t)~seq;
            }
            got_rx=false; got_tx=false; pending=true;
            pdu.id=TEST_ID | CAN_FD_FRAME_U32; pdu.swPduHandle=P85_LOOPBACK_TOKEN;
            pdu.length=lengths[length_index]; pdu.sdu=expected;
            r->submitted++;
            if (Can_Write(12u,&pdu)!=E_OK) { r->failure=4u; goto restore; }
            uint32_t sent=ms();
            while (!got_rx || !got_tx) {
                Can_MainFunction_BusOff(); Can_MainFunction_Mode();
                if (r->faults || r->bad_metadata || r->bad_payload || r->unexpected) {
                    r->failure=5u; goto restore;
                }
                if ((uint32_t)(ms()-sent)>=WAIT_MS) {
                    r->timeouts++; r->failure=6u; goto restore;
                }
                vTaskDelay(pdMS_TO_TICKS(1u));
            }
            pending=false;
            /* A quiet tick exposes duplicate callbacks before the next frame. */
            vTaskDelay(pdMS_TO_TICKS(1u));
            if (r->unexpected) {r->failure=5u;goto restore;}
        }
    }
restore:
    pending=false;
    /* STOP/START fully reinitializes the original generated NORMAL config,
     * RX filters and buffers; it also restores SRXDIS, which NORMAL alone
     * in FlexCAN_SetOperationMode does not restore. */
    if (Can_SetControllerMode(0u,CAN_CS_STOPPED)==E_OK &&
        Can_SetControllerMode(0u,CAN_CS_STARTED)==E_OK) {
        snapshot(r->after);
        if (!(c->CTRL1 & (FLEXCAN_CTRL1_LPB_MASK|FLEXCAN_CTRL1_LOM_MASK)) &&
            (c->MCR & FLEXCAN_MCR_SRXDIS_MASK) &&
            c->CBT==r->before[2] && c->FDCBT==r->before[3] &&
            (c->FDCTRL & FD_CONFIG_MASK)==(r->before[4] & FD_CONFIG_MASK) &&
            Can_GetControllerErrorState(0u,&error)==E_OK && error==CAN_ERRORSTATE_ACTIVE)
            r->restored=1u;
    }
    active=false;
    if (!r->restored && !r->failure) r->failure=7u;
    r->elapsed_ms=ms()-start;
    bool lengths_ok=true;
    for (uint32_t l=0u;l<16u;l++)
        if (r->length_counts[l]!=REPEATS) lengths_ok=false;
    r->state=(lengths_ok && !r->failure && !r->timeouts && !r->faults && !r->bad_metadata &&
        !r->bad_payload && !r->unexpected && r->submitted==r->requested &&
        r->confirmed==r->requested && r->received==r->requested && r->restored) ? 2u:3u;
    p85_report_publish(r, sizeof *r);
    printf("P85 CAN0 internal loopback state=%lu failure=%lu tx=%lu/%lu rx=%lu/%lu "
           "bad_meta=%lu bad_data=%lu unexpected=%lu timeout=%lu restored=%lu ms=%lu\n",
           (unsigned long)r->state,(unsigned long)r->failure,(unsigned long)r->confirmed,
           (unsigned long)r->submitted,(unsigned long)r->received,(unsigned long)r->requested,
           (unsigned long)r->bad_metadata,(unsigned long)r->bad_payload,
           (unsigned long)r->unexpected,(unsigned long)r->timeouts,
           (unsigned long)r->restored,(unsigned long)r->elapsed_ms);
    return r->state==2u;
}
