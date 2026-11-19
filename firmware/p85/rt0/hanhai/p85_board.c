/* C1200 public Hanhai V1.6, realtime core0. This is a FreeRTOS BSP port,
 * not a Linux CAN service. Hardware acceptance is a separate validation step. */
#include "p85_board.h"
#include "p85_can_loopback.h"
#include "p85_can_link.h"
#ifndef P85_BOOT_CAN_LINK
#define P85_BOOT_CAN_LINK 0
#endif
#ifndef P85_BOOT_LOOPBACK
#define P85_BOOT_LOOPBACK 0
#endif
#include "p85_runtime.h"
#include "p85_mcal_tx.h"
#include "p85_bst_msgbox.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Can.h"
#include "SchM_Can.h"
#include "FlexCAN_Ip_DeviceReg.h"
#include "shell.h"
#include <stdio.h>

#define CONTROL_STACK_WORDS 1536u
#define RECEIVE_STACK_WORDS 1024u
static p85_runtime runtime;
static StaticTask_t control_tcb, receive_tcb;
static StackType_t control_stack[CONTROL_STACK_WORDS];
static StackType_t receive_stack[RECEIVE_STACK_WORDS];
static TaskHandle_t control_task, receive_task;
static volatile bool ready;
static volatile int startup_status = -1;
static uint32_t ignored_rx, transport_errors;

static uintptr_t irq_lock(void *context) {
    uint32_t saved;
    (void)context;
    /* GIC priority masking alone does not exclude higher priority CAN IRQs.
     * Save/restore I, including when called from an already masked ISR. */
    __asm__ volatile ("mrs %0, cpsr\n\tcpsid i\n\tdmb sy"
                      : "=r"(saved) : : "memory", "cc");
    return saved;
}
static void irq_unlock(void *context, uintptr_t saved) {
    (void)context;
    __asm__ volatile ("dmb sy" : : : "memory");
    if (!(saved & 0x80u)) __asm__ volatile ("cpsie i" : : : "memory", "cc");
}
static uint32_t now_ms(void) {
    return (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/* Compare the registers written by BOTH generated configuration layers.
 * This checks programming, not physical clock accuracy/transceiver delay.
 * CBT: 1+39+20+20=80 tq, FDCBT: 1+7+4+4=16 tq; 80 MHz, prescaler 1.
 * FPROPSEG is a direct count; other segment fields encode count minus one. */
static bool can_registers_match(void) {
    const FLEXCAN_Type *can = (const FLEXCAN_Type *)FLEXCAN_0_BASE;
    const uint32_t nominal = FLEXCAN_CBT_BTF_MASK | FLEXCAN_CBT_EPROPSEG(38u) |
        FLEXCAN_CBT_EPSEG1(19u) | FLEXCAN_CBT_EPSEG2(19u) |
        FLEXCAN_CBT_EPRESDIV(0u) | FLEXCAN_CBT_ERJW(19u);
    const uint32_t data = FLEXCAN_FDCBT_FPROPSEG(7u) |
        FLEXCAN_FDCBT_FPSEG1(3u) | FLEXCAN_FDCBT_FPSEG2(3u) |
        FLEXCAN_FDCBT_FPRESDIV(0u) | FLEXCAN_FDCBT_FRJW(3u);
    const uint32_t tdc_mask = FLEXCAN_FDCTRL_TDCEN_MASK | FLEXCAN_FDCTRL_TDCOFF_MASK;
    const uint32_t divider = *(volatile const uint32_t *)0xd000002cu;
    return (divider & 0x7cu) == (5u << 2) &&
        can->CBT == nominal && can->FDCBT == data &&
        !(can->CTRL2 & FLEXCAN_CTRL2_BTE_MASK) &&
        (can->MCR & FLEXCAN_MCR_FDEN_MASK) &&
        (can->FDCTRL & FLEXCAN_FDCTRL_FDRATE_MASK) &&
        (can->FDCTRL & tdc_mask) == (FLEXCAN_FDCTRL_TDCEN_MASK |
                                    FLEXCAN_FDCTRL_TDCOFF(12u));
}
static void receive_main(void *argument) {
    (void)argument;
    for (;;) {
        if (p85_hanhai_receive_once(now_ms) < 0) {
            transport_errors++;
            p85_runtime_bus_fault(&runtime);
            vTaskDelay(pdMS_TO_TICKS(10u));
        }
    }
}
static void control_main(void *argument) {
    p85_config config;
    p85_port port = {NULL, irq_lock, irq_unlock, p85_mcal_send, p85_bst_msgbox_reply};
    TickType_t wake;
    (void)argument;
    /* The control core uses float/trig. Opt into the R5 FPU context save. */
    portTASK_USES_FLOATING_POINT();
    p85_default_config(&config, 3u);
    if (!p85_runtime_init(&runtime, &config, &port, now_ms())) startup_status = -2;
    else if (!can_registers_match()) startup_status = -3;
    else if (p85_bst_msgbox_open(&runtime) != 0) startup_status = -4;
    else if (Can_SetControllerMode(0u, CAN_CS_STARTED) != E_OK) {
        startup_status = -6;
        p85_bst_msgbox_close();
    } else if (P85_BOOT_LOOPBACK && !p85_can_loopback_run()) {
        startup_status = -7;
        (void)Can_SetControllerMode(0u, CAN_CS_STOPPED);
        p85_bst_msgbox_close();
    } else if (P85_BOOT_CAN_LINK && !p85_can_link_run()) {
        startup_status = -8;
        (void)Can_SetControllerMode(0u, CAN_CS_STOPPED);
        (void)Can_SetControllerMode(1u, CAN_CS_STOPPED);
        p85_bst_msgbox_close();
    } else {
        receive_task = xTaskCreateStatic(receive_main, "P85_IPC", RECEIVE_STACK_WORDS,
                                        NULL, 4u, receive_stack, &receive_tcb);
        if (!receive_task) {
            startup_status = -5;
            (void)Can_SetControllerMode(0u, CAN_CS_STOPPED);
            p85_bst_msgbox_close();
        } else {
            ready = true;
            startup_status = 0;
        }
    }
    printf("P85 R0 startup=%d, motor=3, CAN0 1M/5M, startup OFF\n", startup_status);
    wake = xTaskGetTickCount();
    for (;;) {
        if (startup_status == 0) {
            Can_ErrorStateType errors;
            /* CAN0 bus-off is configured for polling in this BSP. */
            Can_MainFunction_BusOff();
            Can_MainFunction_Mode();
            if (Can_GetControllerErrorState(0u, &errors) != E_OK ||
                errors != CAN_ERRORSTATE_ACTIVE) p85_runtime_bus_fault(&runtime);
            p85_runtime_step(&runtime, now_ms());
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(P85_TICK_MS));
    }
}
void p85_board_create_tasks(void) {
    configASSERT(configTICK_RATE_HZ == 1000u);
    control_task = xTaskCreateStatic(control_main, "P85_CTRL", CONTROL_STACK_WORDS,
                                    NULL, 5u, control_stack, &control_tcb);
    configASSERT(control_task != NULL);
}
void p85_board_rx(uint32_t id, const uint8_t *data, size_t length) {
    if (!ready) return;
    if ((id & 0x1fffffffu) != 3u || !(id & 0x40000000u) ||
        (id & 0x80000000u) || length != P85_RX_SIZE || !data) {
        ignored_rx++;
        return;
    }
    /* CAN ISR priority 150 in inter_cfg/isr.cfg permits FromISR tick reads. */
    (void)p85_runtime_rx(&runtime, id & 0x1fffffffu,
                        (id & 0x40000000u) != 0u, (id & 0x80000000u) != 0u,
                        data, length, (uint32_t)xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
}
void p85_board_bus_fault(void) {
    p85_can_loopback_fault();
    if (ready) p85_runtime_bus_fault(&runtime);
}
void p85_board_status(void) {
    uintptr_t saved = irq_lock(NULL);
    p85_state state = runtime.control.state;
    p85_error fault = runtime.control.fault;
    uint32_t tx = runtime.control.tx_ok, failed = runtime.control.tx_failed;
    uint32_t rx = runtime.control.rx_ok, ignored = ignored_rx;
    uint32_t ipc = transport_errors, stale = runtime.stale_commands;
    uint32_t replies = runtime.reply_failed;
    irq_unlock(NULL, saved);
    printf("P85 init=%d state=%s fault=%s tx=%lu tx_fail=%lu rx=%lu ignored=%lu "
           "ipc_err=%lu stale=%lu reply_fail=%lu stack_words=%lu/%lu\n",
           startup_status, p85_state_name(state), p85_error_name(fault),
           (unsigned long)tx, (unsigned long)failed, (unsigned long)rx,
           (unsigned long)ignored, (unsigned long)ipc, (unsigned long)stale,
           (unsigned long)replies,
           control_task ? (unsigned long)uxTaskGetStackHighWaterMark(control_task) : 0ul,
           receive_task ? (unsigned long)uxTaskGetStackHighWaterMark(receive_task) : 0ul);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
                 p85stat, p85_board_status, P85 read only status);
