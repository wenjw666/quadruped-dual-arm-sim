#ifndef P85_BOARD_H
#define P85_BOARD_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* Called once by SDK StartupTask, after the BSP has initialized CAN/IPC. */
void p85_board_create_tasks(void);
/* CAN0 hooks consume full MCAL metadata, including bits 31 (IDE) and 30 (FD). */
void p85_board_rx(uint32_t can_id, const uint8_t *data, size_t length);
void p85_board_bus_fault(void);
void p85_board_status(void);
/* Blocks in the vendor query API, then copies completed single-frame packets.
 * Only the receive task may call it. The NEXT query recycles vendor nodes. */
int p85_hanhai_receive_once(uint32_t (*now_ms)(void));
#endif
