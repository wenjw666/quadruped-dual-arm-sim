/* Optional MCAL binding; excluded from the host build. Compile only inside the
 * verified R0 BSP. The required macros intentionally have no guessed defaults.
 */
#include "p85_mcal_tx.h"
#include "Can.h"
#include "Can_Cfg.h"
#ifndef P85_CAN_TX_HTH
#error "Set P85_CAN_TX_HTH to the verified CAN0 TX hardware object"
#endif
#ifndef P85_CAN_TIMING_VERIFIED
#error "Verify 1Mbps arbitration, 5Mbps data, FD+BRS, clock, TDC and transceiver first"
#endif
#if P85_CAN_TIMING_VERIFIED != 1
#error "P85 CAN timing is not verified"
#endif
bool p85_mcal_send(void *context,uint32_t id,bool fd,bool brs,const uint8_t *bytes,size_t n) {
    Can_PduType pdu;
    (void)context;
    if(!bytes||id!=P85_CAN_HOST_ID||!fd||!brs||n!=P85_TX_SIZE)return false;
    pdu.id=id|CAN_FD_FRAME_U32;pdu.swPduHandle=0;pdu.length=P85_TX_SIZE;
    /* MCAL copies payload to the HW mailbox before returning acceptance. Check
     * this contract in the actual BSP; an asynchronous DMA port needs its own
     * retained buffer and completion handling instead of this adapter.
     */
    pdu.sdu=(uint8_t *)(uintptr_t)bytes;
    return Can_Write(P85_CAN_TX_HTH,&pdu)==E_OK;
}
