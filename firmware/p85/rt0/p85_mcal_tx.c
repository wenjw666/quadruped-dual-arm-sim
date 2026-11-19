/* Optional MCAL binding; excluded from the host build. Compile only inside the
 * reviewed R0 BSP. The required macros intentionally have no guessed defaults.
 */
#include "p85_mcal_tx.h"
#include "Can.h"
#include "Can_Cfg.h"
#ifndef P85_CAN_TX_HTH
#error "Set P85_CAN_TX_HTH to the verified CAN0 TX hardware object"
#endif
#ifndef P85_CAN_CONFIG_REVIEWED
#error "Review CAN0 configuration and bind register readback before enabling TX"
#endif
#if P85_CAN_CONFIG_REVIEWED != 1
#error "P85 CAN register configuration has not been reviewed"
#endif
bool p85_mcal_send(void *context,uint32_t id,bool fd,bool brs,const uint8_t *bytes,size_t n) {
    Can_PduType pdu;
    (void)context;
    if(!bytes||id!=P85_CAN_HOST_ID||!fd||!brs||n!=P85_TX_SIZE)return false;
    /* Reserved outside the one-entry CanIf demo table; do not invoke its
     * time-sync TxConfirmation. V1.6 retains this token in a uint16 slot. */
    pdu.id=id|CAN_FD_FRAME_U32;pdu.swPduHandle=0xffffu;pdu.length=P85_TX_SIZE;
    /* V1.6 Can_Ipw_Write -> FlexCAN_Ip_Send -> FlexCAN_SetTxMsgBuff writes
     * payload synchronously before returning. DMA variants need a retained
     * buffer. CONFIG_REVIEWED does not claim electrical timing validation. */
    pdu.sdu=(uint8_t *)(uintptr_t)bytes;
    return Can_Write(P85_CAN_TX_HTH,&pdu)==E_OK;
}
