/* Built in the public Hanhai V1.6 R0 firmware and checked with the board's
 * installed 4.4.7 Linux headers/fake transport. Board execution is unverified.
 */
#include "p85_bst_msgbox.h"
#include <string.h>
#define RT0 0x90u
#define CPU0 0x10u
#define FID 0u
#define SID 2u
static p85_runtime *owner;
static uint8_t service_handle;
static bool opened;
static void *vendor_queue;
static serdes_t serializer; /* Large vendor object stays out of the task stack. */
int p85_bst_msgbox_open(p85_runtime *runtime) {
    int result;
    if(!runtime||opened)return -1;
    result=ipc_trans_layer_stub_create_handle(RT0,FID,SID,RT0,&service_handle);
    if(result!=0)return result;
    result=ipc_trans_layer_register_method(RT0,service_handle,P85_MSGBOX_CMD);
    if(result==0)result=ipc_trans_layer_mmap_session(RT0,service_handle,&vendor_queue);
    if(result!=0||!vendor_queue) {
        ipc_trans_layer_unregister_method(RT0,service_handle);
        ipc_trans_layer_destroy_handle(RT0,service_handle);return result?result:-1;
    }
    owner=runtime;opened=true;return 0;
}
bool p85_bst_msgbox_endpoint(uint8_t *handle,void **queue) {
    if(!opened||!handle||!queue)return false;
    *handle=service_handle;*queue=vendor_queue;return true;
}
void p85_bst_msgbox_close(void) {
    /* Only after both producer and owner tasks have stopped and CAN disabled. */
    if(opened) {
        ipc_trans_layer_unregister_method(RT0,service_handle);
        ipc_trans_layer_destroy_handle(RT0,service_handle);
        opened=false;owner=NULL;vendor_queue=NULL;
    }
}
bool p85_bst_msgbox_ingest(const rw_msg_t *m,uint32_t now) {
    uint32_t cookie;
    if(!opened||!m||m->header.pid!=CPU0||m->header.cid!=RT0||
       m->header.fid!=FID||m->header.sid!=SID||m->header.cmd!=P85_MSGBOX_CMD||
       m->header.typ!=MSGBX_MSG_TYPE_METHOD||m->header.len!=4||
       !m->header.is_eof||m->header.idx!=0)return false;
    /* Preserve the transport correlation/security bits in the queued route. */
    cookie=m->header.tok|(m->header.ver<<8)|(m->header.is_nonsec<<12)|(m->header.is_64b<<13);
    return p85_runtime_request(owner,(const uint8_t *)m->payload,P85_WIRE_SIZE,cookie,now);
}
bool p85_bst_msgbox_reply(void *context,uint32_t cookie,const uint8_t *data,size_t n) {
    (void)context;
    if(!opened||!data||n!=P85_WIRE_SIZE)return false;
    memset(&serializer,0,sizeof(serializer));ipc_ser_init(&serializer);
    serializer.header.pid=RT0;serializer.header.cid=CPU0;
    serializer.header.fid=FID;serializer.header.sid=SID;serializer.header.cmd=P85_MSGBOX_CMD;
    serializer.header.typ=MSGBX_MSG_TYPE_REPLY;serializer.header.tok=cookie&255u;
    serializer.header.ver=(cookie>>8)&15u;serializer.header.is_nonsec=(cookie>>12)&1u;
    serializer.header.is_64b=(cookie>>13)&1u;
    memcpy(serializer.msg_pool[0].payload,data,n);serializer.pos=(uint32_t)n;
    if(ipc_ser_finish(&serializer)!=0)return false;
    return ipc_trans_layer_stub_send_reply_msg(RT0,service_handle,&serializer)==0;
}
