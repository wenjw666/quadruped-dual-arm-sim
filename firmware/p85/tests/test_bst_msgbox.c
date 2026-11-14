/* Tests routing/serialization using the real installed vendor headers and a
 * fake transport. Does not create a real R-core service or send hardware data.
 */
#include "p85_bst_msgbox.h"
#include <stdio.h>
#include <string.h>
static unsigned checks,failures,created,destroyed,unregistered,sent;
static int register_result;
static bool mapped=true,send_ok=true;
static rw_msg_t last_reply;
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);failures++;}}while(0)
int32_t ipc_trans_layer_stub_create_handle(uint8_t endid,uint8_t fid,uint8_t sid,uint8_t cid,uint8_t *handle) {
 CHECK(endid==0x90&&fid==0&&sid==2&&cid==0x90);*handle=11;created++;return 0;
}
int32_t ipc_trans_layer_register_method(uint8_t endid,uint8_t handle,uint8_t cmd) {
 CHECK(endid==0x90&&handle==11&&cmd==0x85);return register_result;
}
int32_t ipc_trans_layer_mmap_session(uint8_t endid,uint8_t handle,void **queue) {
 CHECK(endid==0x90&&handle==11);*queue=mapped?&mapped:NULL;return 0;
}
int32_t ipc_trans_layer_unregister_method(uint8_t endid,uint8_t handle) {
 CHECK(endid==0x90&&handle==11);unregistered++;return 0;
}
int32_t ipc_trans_layer_destroy_handle(uint8_t endid,uint8_t handle) {
 CHECK(endid==0x90&&handle==11);destroyed++;return 0;
}
int32_t ipc_trans_layer_stub_send_reply_msg(uint8_t endid,uint8_t handle,serdes_t *message) {
 CHECK(endid==0x90&&handle==11);last_reply=message->msg_pool[0];sent++;return send_ok?0:-1;
}
static uintptr_t lock(void *ctx){(void)ctx;return 0;}
static void unlock(void *ctx,uintptr_t saved){(void)ctx;(void)saved;}
static bool can_send(void *ctx,uint32_t id,bool fd,bool brs,const uint8_t *data,size_t bytes) {
 (void)ctx;(void)id;(void)fd;(void)brs;(void)data;(void)bytes;return true;
}
int main(void) {
 p85_runtime runtime;p85_config cfg;p85_port port={NULL,lock,unlock,can_send,p85_bst_msgbox_reply};
 rw_msg_t message;uint8_t request[32]={0},handle=0;void *queue=NULL;
 p85_default_config(&cfg,3);CHECK(p85_runtime_init(&runtime,&cfg,&port,0));
 CHECK(!p85_bst_msgbox_endpoint(&handle,&queue));
 CHECK(p85_bst_msgbox_open(NULL)!=0);CHECK(created==0);
 register_result=-12;CHECK(p85_bst_msgbox_open(&runtime)==-12);CHECK(destroyed==1&&unregistered==1);
 register_result=0;mapped=false;CHECK(p85_bst_msgbox_open(&runtime)!=0);CHECK(destroyed==2);
 mapped=true;CHECK(p85_bst_msgbox_open(&runtime)==0);CHECK(p85_bst_msgbox_open(&runtime)!=0);
 CHECK(p85_bst_msgbox_endpoint(&handle,&queue)&&handle==11&&queue==&mapped);
 CHECK(!p85_bst_msgbox_endpoint(NULL,&queue));
 memset(&message,0,sizeof(message));message.header.pid=0x10;message.header.cid=0x90;
 message.header.fid=0;message.header.sid=2;message.header.cmd=0x85;
 message.header.typ=MSGBX_MSG_TYPE_METHOD;message.header.len=4;message.header.is_eof=1;
 message.header.tok=0xa5;message.header.ver=3;message.header.is_nonsec=1;message.header.is_64b=1;
 memcpy(request,"P85C",4);request[4]=1;request[6]=0xa5;request[7]=0xab;
 memcpy(message.payload,request,32);
 CHECK(p85_bst_msgbox_ingest(&message,0));CHECK(sent==0);p85_runtime_step(&runtime,0);CHECK(sent==1);
 CHECK(last_reply.header.pid==0x90&&last_reply.header.cid==0x10&&last_reply.header.typ==MSGBX_MSG_TYPE_REPLY);
 CHECK(last_reply.header.tok==0xa5&&last_reply.header.ver==3&&last_reply.header.is_nonsec==1);
 CHECK(last_reply.header.is_64b==1&&last_reply.header.is_eof==1&&last_reply.header.idx==0&&last_reply.header.len==4);
 CHECK(!memcmp(last_reply.payload,"P85R",4));CHECK(((uint8_t *)last_reply.payload)[6]==0xa5);
 message.header.cmd=1;CHECK(!p85_bst_msgbox_ingest(&message,1));message.header.cmd=0x85;
 message.header.len=3;CHECK(!p85_bst_msgbox_ingest(&message,1));message.header.len=4;
 message.header.idx=1;CHECK(!p85_bst_msgbox_ingest(&message,1));message.header.idx=0;
 message.header.pid=0x11;CHECK(!p85_bst_msgbox_ingest(&message,1));
 send_ok=false;CHECK(!p85_bst_msgbox_reply(NULL,0,request,32));
 p85_bst_msgbox_close();CHECK(destroyed==3);p85_bst_msgbox_close();CHECK(destroyed==3);
 CHECK(!p85_bst_msgbox_endpoint(&handle,&queue));
 CHECK(!p85_bst_msgbox_ingest(&message,2));CHECK(!p85_bst_msgbox_reply(NULL,0,request,32));
 printf("P85 BST adapter: %u checks, %u failures\n",checks,failures);return failures?1:0;
}
