/* Adapter compiled against the board's official hanhai-ipc-ubuntu-dev headers.
 * Deliberately uses the public library, never duplicates kernel ioctl structs.
 */
#define _POSIX_C_SOURCE 200809L
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include "ipc_trans_layer.h"
#define P85_CPU 0x10u
#define P85_RT0 0x90u
#define P85_FID 0u
#define P85_SID 2u
#define P85_CMD 0x85u
static uint8_t handle;
static bool opened;
static int online_state;
static void *mapped_queue;
static serdes_t outgoing;
static uint64_t now_ms(void) {
 struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);
 return (uint64_t)t.tv_sec*1000u+(uint64_t)t.tv_nsec/1000000u;
}
int p85_ipc_open(void) {
 int ret;
 if(opened)return -1;
 ret=ipc_trans_layer_proxy_create_handle(P85_CPU,P85_FID,P85_SID,P85_RT0,0,&handle);
 if(ret!=0)return ret;
 ret=ipc_trans_layer_mmap_session(P85_CPU,handle,&mapped_queue);
 if(ret!=0||!mapped_queue) {ipc_trans_layer_destroy_handle(P85_CPU,handle);return ret?ret:-1;}
 online_state=ipc_trans_layer_query_info(P85_CPU,handle,0,1);
 opened=true;return 0;
}
int p85_ipc_online(void) {
 if(!opened)return -1;
 /* The user-side query reports a one-shot endpoint transition. Cache it
  * at open; repeating it after HELLO returns -1 even while RT0 remains up. */
 return online_state;
}
int p85_ipc_exchange(const uint8_t *request,uint8_t *reply,int timeout_ms) {
 rw_msg_t received;uint64_t stamp,deadline;int ret;
 if(!opened||!request||!reply||timeout_ms<1||timeout_ms>1000)return -1;
 if(online_state!=2)return -404;
 memset(&outgoing,0,sizeof(outgoing));ipc_ser_init(&outgoing);
 outgoing.header.pid=P85_CPU;outgoing.header.cid=P85_RT0;
 outgoing.header.fid=P85_FID;outgoing.header.sid=P85_SID;
 outgoing.header.cmd=P85_CMD;outgoing.header.typ=MSGBX_MSG_TYPE_METHOD;
 outgoing.header.tok=request[6];
 outgoing.header.len=4;outgoing.header.is_eof=1;
 memcpy(outgoing.msg_pool[0].payload,request,32);outgoing.pos=32;ipc_ser_finish(&outgoing);
 ret=ipc_trans_layer_proxy_send_method(P85_CPU,handle,&outgoing);
 if(ret!=0)return ret;
 deadline=now_ms()+(uint64_t)timeout_ms;
 do {
  uint64_t current=now_ms();
  int remaining;if(current>=deadline)break;remaining=(int)(deadline-current);
  memset(&received,0,sizeof(received));
  ret=ipc_trans_layer_get_rwmsg(P85_CPU,handle,remaining,&received,&stamp);
  if(ret!=0)continue;
  if(received.header.pid!=P85_RT0||received.header.cid!=P85_CPU||
     received.header.fid!=P85_FID||received.header.sid!=P85_SID||
     received.header.cmd!=P85_CMD||received.header.typ!=MSGBX_MSG_TYPE_REPLY||
     received.header.len!=4||!received.header.is_eof||received.header.idx!=0||
     received.header.tok!=outgoing.header.tok)continue;
  memcpy(reply,received.payload,32);
  if(!memcmp(reply,"P85R",4)&&reply[4]==1&&reply[6]==request[6]&&reply[7]==request[7])return 0;
 }while(now_ms()<deadline);
 return -110;
}
void p85_ipc_close(void) {
 if(opened){ipc_trans_layer_destroy_handle(P85_CPU,handle);opened=false;online_state=-1;mapped_queue=NULL;}
}
