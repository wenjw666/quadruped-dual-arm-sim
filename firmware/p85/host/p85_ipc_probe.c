#include <stdio.h>
int p85_ipc_open(void);
int p85_ipc_online(void);
void p85_ipc_close(void);
int main(void) {
 int result=p85_ipc_open();
 if(result!=0){printf("MsgBox create/map failed: %d\n",result);return 1;}
 result=p85_ipc_online();
 printf("MsgBox CPU_0 -> REALTIME_0 fid=0 sid=2: query=%d (%s); no payload sent\n",
 result,result==2?"R core online; P85 protocol unverified":"core/transport unavailable");
 p85_ipc_close();return result==2?0:3;
}
