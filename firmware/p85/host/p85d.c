/* Linux-only software plant. This process never opens CAN or MsgBox devices. */
#define _POSIX_C_SOURCE 200809L
#include "p85_wire.h"
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>
static volatile sig_atomic_t running=1;
static void stop_handler(int signum) { (void)signum;running=0; }
static uint64_t ms(void) {
 struct timespec ts;clock_gettime(CLOCK_MONOTONIC,&ts);
 return (uint64_t)ts.tv_sec*1000u+(uint64_t)ts.tv_nsec/1000000u;
}
static uint16_t pack(float x,float low,float high) {
 double v=((double)x-low)*65535.0/(high-low);
 if(v<0)v=0;
 if(v>65535)v=65535;
 return (uint16_t)floor(v+0.5);
}
static float unpack(const uint8_t *p,float low,float high) {
 uint16_t u=p85_get_u16(p);if(u==0x8000u&&low==-high)return 0;
 return low+(float)u*(high-low)/65535.0f;
}
int main(int argc,char **argv) {
 struct sockaddr_un address,peer;int fd=-1,mode=0;uint64_t epoch,next,drop_after=UINT64_MAX;
 p85_config cfg;p85_control c;p85_session session={false,0};
 uint8_t request[64],response[32],tx[64],fb[12];
 bool enabled=false;float position=0,velocity=0,torque=0;const char *socket_path=NULL;
 for(int i=1;i<argc;i++) {
  if(!strcmp(argv[i],"--simulate"))mode=1;
  else if(!strcmp(argv[i],"--no-motor"))mode=2;
  else if(!strcmp(argv[i],"--socket")&&i+1<argc)socket_path=argv[++i];
  else if(!strcmp(argv[i],"--drop-feedback-after-ms")&&i+1<argc)drop_after=strtoull(argv[++i],NULL,10);
  else {fprintf(stderr,"usage: p85d (--simulate | --no-motor) --socket PATH [--drop-feedback-after-ms N]\n");return 2;}
 }
 if(!mode||!socket_path||strlen(socket_path)>=sizeof(address.sun_path))return 2;
 p85_default_config(&cfg,3);if(!p85_init(&c,&cfg,0))return 2;
 fd=socket(AF_UNIX,SOCK_DGRAM,0);if(fd<0){perror("socket");return 1;}
 memset(&address,0,sizeof(address));address.sun_family=AF_UNIX;strcpy(address.sun_path,socket_path);
 umask(0077);
 if(bind(fd,(struct sockaddr*)&address,sizeof(address))<0){perror("bind (existing socket is not removed)");close(fd);return 1;}
 signal(SIGTERM,stop_handler);signal(SIGINT,stop_handler);
 printf("P85 %s only; no hardware IO; socket=%s\n",mode==1?"SIMULATOR":"NO-MOTOR",socket_path);fflush(stdout);
 epoch=ms();next=epoch;
 while(running) {
  uint64_t now=ms();socklen_t peer_length=sizeof(peer);
  ssize_t n=recvfrom(fd,request,sizeof(request),MSG_DONTWAIT,(struct sockaddr*)&peer,&peer_length);
  if(n>=0) {
   p85_dispatch(&c,&session,request,(size_t)n,(uint32_t)(now-epoch),response);
   if(sendto(fd,response,sizeof(response),MSG_DONTWAIT,(struct sockaddr*)&peer,peer_length)<0&&errno!=ENOENT)perror("reply");
  } else if(errno!=EAGAIN&&errno!=EINTR){perror("receive");break;}
  if(now>=next) {
   uint32_t t=(uint32_t)(now-epoch);next=now+P85_TICK_MS;
   if(p85_tick(&c,t,tx)) {
    p85_tx_result(&c,true,t);
    if(mode==1&&now-epoch<drop_after) {
     const uint8_t *slot=tx+22;
     bool special=true;for(unsigned k=0;k<9;k++)if(slot[k]!=0xff)special=false;
     if(special) {if(slot[9]==P85_SPECIAL_DISABLE)enabled=false;else if(slot[9]==P85_SPECIAL_ENABLE)enabled=true;velocity=0;torque=0;}
     else if(enabled) {
      float target=unpack(slot,-12.5f,12.5f),vref=unpack(slot+2,-65,65);
      float kp=unpack(slot+4,0,500),kd=unpack(slot+6,0,50);
      float effort=kp*(target-position)+kd*(vref-velocity)+unpack(slot+8,-200,200);
      /* Ideal position servo is intentional: validates software, not motor dynamics. */
      velocity=vref;position=target;torque=effort;
     }
     memset(fb,0,sizeof(fb));memcpy(fb,tx,2);
     p85_put_u16(fb+2,pack(position,-12.5f,12.5f));p85_put_u16(fb+4,pack(velocity,-65,65));
     p85_put_u16(fb+6,pack(torque,-200,200));fb[8]=65;fb[9]=enabled?1:0;fb[10]=48;
     p85_receive(&c,3,true,false,fb,sizeof(fb),t);
    }
   }
  }
  {struct timespec wait_time={0,1000000};nanosleep(&wait_time,NULL);}
 }
 close(fd);unlink(socket_path);return 0;
}
