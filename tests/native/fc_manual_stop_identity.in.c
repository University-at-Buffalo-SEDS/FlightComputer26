#include "sedsnet.h"
#include "sedsnet_config.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static struct { SedsRouter *r; } g_router;
static SedsRouter *recovery_router;
static unsigned stop_calls,callbacks,floor_checks;
static SedsResult handler_result;
static char callback_sender[80];
static void fc_recovery_manual_stop(void){++stop_calls;}
static volatile uint32_t g_reliable_refusal_count;static bool fc_recovery_abort_latched(void){return false;}
static void fc_recovery_manual_stop_original_utc(uint64_t source){(void)source;fc_recovery_manual_stop();}
/* The real floor is deliberately modelled as denying ordinary commands.
 * This tests that the actual manual handler never calls that gate, not NVM. */
static uint8_t board_recovery_command_time_allowed(uint64_t timestamp){(void)timestamp;++floor_checks;return 0;}
static uint8_t board_recovery_floor_active(void){return 1;}
/* SOURCE_FUNCTION:authorized_owner */
/* SOURCE_FUNCTION:board_recovery_manual_stop_source_allowed */
/* Freshness clock: never reached here, the modelled floor refuses first. */
static __attribute__((unused)) uint64_t board_recovery_station_now(SedsRouter*r){uint64_t t=0;return seds_router_get_network_time_ms(r,&t)==SEDS_OK?t:0U;}
/* SOURCE_FUNCTION:board_recovery_command_is_stale */
/* SOURCE_FUNCTION:Manual_Abort_handler */
static uint64_t now(void*u){(void)u;return 123456;}
static SedsResult observe(const SedsPacketView*p,void*u){++callbacks;assert(p->sender_len<sizeof callback_sender);memcpy(callback_sender,p->sender,p->sender_len);callback_sender[p->sender_len]=0;handler_result=Manual_Abort_handler(p,u);printf("callback sender=%s timestamp=%llu payload=%zu result=%d stop_calls=%u floor_calls=%u\n",callback_sender,(unsigned long long)p->timestamp,p->payload_len,handler_result,stop_calls,floor_checks);return handler_result;}
static FILE*discovery_out;
static SedsResult capture(const uint8_t*b,size_t n,void*u){(void)u;if(discovery_out){assert(n<=UINT32_MAX);uint32_t len=(uint32_t)n;assert(fwrite(&len,4,1,discovery_out)==1&&fwrite(b,1,n,discovery_out)==n);}return SEDS_OK;}
static SedsResult discard(const SedsPacketView*p,void*u){(void)p;(void)u;return SEDS_OK;}
static void load_discovery(SedsRouter*r,uint32_t sid,const char*path){FILE*f=fopen(path,"rb");assert(f);uint32_t n;uint8_t bytes[65536];while(fread(&n,4,1,f)==1){assert(n<=sizeof bytes&&fread(bytes,1,n,f)==n);assert(seds_router_receive_packed_from_side(r,sid,bytes,n)==SEDS_OK);assert(seds_router_process_all_queues(r)==SEDS_OK);}fclose(f);}
static const char*payload_for(const char*kind){if(!strcmp(kind,"manual"))return "SEDS_MANUAL_ABORT_V1:";if(!strcmp(kind,"auto"))return "SEDS_AUTO_ABORT_V1:";if(!strcmp(kind,"long"))return "SEDS_MANUAL_ABORT_V1:x";if(!strcmp(kind,"short"))return "SEDS_MANUAL_ABORT_V1";return "Manual Abort Command Issued";}
int main(int argc,char**argv){
 setbuf(stdout,NULL);assert(argc==9);assert(seds_schema_register_json_file(argv[1],strlen(argv[1]))==SEDS_OK);
 const char*mode=argv[2],*sender=argv[3],*form=argv[4],*marker=payload_for(argv[5]);unsigned expected=(unsigned)atoi(argv[8]);uint32_t ep=SEDS_EP_ABORT;SedsPacketView packet={.ty=SEDS_DT_ABORT,.data_size=strlen(marker),.sender=sender,.sender_len=strlen(sender),.endpoints=&ep,.num_endpoints=1,.timestamp=1,.payload=(const uint8_t*)marker,.payload_len=strlen(marker)};
 if(!strcmp(mode,"produce")){
  SedsLocalEndpointDesc local={.endpoint=SEDS_EP_GROUND_STATION,.packet_handler=discard};SedsRouter*r=seds_router_new(now,NULL,&local,1);assert(r);assert(seds_router_set_sender_id(r,sender,strlen(sender))==SEDS_OK);uint32_t assigned=!strcmp(sender,"GS")?101:!strcmp(sender,"RRLS")?102:!strcmp(sender,"identity-collision-51666")?201:!strcmp(sender,"identity-collision-117548")?202:103;assert(seds_router_configure_address(r,2,assigned)==SEDS_OK);
  discovery_out=fopen(argv[7],"wb");assert(discovery_out);assert(seds_router_add_side_packed(r,"CAN",3,capture,NULL,false)>=0);bool queued=false;assert(seds_router_poll_discovery(r,&queued)==SEDS_OK);assert(seds_router_process_all_queues(r)==SEDS_OK);fclose(discovery_out);discovery_out=NULL;
  char alias[32];if(!strcmp(form,"assigned")){snprintf(alias,sizeof alias,"@addr:%u",assigned);packet.sender=alias;packet.sender_len=strlen(alias);}
  uint8_t bytes[1024];int32_t n=seds_pkt_pack(&packet,bytes,sizeof bytes);assert(n>0);FILE*f=fopen(argv[6],"wb");assert(f&&fwrite(bytes,1,(size_t)n,f)==(size_t)n);fclose(f);printf("producer sender=%s form=%s marker=%s bytes=%d\n",sender,form,argv[5],n);seds_router_free(r);return 0;
 }
 SedsLocalEndpointDesc local={.endpoint=SEDS_EP_ABORT,.packet_handler=observe};SedsRouter*r=seds_router_new(now,NULL,&local,1);assert(r);assert(seds_router_set_sender_id(r,"FC",2)==SEDS_OK);assert(seds_router_set_local_network_time(r,true,2026,true,1,true,1,true,0,true,0,true,3,true,0)==SEDS_OK);g_router.r=r;recovery_router=r;int32_t sid=seds_router_add_side_packet(r,"CAN",3,discard,NULL,false);assert(sid>=0);
 if(strcmp(mode,"unknown")!=0)load_discovery(r,(uint32_t)sid,argv[7]);
 if(!strcmp(mode,"collision")){
  const char*alias="@addr:2711038533";const char*a="identity-collision-51666",*b="identity-collision-117548";
  assert(!seds_router_sender_matches_hostname(r,alias,strlen(alias),a,strlen(a)));
  assert(!seds_router_sender_matches_hostname(r,alias,strlen(alias),b,strlen(b)));
  assert(seds_router_sender_matches_hostname(r,"@addr:201",9,a,strlen(a)));
  assert(seds_router_sender_matches_hostname(r,"@addr:202",9,b,strlen(b)));
 }
 if(!strcmp(mode,"direct")){
  if(!strcmp(form,"embeddednul")){packet.sender="GS\0";packet.sender_len=3;}
  assert(board_recovery_command_is_stale(&packet));assert(floor_checks==1);floor_checks=0;
  handler_result=Manual_Abort_handler(&packet,NULL);printf("direct sender=%.*s result=%d stop_calls=%u floor_calls=%u\n",(int)packet.sender_len,packet.sender,handler_result,stop_calls,floor_checks);
 }else{
  FILE*f=fopen(argv[6],"rb");assert(f);uint8_t bytes[1024];size_t n=fread(bytes,1,sizeof bytes,f);assert(n>0);fclose(f);
  SedsPacketView unpacked={0};/* Ordinary command with same old timestamp is floor-denied. */
  assert(board_recovery_command_is_stale(&packet));assert(floor_checks==1);floor_checks=0;
  (void)unpacked;SedsResult result=seds_router_receive_packed_from_side(r,(uint32_t)sid,bytes,n);(void)seds_router_process_all_queues(r);printf("receive result=%d callbacks=%u stop_calls=%u floor_calls=%u\n",result,callbacks,stop_calls,floor_checks);assert(callbacks>=1);if(stop_calls>0)assert(callbacks==1);
 }
 assert(floor_checks==0);assert(stop_calls==expected);printf("PASS expected=%u actual=%u\n",expected,stop_calls);seds_router_free(r);return 0;
}
