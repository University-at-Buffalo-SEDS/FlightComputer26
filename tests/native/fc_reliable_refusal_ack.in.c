#include "sedsnet.h"
#include "sedsnet_config.h"
#include "ground_heartbeat_owner.h"
#include "flight_state_contract.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* Real SDK router with the FC's actual ABORT, FLIGHT_CONTROLLER and managed
 * FLIGHT_STATE handlers. Owners below them are intent counters only. */
static struct { SedsRouter *r; } g_router;
volatile uint32_t g_reliable_refusal_count,g_flight_state_retired_received;
static bool source_ok=true,managed_ok=true,controller_ok=true,stopped;
static unsigned stop_calls,postinits,commands,biases,acks,writes;
static void fc_recovery_manual_stop_original_utc(uint64_t source){(void)source;++stop_calls;stopped=true;}
static void fc_recovery_manual_stop(void){fc_recovery_manual_stop_original_utc(0U);}
static __attribute__((unused)) bool fc_recovery_abort_latched(void){return stopped;}
static uint8_t board_recovery_manual_stop_source_allowed(SedsRouter*r,const SedsPacketView*p){assert(r&&p);return source_ok;}
static uint8_t board_recovery_managed_update_allowed(const SedsPacketView*p){assert(p);return managed_ok;}
static __attribute__((unused)) uint8_t board_recovery_managed_report_ignorable(const SedsPacketView*p){assert(p);return 0U;}
static uint8_t board_recovery_controller_update_allowed(const SedsPacketView*p){assert(p);return controller_ok;}
/* distribution.c operator Deploy/Expand selector (compat bytes 13/14); this matrix sends ordinary commands. */
static bool operator_pyro_command(const SedsPacketView*p){return p->ty==SEDS_DT_FLIGHT_COMMAND&&p->payload&&p->payload_len==1&&(p->payload[0]==13||p->payload[0]==14);}
static bool pyro_ok;
static uint8_t board_recovery_operator_pyro_allowed(const SedsPacketView*p){(void)p;assert(pyro_ok&&"operator Deploy/Expand only in the copy-dedupe mode");return 1U;}
static void board_recovery_observe_station_time(SedsRouter*r,const SedsPacketView*p){(void)r;(void)p;}
static SedsResult pulse_ground(void){return SEDS_OK;}
/* distribution.c implicit_postinit runs for real; its owners are stand-ins. A
 * Postinit request is counted at the syscall queue. The recovery owner holds
 * the sequence once the manual Stop latches (fc_recovery_owner.c). */
enum {Postinit=3,G_Armed=6,Postinit_Signal=7,Postinit_Requested=1u<<8,TX_NO_WAIT=0,TX_SUCCESS=0};
typedef uint32_t fc_msg;
#define fc_mask(m) (m)
#define option(o) (o)
static uint32_t g_conf;static int seds_syscall;
static int current(void){return Postinit-1;}
static uint32_t fc_recovery_epoch(void){return 1U;}
static bool fc_recovery_holds_sequence(void){return stopped;}
static unsigned tx_queue_send(void*q,void*m,unsigned w){(void)q;(void)m;(void)w;++postinits;return TX_SUCCESS;}
/* distribution.c puts the return type on its own line. */
static inline SedsResult
/* SOURCE_FUNCTION:implicit_postinit */
static SedsResult process_gps_packet(const uint8_t*d,size_t n){(void)d;(void)n;return SEDS_OK;}
static SedsResult dispatch_flight_cmd(const uint8_t*d,size_t n){(void)d;(void)n;++commands;return SEDS_OK;}
static SedsResult update_ascent_biases(const uint8_t*d,size_t n){(void)d;(void)n;++biases;return SEDS_OK;}
/* SOURCE_FUNCTION:Manual_Abort_handler */
/* SOURCE_FUNCTION:on_fc_packet_impl */
#define FLIGHT_STATE_PERSIST_KEY 0x46535445u
#define FLIGHT_STATE_MAX_VALUE 15U
#define LAUNCHCORE_PERSIST_OK 0
static __attribute__((unused)) uint32_t g_flight_state_cache_reports_ignored;
static uint32_t g_flight_state_cache_value,g_flight_state_cache_writes,g_flight_state_cache_updates,g_flight_state_cache_errors;
static bool g_has_value,g_persist_ready=true,g_network_value_seen;static uint8_t g_persisted_state=UINT8_MAX;static SedsRouter*g_cache_router;
static int persistent_store_set(uint32_t key,const void*p,size_t n){assert(key==FLIGHT_STATE_PERSIST_KEY&&p&&n==1);++writes;return LAUNCHCORE_PERSIST_OK;}
static SedsResult flight_state_cache_apply_network_update(const SedsPacketView*p){return implicit_postinit(p->payload,p->payload_len);}
/* SOURCE_FUNCTION:clear_managed_cache */
/* SOURCE_FUNCTION:persist_update */
static uint64_t now(void*u){(void)u;return 1767225601000ULL;}
/* Copy-dedupe mode: a real station router whose packed radio side is captured. */
static SedsRouter*gs_router;static int32_t gs_side=-1;static uint8_t gs_wire[512];static int32_t gs_wire_len,gs_frames;
static SedsResult gs_tx(const uint8_t*b,size_t n,void*u){(void)u;if(n<=sizeof gs_wire){memcpy(gs_wire,b,n);gs_wire_len=(int32_t)n;++gs_frames;}return SEDS_OK;}
static SedsResult side(const SedsPacketView*p,void*u){(void)u;if(p->sender_len>=7&&!memcmp(p->sender,"E2EACK:",7))acks++;
 if(gs_router){(void)seds_router_receive_from_side(gs_router,(uint32_t)gs_side,p);}
 return SEDS_OK;}
typedef struct {SedsRouter*receiver;uint32_t ingress;} DiscoveryTarget;
static SedsResult learn_discovery(const SedsPacketView*p,void*u){DiscoveryTarget*t=u;return seds_router_receive_from_side(t->receiver,t->ingress,p);}
static void learn_gs(SedsRouter*receiver,uint32_t ingress){
 SedsRouter*gs=seds_router_new(now,NULL,NULL,0);assert(gs);assert(seds_router_set_sender_id(gs,"GS",2)==SEDS_OK);assert(seds_router_configure_address(gs,2,101)==SEDS_OK);
 DiscoveryTarget t={receiver,ingress};assert(seds_router_add_side_packet(gs,"discovery",9,learn_discovery,&t,false)>=0);bool queued=false;assert(seds_router_poll_discovery(gs,&queued)==SEDS_OK);assert(seds_router_process_all_queues(gs)==SEDS_OK);assert(seds_router_process_all_queues(receiver)==SEDS_OK);seds_router_free(gs);
}
static SedsRouter*r;static int32_t sid;static uint8_t wire[512];static int32_t n_wire;static uint64_t stamp=1767225600000ULL;
/* One delivery of a packet; a retransmit reuses the same timestamp/payload. */
static SedsResult deliver(SedsDataType ty,const uint32_t*eps,size_t n,const void*payload,size_t len,uint64_t ts){
 SedsPacketView p={.ty=ty,.data_size=len,.sender="GS",.sender_len=2,.endpoints=eps,.num_endpoints=n,.timestamp=ts,.payload=payload,.payload_len=len};
 n_wire=seds_pkt_pack(&p,wire,sizeof wire);assert(n_wire>0);
 SedsResult result=seds_router_receive_packed_from_side(r,(uint32_t)sid,wire,(size_t)n_wire);(void)seds_router_process_all_queues(r);return result;
}
/* The sender's retransmit: the identical wire frame again. */
static SedsResult redeliver(void){SedsResult result=seds_router_receive_packed_from_side(r,(uint32_t)sid,wire,(size_t)n_wire);(void)seds_router_process_all_queues(r);return result;}
static const uint32_t abort_eps[]={SEDS_EP_GROUND_STATION,SEDS_EP_FLIGHT_CONTROLLER,SEDS_EP_VALVE_BOARD,SEDS_EP_ACTUATOR_BOARD,SEDS_EP_ABORT,SEDS_EP_FLIGHT_STATE,SEDS_EP_SD_CARD};
static const char manual[]="SEDS_MANUAL_ABORT_V1:";
int main(int argc,char**argv){
 setbuf(stdout,NULL);assert(argc==3);assert(seds_schema_register_json_file(argv[1],strlen(argv[1]))==SEDS_OK);
 SedsLocalEndpointDesc locals[]={{.endpoint=SEDS_EP_ABORT,.packet_handler=Manual_Abort_handler},{.endpoint=SEDS_EP_FLIGHT_CONTROLLER,.packet_handler=on_fc_packet_impl}};
 /* The copy-dedupe mode uses the FC firmware's 16-entry recent packet-ID ring
  * (CMakeLists.txt SEDSNET_MAX_RECENT_RX_IDS); the queue budget is the
  * SEDSnet churn test's 32 KiB so the host discovery exchange fits. */
 const SedsRuntimeMemoryConfig fc_memory={.max_queue_budget=32768,.max_recent_rx_ids=16,.starting_queue_size=512,.queue_grow_step=1.0};
 r=!strcmp(argv[2],"flight_cmd_copy_dedupe")?seds_router_new_with_memory(now,NULL,locals,2,SEDS_ROUTER_E2E_DISABLED,0,&fc_memory):seds_router_new(now,NULL,locals,2);assert(r);assert(seds_router_set_sender_id(r,"FC",2)==SEDS_OK);g_router.r=r;g_cache_router=r;
 assert(seds_router_enable_network_variable(r,SEDS_DT_FLIGHT_STATE,true,false)==SEDS_OK);
 assert(seds_router_on_network_variable_update(r,SEDS_DT_FLIGHT_STATE,persist_update,NULL)==SEDS_OK);
 sid=seds_router_add_side_packet(r,"radio",5,side,NULL,false);assert(sid>=0);learn_gs(r,(uint32_t)sid);
 const char*mode=argv[2];
 if(!strcmp(mode,"abort")){
  /* The station's manual ABORT, with its full endpoint set, retransmitted. */
  SedsResult a=deliver(SEDS_DT_ABORT,abort_eps,7,manual,sizeof manual-1U,stamp);
  SedsResult b=redeliver();
  printf("abort results=%d,%d acks=%u stop_calls=%u\n",a,b,acks,stop_calls);
  assert(a==SEDS_OK&&acks>=1&&stop_calls==1);
 }else if(!strcmp(mode,"abort_refused_stopped")){
  /* Refused source while already stopped: acknowledged, nothing new applied. */
  stopped=true;source_ok=false;unsigned before=g_reliable_refusal_count;
  SedsResult a=deliver(SEDS_DT_ABORT,abort_eps,7,manual,sizeof manual-1U,stamp);
  printf("refused stopped result=%d acks=%u stop_calls=%u\n",a,acks,stop_calls);
  assert(a==SEDS_OK&&acks>=1&&stop_calls==0&&g_reliable_refusal_count>before);
 }else if(!strcmp(mode,"abort_refused_running")){
  /* Refused source while NOT stopped: never silently acknowledged. */
  source_ok=false;
  SedsResult a=deliver(SEDS_DT_ABORT,abort_eps,7,manual,sizeof manual-1U,stamp);
  printf("refused running result=%d acks=%u stop_calls=%u\n",a,acks,stop_calls);
  assert(a!=SEDS_OK&&acks==0&&stop_calls==0);
 }else if(!strcmp(mode,"fstate_refused")){
  /* A stale station FLIGHT_STATE (schema endpoints and one to the FC). */
  managed_ok=false;uint8_t state=3;const uint32_t eps[]={SEDS_EP_FLIGHT_STATE,SEDS_EP_SD_CARD,SEDS_EP_GROUND_STATION};
  SedsResult a=deliver(SEDS_DT_FLIGHT_STATE,eps,3,&state,1,stamp);unsigned first=acks;
  const uint32_t fc_eps[]={SEDS_EP_FLIGHT_STATE,SEDS_EP_FLIGHT_CONTROLLER};
  SedsResult b=deliver(SEDS_DT_FLIGHT_STATE,fc_eps,2,&state,1,stamp+1U);
  printf("fstate results=%d,%d acks=%u,%u postinits=%u writes=%u\n",a,b,first,acks,postinits,writes);
  assert(a==SEDS_OK&&b==SEDS_OK&&first>=1&&acks>first&&postinits==0&&writes==0&&!g_has_value);
 }else if(!strcmp(mode,"fstate_aborted_after_stop")){
  /* The station's manual Abort: ABORT lands first and latches the Stop, then
   * its FLIGHT_STATE=15 write arrives. Withholding that receipt made the
   * station re-send it 12 times (4.8 s) on every bench Abort. */
  SedsResult a=deliver(SEDS_DT_ABORT,abort_eps,7,manual,sizeof manual-1U,stamp);unsigned after_abort=acks;
  uint8_t state=15;const uint32_t fc_eps[]={SEDS_EP_FLIGHT_STATE,SEDS_EP_FLIGHT_CONTROLLER};
  SedsResult b=deliver(SEDS_DT_FLIGHT_STATE,fc_eps,2,&state,1,stamp+1U);
  printf("aborted after stop results=%d,%d acks=%u,%u stopped=%d postinits=%u writes=%u value=%u\n",a,b,after_abort,acks,stopped,postinits,writes,g_flight_state_cache_value);
  assert(a==SEDS_OK&&stopped&&after_abort>=1&&acks>after_abort&&postinits==0&&writes==1&&g_flight_state_cache_value==15U);
 }else if(!strcmp(mode,"fstate_armed_held_after_stop")){
  /* Armed would request Postinit: while the Stop holds the sequence it is
   * still not applied (nor acknowledged), so it can land once released. */
  stopped=true;uint8_t state=6;const uint32_t fc_eps[]={SEDS_EP_FLIGHT_STATE,SEDS_EP_FLIGHT_CONTROLLER};
  (void)deliver(SEDS_DT_FLIGHT_STATE,fc_eps,2,&state,1,stamp);
  printf("armed held acks=%u postinits=%u writes=%u\n",acks,postinits,writes);
  assert(acks==0&&postinits==0&&writes==0);
  stopped=false;(void)redeliver();
  printf("armed released acks=%u postinits=%u writes=%u\n",acks,postinits,writes);
  assert(acks>=1&&postinits==1&&writes==1);
 }else if(!strcmp(mode,"flight_cmd_copy_dedupe")){
  /* The station's heard-gap copies of a FLIGHT_COMMAND are the same frame
   * (same bytes, same packet ID) as its SDK re-sends. A real station router
   * sends Deploy and Expand end to end to this FC router; the FC gets each
   * frame, a copy at once (the next gap), a copy after 64 other packets have
   * rolled its 16-entry recent-ID ring over (its receipt long gone), and one
   * more: each command is dispatched (pulses) exactly once. */
  pyro_ok=true;
  gs_router=seds_router_new(now,NULL,NULL,0);assert(gs_router);
  assert(seds_router_set_sender_id(gs_router,"GS",2)==SEDS_OK);assert(seds_router_configure_address(gs_router,2,101)==SEDS_OK);
  gs_side=seds_router_add_side_packed(gs_router,"radio",5,gs_tx,NULL,false);assert(gs_side>=0);
  /* The FC announces its endpoints to the station (discovery over the FC's side). */
  bool queued=false;assert(seds_router_poll_discovery(r,&queued)==SEDS_OK);assert(seds_router_process_all_queues(r)==SEDS_OK);
  assert(seds_router_process_all_queues(gs_router)==SEDS_OK);
  const uint32_t hb_eps[]={SEDS_EP_FLIGHT_CONTROLLER};
  for(uint8_t cmd=13;cmd<=14;++cmd){
   unsigned before=commands,acks_before=acks;int32_t frames_before=gs_frames;
   assert(seds_router_log_bytes(gs_router,SEDS_DT_FLIGHT_COMMAND,&cmd,1)==SEDS_OK);
   assert(seds_router_process_all_queues(gs_router)==SEDS_OK);
   assert(gs_frames>frames_before&&"the station sent the command on its radio side");
   uint8_t copy[512];int32_t copy_len=gs_wire_len;memcpy(copy,gs_wire,(size_t)copy_len);
   SedsResult a=seds_router_receive_packed_from_side(r,(uint32_t)sid,copy,(size_t)copy_len);(void)seds_router_process_all_queues(r);
   unsigned after_first=commands-before;
   SedsResult b=seds_router_receive_packed_from_side(r,(uint32_t)sid,copy,(size_t)copy_len);(void)seds_router_process_all_queues(r);
   unsigned after_copy=commands-before,acked=acks;
   for(uint64_t k=0;k<64;++k){uint8_t beat=0;(void)deliver(SEDS_DT_FLIGHT_BUZZER,hb_eps,1,&beat,1,stamp+1000U+cmd*100U+k);}
   unsigned after_churn=commands-before;
   SedsResult c=seds_router_receive_packed_from_side(r,(uint32_t)sid,copy,(size_t)copy_len);(void)seds_router_process_all_queues(r);
   unsigned after_late=commands-before;
   SedsResult d=seds_router_receive_packed_from_side(r,(uint32_t)sid,copy,(size_t)copy_len);(void)seds_router_process_all_queues(r);
   printf("cmd %u results=%d,%d,%d,%d dispatches after first %u, immediate copy %u, churn %u, late copy %u, second late copy %u acks=%u->%u->%u\n",
          cmd,a,b,c,d,after_first,after_copy,after_churn,after_late,commands-before,acks_before,acked,acks);
   assert(after_first==1U&&commands-before==1U);
  }
  seds_router_free(gs_router);gs_router=NULL;
 }else if(!strcmp(mode,"controller_refused")){
  controller_ok=false;uint8_t cmd=1;const uint32_t eps[]={SEDS_EP_GROUND_STATION,SEDS_EP_SD_CARD,SEDS_EP_FLIGHT_CONTROLLER};
  SedsResult a=deliver(SEDS_DT_FLIGHT_COMMAND,eps,3,&cmd,1,stamp);
  printf("controller result=%d acks=%u commands=%u\n",a,acks,commands);
  assert(a==SEDS_OK&&acks>=1&&commands==0);
 }else assert(!"unknown mode");
 printf("PASS %s\n",mode);seds_router_free(r);return 0;
}
