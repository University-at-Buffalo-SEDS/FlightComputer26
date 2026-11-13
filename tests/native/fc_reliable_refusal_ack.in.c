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
/* distribution.c operator Deploy/Expand selector (compat bytes 12/13, extmap); this matrix sends ordinary commands. */
static bool operator_pyro_command(const SedsPacketView*p){return p->ty==SEDS_DT_FLIGHT_COMMAND&&p->payload&&p->payload_len==1&&(p->payload[0]==12||p->payload[0]==13);}
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
/* deployment_pulse.c runs for real: a Deploy (12) or Expand (13) starts the
 * actual channel pulse, which cannot be re-started while it runs and ends
 * after 500 ticks of fc_runtime_tick; Rewind_State (22) steps the state. */
enum {FC_LOG_PYRO_HIGH,FC_LOG_PYRO_LOW,FC_LOG_PYRO_REJECTED,FC_LOG_PYRO_ACTIVE};
#define CO2_ASSERT_INTERVAL 500
#define REEF_ASSERT_INTERVAL 500
#define Parachute_Deployed (1u<<16)
#define Parachute_Expanded (1u<<17)
#define CO2_Asserted (1u<<18)
#define REEF_Asserted (1u<<19)
static volatile uint32_t g_fc_runtime_clock_active,g_deployment_pulse_completions,co2_ticks,reef_ticks;
static uint32_t irq_mask,hal_ticks,co2_on,reef_on;
static uint32_t __get_PRIMASK(void){return irq_mask;}
static void __disable_irq(void){irq_mask=1;}
static void __set_PRIMASK(uint32_t v){irq_mask=v;}
static uint32_t HAL_GetTick(void){return hal_ticks;}
static void HAL_IncTick(void){++hal_ticks;}
static void HAL_SuspendTick(void){}
static bool fc_recovery_inhibits_manual_pyro(void){return false;}
static bool fc_recovery_inhibits_pyro(void){return false;}
static bool fc_watchdog_can_actuate(void){return true;}
static bool fc_watchdog_record_deployment(bool reef,bool manual){(void)reef;(void)manual;return true;}
static void fc_recovery_ota_reset_retire_locked(void){}
static void fc_sd_capture_event(int kind,bool reef){(void)kind;(void)reef;}
static void co2_high(void){co2_on=1;}
static void co2_low(void){co2_on=0;}
static void reef_high(void){reef_on=1;}
static void reef_low(void){reef_on=0;}
/* SOURCE_FUNCTION:deployment_pulse_start */
/* SOURCE_FUNCTION:fc_runtime_tick */
static unsigned rewinds;
static void run_ticks(unsigned n){while(n--)fc_runtime_tick();}
static SedsResult dispatch_flight_cmd(const uint8_t*d,size_t n){(void)n;++commands;
 if(d[0]==12)deployment_pulse_start(false,true);else if(d[0]==13)deployment_pulse_start(true,true);else if(d[0]==22)++rewinds;
 return SEDS_OK;}
static SedsResult update_ascent_biases(const uint8_t*d,size_t n){(void)d;(void)n;++biases;return SEDS_OK;}
/* SOURCE_FUNCTION:Manual_Abort_handler */
#define FLIGHT_CMD_SEEN 16U
static volatile uint32_t g_flight_state_writes_out_of_order;
/* SOURCE_FUNCTION:flight_state_cache_order */
#define FLIGHT_CMD_SEEN_MS 16000U
static volatile uint32_t g_flight_cmd_repeats_dropped;
/* SOURCE_FUNCTION:flight_cmd_repeat */
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
/* The routers' clock follows the FC tick, so a deliberate re-press carries a new station timestamp. */
static uint64_t now(void*u){(void)u;return 1767225601000ULL+hal_ticks;}
/* Copy-dedupe mode: a real station router whose packed radio side is captured. */
static SedsRouter*gs_router;static int32_t gs_side=-1;static uint8_t gs_wire[512];static int32_t gs_wire_len,gs_frames;
/* Only the station's FLIGHT_COMMAND frames are captured (discovery and
 * receipts also leave on this side). */
static SedsResult gs_tx(const uint8_t*b,size_t n,void*u){(void)u;
 SedsOwnedHeader*h=seds_pkt_unpack_header_owned(b,n);SedsPacketView v;
 const bool command=h&&seds_owned_header_view(h,&v)==SEDS_OK&&v.ty==SEDS_DT_FLIGHT_COMMAND;if(h)seds_owned_header_free(h);
 if(command&&n<=sizeof gs_wire){memcpy(gs_wire,b,n);gs_wire_len=(int32_t)n;++gs_frames;}return SEDS_OK;}
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
 r=(!strcmp(argv[2],"flight_cmd_copy_dedupe")||!strcmp(argv[2],"flight_cmd_ring")||!strcmp(argv[2],"fstate_order"))?seds_router_new_with_memory(now,NULL,locals,2,SEDS_ROUTER_E2E_DISABLED,0,&fc_memory):seds_router_new(now,NULL,locals,2);assert(r);assert(seds_router_set_sender_id(r,"FC",2)==SEDS_OK);g_router.r=r;g_cache_router=r;
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
  /* The station's re-sends and heard-gap copies of a FLIGHT_COMMAND are the
   * same frame (same bytes, same packet ID and station timestamp). A real
   * station router sends Deploy, Expand and Rewind_State end to end to this
   * FC router; the FC gets each frame, a copy at once (its receipt already
   * sent), a copy after 64 other packets rolled its 16-entry recent-ID ring
   * over and the 500 ms pulse ended, and one more 4 s later (inside the
   * station's 4.8 s re-send window): each command is dispatched and pulses
   * exactly once. A deliberate re-press is a new frame with a new station
   * timestamp and runs again. */
  pyro_ok=true;
  gs_router=seds_router_new(now,NULL,NULL,0);assert(gs_router);
  assert(seds_router_set_sender_id(gs_router,"GS",2)==SEDS_OK);assert(seds_router_configure_address(gs_router,2,101)==SEDS_OK);
  gs_side=seds_router_add_side_packed(gs_router,"radio",5,gs_tx,NULL,false);assert(gs_side>=0);
  /* The FC announces its endpoints to the station (discovery over the FC's side). */
  bool queued=false;assert(seds_router_poll_discovery(r,&queued)==SEDS_OK);assert(seds_router_process_all_queues(r)==SEDS_OK);
  assert(seds_router_process_all_queues(gs_router)==SEDS_OK);
  const uint32_t hb_eps[]={SEDS_EP_FLIGHT_CONTROLLER};
  const uint8_t cmds[]={12,13,22};
  for(size_t c=0;c<sizeof cmds;++c){
   const uint8_t cmd=cmds[c];
   unsigned before=commands,pulses_before=g_deployment_pulse_completions,rewinds_before=rewinds;
   uint8_t copy[512];int32_t copy_len=0;SedsResult res[5];unsigned seen[5];
   for(int press=0;press<2;++press){
    int32_t frames_before=gs_frames;
    assert(seds_router_log_bytes(gs_router,SEDS_DT_FLIGHT_COMMAND,&cmd,1)==SEDS_OK);
    /* FLIGHT_COMMAND is Ordered at the station: pump it (time advancing)
     * until this press is on its radio side. */
    for(int k=0;k<100&&gs_frames==frames_before;++k){assert(seds_router_process_all_queues(gs_router)==SEDS_OK);if(gs_frames==frames_before)run_ticks(50);}
    assert(gs_frames>frames_before&&"the station sent the command on its radio side");
    copy_len=gs_wire_len;memcpy(copy,gs_wire,(size_t)copy_len);
    for(int k=0;k<5;++k){
     if(k==2)for(uint64_t m=0;m<64;++m){uint8_t beat=0;(void)deliver(SEDS_DT_FLIGHT_BUZZER,hb_eps,1,&beat,1,stamp+1000U+cmd*100U+m);}
     res[k]=seds_router_receive_packed_from_side(r,(uint32_t)sid,copy,(size_t)copy_len);(void)seds_router_process_all_queues(r);
     run_ticks(k==3?4000U:600U); /* the pulse ends; the last copy is 4 s after the one before */
     seen[k]=commands-before;
    }
    unsigned pulses=g_deployment_pulse_completions-pulses_before,steps=rewinds-rewinds_before;
    printf("cmd %u press %d results=%d,%d,%d,%d,%d dispatches after first %u, immediate copy %u, late copy %u, second late copy %u, 4 s copy %u; pulses %u rewinds %u acks=%u\n",
           cmd,press+1,res[0],res[1],res[2],res[3],res[4],seen[0],seen[1],seen[2],seen[3],seen[4],pulses,steps,acks);
    const unsigned want=(unsigned)press+1U;
    assert(res[0]==SEDS_OK&&res[1]==SEDS_OK&&res[2]==SEDS_OK&&res[3]==SEDS_OK&&res[4]==SEDS_OK&&"every copy is acknowledged");
    assert(seen[4]==want&&"one dispatch per press");
    if(cmd==22)assert(steps==want&&pulses==0U);else assert(pulses==want&&"one pulse per press");
   }
  }
  seds_router_free(gs_router);gs_router=NULL;
 }else if(!strcmp(mode,"flight_cmd_ring")){
  /* Twelve different station flight commands within 16 s, then a late copy
   * of the first (its recent-ID entry rolled over): still remembered, not
   * run again. */
  pyro_ok=true;
  gs_router=seds_router_new(now,NULL,NULL,0);assert(gs_router);
  assert(seds_router_set_sender_id(gs_router,"GS",2)==SEDS_OK);assert(seds_router_configure_address(gs_router,2,101)==SEDS_OK);
  gs_side=seds_router_add_side_packed(gs_router,"radio",5,gs_tx,NULL,false);assert(gs_side>=0);
  bool queued=false;assert(seds_router_poll_discovery(r,&queued)==SEDS_OK);assert(seds_router_process_all_queues(r)==SEDS_OK);
  assert(seds_router_process_all_queues(gs_router)==SEDS_OK);
  const uint32_t hb_eps[]={SEDS_EP_FLIGHT_CONTROLLER};
  const uint8_t cmds[12]={14,15,17,18,19,20,23,24,25,26,27,28};
  uint8_t first[512];int32_t first_len=0;const uint32_t start=hal_ticks;
  for(size_t c=0;c<sizeof cmds;++c){
   int32_t frames_before=gs_frames;
   assert(seds_router_log_bytes(gs_router,SEDS_DT_FLIGHT_COMMAND,&cmds[c],1)==SEDS_OK);
   for(int k=0;k<100&&gs_frames==frames_before;++k){assert(seds_router_process_all_queues(gs_router)==SEDS_OK);if(gs_frames==frames_before)run_ticks(50);}
   assert(gs_frames>frames_before);
   if(c==0){first_len=gs_wire_len;memcpy(first,gs_wire,(size_t)first_len);}
   assert(seds_router_receive_packed_from_side(r,(uint32_t)sid,gs_wire,(size_t)gs_wire_len)==SEDS_OK);(void)seds_router_process_all_queues(r);
   run_ticks(200);
  }
  unsigned before=commands;
  for(uint64_t m=0;m<64;++m){uint8_t beat=0;(void)deliver(SEDS_DT_FLIGHT_BUZZER,hb_eps,1,&beat,1,stamp+9000U+m);}
  SedsResult late=seds_router_receive_packed_from_side(r,(uint32_t)sid,first,(size_t)first_len);(void)seds_router_process_all_queues(r);
  printf("ring: %u commands dispatched in %u ms, late copy of the first result=%d extra dispatches %u\n",before,hal_ticks-start,late,commands-before);
  assert(before==12U&&hal_ticks-start<16000U&&late==SEDS_OK&&commands==before);
  seds_router_free(gs_router);gs_router=NULL;
 }else if(!strcmp(mode,"fstate_order")){
  /* The station writes Idle-like 2, then Aborted 15. A refused write (stale
   * or unready) retires the SDK's managed cache, which alone ordered them;
   * then a late re-send of the 2 arrives (its recent-ID entry rolled over).
   * The board stays Aborted. Then an Advance: 1, 2, a refused write, a late
   * 1, which must not step it back; a newer 7 still applies (wire values
   * that map onto themselves, flight_state_contract.h). */
  const uint32_t fc_eps[]={SEDS_EP_FLIGHT_STATE,SEDS_EP_FLIGHT_CONTROLLER},hb_eps[]={SEDS_EP_FLIGHT_CONTROLLER};
  struct {uint8_t value;uint64_t ts;int refused,late;} steps[]={
   {2,0,0,0},{15,1,0,0},{9,2,1,0},{2,0,0,1},
   {1,3,0,0},{2,4,0,0},{9,5,1,0},{1,3,0,1},{7,6,0,0}};
  const uint32_t want[]={2,15,15,15,1,2,2,2,7};
  for(size_t i=0;i<sizeof steps/sizeof steps[0];++i){
   if(steps[i].late)for(uint64_t m=0;m<64;++m){uint8_t beat=0;(void)deliver(SEDS_DT_FLIGHT_BUZZER,hb_eps,1,&beat,1,stamp+5000U+i*100U+m);}
   managed_ok=!steps[i].refused;unsigned before=acks;
   SedsResult a=deliver(SEDS_DT_FLIGHT_STATE,fc_eps,2,&steps[i].value,1,stamp+steps[i].ts);
   managed_ok=true;
   printf("step %zu write %u ts+%llu%s%s result=%d value=%u acked=%d out_of_order=%u\n",i,steps[i].value,(unsigned long long)steps[i].ts,
          steps[i].refused?" refused":"",steps[i].late?" late copy":"",a,g_flight_state_cache_value,acks>before,g_flight_state_writes_out_of_order);
   assert(a==SEDS_OK&&acks>before&&g_flight_state_cache_value==want[i]);
  }
 }else if(!strcmp(mode,"controller_refused")){
  controller_ok=false;uint8_t cmd=1;const uint32_t eps[]={SEDS_EP_GROUND_STATION,SEDS_EP_SD_CARD,SEDS_EP_FLIGHT_CONTROLLER};
  SedsResult a=deliver(SEDS_DT_FLIGHT_COMMAND,eps,3,&cmd,1,stamp);
  printf("controller result=%d acks=%u commands=%u\n",a,acks,commands);
  assert(a==SEDS_OK&&acks>=1&&commands==0);
 }else assert(!"unknown mode");
 printf("PASS %s\n",mode);seds_router_free(r);return 0;
}
