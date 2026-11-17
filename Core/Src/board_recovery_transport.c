#include "persistent_store_health.h"
#include "board_commissioning.h"
#include "tx_api.h"
#include "board_recovery_transport.h"
#include "board_recovery_wire.h"
#include "board_recovery_boot.h"
#include "board_recovery_floor.h"
#include "board_recovery.h"
#include "fc_recovery_owner.h"
#include "network_stamp.h"
#include <string.h>
static SedsRouter *recovery_router;
/* Delivered reliable packets deliberately not applied, yet acknowledged so
 * the sender stops retransmitting them. */
volatile uint32_t g_reliable_refusal_count __attribute__((used));
void board_recovery_transport_set_router(SedsRouter *router) { recovery_router=router; }
/* A station checked by its fixed name ("GS", "RRLS"): matches the name, an
 * address the SDK resolves to it, or, when this board never learned the
 * station (rebooted, or behind a relay before it heard the station), the
 * name's own compact identity unless another known name shares it. */
static uint8_t authorized_owner(SedsRouter *router,const SedsPacketView *packet,
                                const char *owner,size_t owner_len)
{
 return packet!=NULL && seds_router_sender_matches_station(router,
     packet->sender,packet->sender_len,owner,owner_len);
}
/* A station: the GS, or the RRLS when it takes over (name or compact alias,
 * resolved by the SDK). Freshness, epoch and nonce checks still apply. */
static uint8_t authorized_primary(SedsRouter *router,const SedsPacketView *packet)
{
 return authorized_owner(router,packet,"GS",2U) || authorized_owner(router,packet,"RRLS",4U);
}
uint8_t board_recovery_manual_stop_source_allowed(SedsRouter *router,const SedsPacketView *packet)
{
 /* This known manual OFF-only intent is deliberately NOT ordinary UTC/floor
  * gated. A clock rollback must not disable an operator emergency Stop. */
 return router!=0 && packet!=0 &&
        (authorized_owner(router,packet,"GS",2U) ||
         authorized_owner(router,packet,"RRLS",4U));
}
/* Station request validity window: at most 15 s old and 1 s ahead of this
 * board's network time, judged by SEDSnet across the time steps it remembers
 * (a newer-epoch stamp passes while this board re-anchors onto it). A stamp
 * from before a forgotten step, or no network time, is never fresh. */
uint8_t board_recovery_stamp_fresh(SedsRouter*router,uint64_t key){
 bool fresh=false;if(!router)router=recovery_router;
 return router!=NULL&&key!=0U&&seds_router_network_stamp_fresh(router,network_stamp_epoch(key),network_stamp_ms(key),15000U,1000U,&fresh)==SEDS_OK&&fresh;
}
uint8_t board_recovery_command_is_stale(const SedsPacketView *packet)
{
 if (packet==0) return 1U;
 const uint64_t key=NETWORK_STAMP_OF(packet);
 /* Ordered by (epoch, stamp): a request after a backward step is newer than
  * a floor set before it; a copy stamped before the floor never is. */
 if (!board_recovery_command_time_allowed(key)) return 1U;
 if (board_recovery_floor_active() && !board_recovery_stamp_fresh(recovery_router,key)) return 1U;
 return 0U;
}
uint8_t board_recovery_managed_update_allowed(const SedsPacketView *packet)
{
 if (!persistent_store_controls_ready() || !board_recovery_floor_is_healthy()) return 0U;
 if (!board_recovery_floor_active()) return packet!=NULL && packet->ty==SEDS_DT_FLIGHT_STATE ? recovery_router!=NULL && authorized_primary(recovery_router,packet) : 1U;
 return recovery_router!=0 && authorized_primary(recovery_router,packet) &&
        !board_recovery_command_is_stale(packet);
}
/* The operator's emergency Deploy/Expand: the station's sender identity is the
 * only gate, like the manual Stop. A storage or floor fault, or no network
 * time, must not refuse it; a request more than 15 s old by the network clock
 * (judged across time steps) still does, so a long-delayed retry cannot fire
 * late, as does one stamped before a step this board no longer remembers. */
uint8_t board_recovery_operator_pyro_allowed(const SedsPacketView *packet)
{
 if (recovery_router==0 || !board_recovery_manual_stop_source_allowed(recovery_router,packet)) return 0U;
 uint8_t kind=3U;int64_t age=0;
 if (packet->timestamp==0U || packet->timestamp==UINT64_MAX ||
     seds_router_network_stamp_age_ms(recovery_router,packet->time_epoch,packet->timestamp,&kind,&age)!=SEDS_OK) return 1U;
 /* 0 age (negative: ahead), 1 newer epoch (age: since first heard of it),
  * 2 older untranslatable epoch, 3 no network time. */
 if (kind==3U) return 1U;
 return kind!=2U && age<=15000;
}
/* Op 3 admission from the actual readback (taken under the IRQ guard that
 * deployment_pulse_start uses): no pyro output HIGH and no pulse ticking.
 * FC_RESET_ANY_HOLDS_UNDEPLOYED_FLIGHT (an open policy question) also
 * refuses Ascent..Apogee. Never seals, cancels or advances anything. */
#ifndef FC_RESET_ANY_HOLDS_UNDEPLOYED_FLIGHT
#define FC_RESET_ANY_HOLDS_UNDEPLOYED_FLIGHT 0
#endif
static bool fc_reset_any_state_permitted(const BoardRecoverySnapshot *s)
{
 if (FC_RESET_ANY_HOLDS_UNDEPLOYED_FLIGHT && s->actual_state>=8U && s->actual_state<=10U) return false; /* Ascent..Apogee */
 return !(s->flags&BOARD_RECOVERY_OUTPUTS_VALID) || (s->flags&BOARD_RECOVERY_SAFE_OUTPUTS);
}
/* Ordinary controller admission: identity reuse only, never emergency-Stop policy. */
uint8_t board_recovery_controller_update_allowed(const SedsPacketView *packet)
{
 if (!persistent_store_controls_ready() || !board_recovery_floor_is_healthy()) return 0U;
 if (!board_recovery_floor_active()) return 1U;
 return recovery_router!=0 && board_recovery_manual_stop_source_allowed(recovery_router,packet) &&
        !board_recovery_command_is_stale(packet);
}
/* Why the check above refuses ordinary controller updates, for BOARD_HEALTH
 * word 6 bits 24..27: 1 store not ready, 2 commissioning hold, 4 store
 * ownership fault, 8 recovery floor failed. Cached state only, no storage IO. */
uint32_t board_recovery_controller_gate(void)
{
 uint32_t gate=0U;
 if (!persistent_store_controls_ready())
  gate=persistent_store_commission_pending()?2U:persistent_store_ownership_faulted()?4U:1U;
 if (!board_recovery_floor_cached_health()) gate|=8U;
 return gate;
}
/* A well-formed FLIGHT_STATE whose sender is not the primary station is a
 * state report (this board's own, or the FC's that the station follows),
 * never a desired-state write: callers ignore it instead of failing SDK
 * delivery. */
uint8_t board_recovery_managed_report_ignorable(const SedsPacketView *packet)
{
 return packet!=NULL && packet->ty==SEDS_DT_FLIGHT_STATE && recovery_router!=NULL &&
        packet->sender!=NULL && packet->sender_len!=0U &&
        !authorized_primary(recovery_router,packet);
}
uint8_t board_recovery_automatic_is_stale(const SedsPacketView *packet)
{
 return board_recovery_command_is_stale(packet);
}

static SedsResult commissioning_status(SedsRouter*router,uint8_t result,const BoardRecoveryRequest*request,uint32_t uptime,uint8_t extra_flags){
 BoardRecoverySnapshot snapshot={.actual_state=255};
 if(board_recovery_snapshot(&snapshot)!=0)result=BOARD_RECOVERY_RESULT_IO;
 uint64_t generation=board_commissioning_query_generation();
 if(!generation&&!board_commissioning_uncertain_first())snapshot.flags&=(uint8_t)~BOARD_RECOVERY_SUPPORTED;
 if(board_recovery_is_recover(request->operation)&&result==BOARD_RECOVERY_RESULT_OK&&(snapshot.flags&BOARD_RECOVERY_ABORT_LATCHED)&&!board_commissioning_pending())result=BOARD_RECOVERY_RESULT_STALE;
 uint8_t wire[BOARD_RECOVERY_STATUS_SIZE];
 board_recovery_encode_status(wire,result,snapshot.actual_state,(uint8_t)(snapshot.flags|extra_flags|BOARD_RECOVERY_VERIFIED_IMPLEMENTATION|BOARD_RECOVERY_MANUAL_UTC_IMPLEMENTATION|BOARD_RECOVERY_RESET_ANY_STATE_IMPLEMENTATION),request,generation,snapshot.abort_epoch,uptime,snapshot.actual_outputs_mask);
 return seds_router_log_queue_typed(router,SEDS_DT_BOARD_STATE_STATUS,wire,sizeof wire,1,SEDS_EK_UNSIGNED);
}
static bool commissioning_fresh(void*context,uint64_t key){return board_recovery_stamp_fresh((SedsRouter*)context,key)!=0U;}
static bool commissioning_reply(void*context,const BoardRecoveryRequest*request,uint8_t result){uint32_t uptime=(uint32_t)(((uint64_t)tx_time_get()*1000U)/TX_TIMER_TICKS_PER_SECOND);return commissioning_status((SedsRouter*)context,result,request,uptime,0U)==SEDS_OK;}
/* BOARD_STATE_STATUS is an ordered reliable stream: a status goes on air only
 * once every older one has its station receipt. The Abort push carries the
 * live snapshot, so it supersedes them; without this it waits behind them, for
 * seconds when their receipts are lost on air. Rows are read one at a time (no
 * scratch); a completed row shifts the rest down, so that index is read again.
 * The SDK never completes ABORT or another urgent type this way. */
static void retire_superseded_status(SedsRouter*router){
 SedsReliablePendingRow row;
 for(size_t index=0,guard=0;guard<64U;++guard){
  if(seds_router_copy_reliable_pending(router,index,&row,1)<=(int32_t)index)return;
  if(row.data_type==(uint32_t)SEDS_DT_BOARD_STATE_STATUS&&
     seds_router_complete_reliable(router,row.packet_id)==SEDS_OK)continue;
  ++index;
 }
}
/* Issuer of the last valid station QUERY, echoed by the Abort push. */
static uint64_t last_query_issuer;
/* Caller holds the router: one unsolicited status for a fresh Abort latch, so
 * the station sees it even while its ordered queries wait. Nonce 0, the last
 * valid QUERY issuer, the QUERY generation. */
SedsResult board_recovery_push_abort_status(SedsRouter*router){
 if(!router)return SEDS_BAD_ARG;
 retire_superseded_status(router);
 BoardRecoveryRequest request={0};request.issuer_session=last_query_issuer;
 return commissioning_status(router,BOARD_RECOVERY_RESULT_OK,&request,
  (uint32_t)(((uint64_t)tx_time_get()*1000U)/TX_TIMER_TICKS_PER_SECOND),BOARD_RECOVERY_ABORT_LATCHED);
}
SedsResult board_recovery_handle(SedsRouter *router,const SedsPacketView *packet,
                                uint32_t local_endpoint,uint32_t uptime_ms)
{
 BoardRecoveryRequest request;
 /* A retry of a misaddressed, foreign or malformed request is refused the
  * same way: acknowledge it once instead of failing SDK delivery. */
 if (router==0 || packet==0 || packet->ty!=SEDS_DT_BOARD_RECOVERY_REQUEST ||
     packet->num_endpoints!=1U || packet->endpoints==0 ||
     packet->endpoints[0]!=local_endpoint || authorized_primary(router,packet)==0U ||
     board_recovery_decode_request(packet->payload,packet->payload_len,&request)==0U)
 { ++g_reliable_refusal_count; return SEDS_OK; }
 BoardRecoverySnapshot snapshot={.actual_state=255U};
 uint64_t generation=board_commissioning_query_generation();
 /* Stored MAX is exhaustion; only strict eligible cache may advertiseMAX. */
 /* Eligible MAX is supplied only by the strict commissioning boot cache. */
 uint8_t result=BOARD_RECOVERY_RESULT_OK;
 if (board_recovery_snapshot(&snapshot)!=0) result=BOARD_RECOVERY_RESULT_IO;
 if ((generation==0U&&!board_commissioning_uncertain_first()) || (snapshot.flags&BOARD_RECOVERY_SUPPORTED)==0U) {
  snapshot.flags&=(uint8_t)~BOARD_RECOVERY_SUPPORTED;
  result=BOARD_RECOVERY_RESULT_UNSUPPORTED;
 }
 /* FC owns only explicit Startup recovery, not Idle/output recovery.
  * Reject unsupported intent before loading/committing a floor or calling owner. */
 if (board_recovery_is_recover(request.operation) && request.desired_state!=0U)
  result=BOARD_RECOVERY_RESULT_INVALID;
 /* Commissioning has its own explicit two-leg path. QUERY only reads boot
  * caches/registers and never provisions or refreshes storage. */
 if(request.operation==BOARD_RECOVERY_QUERY){
  last_query_issuer=request.issuer_session;
  if(result==BOARD_RECOVERY_RESULT_OK){
   result=board_commissioning_query_result();
   if(board_commissioning_pending()&&!board_commissioning_note_query(&request,NETWORK_STAMP_OF(packet),snapshot.abort_epoch,router,commissioning_fresh))result=BOARD_RECOVERY_RESULT_IO;
  }
  if(!board_commissioning_pending()&&fc_recovery_reset_failed()&&result==BOARD_RECOVERY_RESULT_OK)result=BOARD_RECOVERY_RESULT_IO;
  return commissioning_status(router,result,&request,uptime_ms,0U);
 }
 /* Op 3 never cuts a pulse: refuse while one runs, before any seal or epoch. */
 if(request.operation==BOARD_RECOVERY_RESET_STARTUP_ANY&&result==BOARD_RECOVERY_RESULT_OK&&!fc_reset_any_state_permitted(&snapshot))
  result=BOARD_RECOVERY_RESULT_UNSAFE;
 if(result==BOARD_RECOVERY_RESULT_OK){
  int32_t staged=board_commissioning_stage(&request,NETWORK_STAMP_OF(packet),snapshot.abort_epoch,router,commissioning_reply,commissioning_fresh);
  if(staged!=-2){if(staged==-1)return SEDS_OK;return commissioning_status(router,(uint8_t)staged,&request,uptime_ms,0U);}
 }
 /* No ordinary fallback may bypass the durable transaction service. Strict
  * unclassified/tombstoned/failed metadata remains observable and inhibited. */
 return commissioning_status(router,result==BOARD_RECOVERY_RESULT_OK?BOARD_RECOVERY_RESULT_IO:result,&request,uptime_ms,0U);
}
