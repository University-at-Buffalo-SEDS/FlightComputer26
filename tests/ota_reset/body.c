
static void actual_advance(void){/*FORWARD*/}
static void actual_rewind(void){/*REVERSE*/}
static void go_armed(void){actual_advance();actual_advance();assert(current()==Armed);}
static void begin_job(void){
 static SedsRouter router=1;assert(ota_stream_init(&router)==SEDS_OK);
 SedsP2pStreamEventView e={SEDS_P2P_STREAM_ACCEPTED,7,NULL,0};assert(ota_stream_event(&e,NULL)==SEDS_OK);
 const uint8_t begin[]={1,16,0,0,0};uint8_t chunk[21]={2};
 e.kind=SEDS_P2P_STREAM_DATA;e.payload=begin;e.payload_len=sizeof begin;assert(ota_stream_event(&e,NULL)==SEDS_OK);
 e.payload=chunk;e.payload_len=sizeof chunk;assert(ota_stream_event(&e,NULL)==SEDS_OK);
}
static void finish_job(void){const uint8_t finish[]={3};SedsP2pStreamEventView e={SEDS_P2P_STREAM_DATA,7,finish,1};assert(ota_stream_event(&e,NULL)==(ota_fail_leave?SEDS_ERR:SEDS_OK));}
static void accepted_after_arming(void){go_armed();enter_launch(false);assert(owner.admitted_flight&&!(g_conf&option(Launch_Requested))&&!ascent_initializations);}
static void startup_phase_aba(void){assert(current()==Startup);actual_advance();actual_rewind();assert(current()==Startup);}
int main(int argc,char **argv){assert(argc==2);
 // Production global initializer is Startup. No output/readiness is changed
 // to qualify an unsafe production state: these are mocked boot boundaries.

 begin_job();uint32_t before=fc_recovery_epoch();assert(state.reset_eligible&&state.reset_epoch==before);
 if(!strcmp(argv[1],"mutation_during_finish")){during_finish=startup_phase_aba;}
 if(!strcmp(argv[1],"mutation_before_finish")){startup_phase_aba();}
 if(!strcmp(argv[1],"finish_leave_failure")){ota_fail_leave=true;}
 finish_job();
 if(!strcmp(argv[1],"finish_leave_failure")){ota_fail_leave=false;}
 if(!strcmp(argv[1],"accepted_before_eval")){accepted_after_arming();assert(fc_recovery_epoch()==before);}
 else if(!strcmp(argv[1],"checkpoint_owner_restore")){assert(fc_recovery_restore_admitted_flight()&&owner.admitted_flight);}
 else if(!strcmp(argv[1],"manual_stop")){fc_recovery_manual_stop();assert(fc_recovery_epoch()==before+1);}
 else if(!strcmp(argv[1],"reset_request_same_epoch")){assert(fc_recovery_request_startup(before)==-14&&fc_recovery_epoch()==before);}
 else if(!strcmp(argv[1],"legacy_eval_stop_same_epoch")){fc_recovery_request_eval_stop();assert(owner.legacy_stop&&fc_recovery_epoch()==before);}
 else if(!strcmp(argv[1],"phase_aba_same_epoch")){startup_phase_aba();assert(fc_recovery_epoch()==before);}
 else if(!strcmp(argv[1],"pyro_high_low")){deployment_pulse_start(false);assert(gpio.ODR);for(unsigned i=0;i<500;i++)fc_runtime_tick();assert(!gpio.ODR);}
 else if(!strcmp(argv[1],"terminal")){SedsP2pStreamEventView e={SEDS_P2P_STREAM_CLOSED,7,NULL,0};assert(ota_stream_event(&e,NULL)==SEDS_OK);}
 else if(!strcmp(argv[1],"expired_newjob")){startup_phase_aba();assert(!state.reset_eligible);ota_now=300;telemetry_poll_discovery();assert(!ota_nvics);begin_job();finish_job();ota_now=550;telemetry_poll_discovery();assert(ota_nvics==1);return 0;}
 else if(!strcmp(argv[1],"held_router")){telemetry_mu.tx_mutex_owner=&telemetry_task;telemetry_mu.tx_mutex_ownership_count=1;}
 else if(!strcmp(argv[1],"foreign_worker")){current_thread=&recovery_task;}
 else if(!strcmp(argv[1],"store_leave_failure")){ota_fail_leave=true;}
 else if(!strcmp(argv[1],"time_regression")){state.reset_requested_ms=100;ota_now=99;telemetry_poll_discovery();assert(!state.reset_eligible);}
 else if(!strcmp(argv[1],"finish_leave_failure")||!strcmp(argv[1],"mutation_during_finish")||!strcmp(argv[1],"mutation_before_finish")||!strcmp(argv[1],"unchanged_startup")){}
 else assert(0);
 ota_now=249;telemetry_poll_discovery();assert(!ota_nvics);
 ota_now=250;telemetry_poll_discovery();
 unsigned expected=!strcmp(argv[1],"unchanged_startup");
 printf("{\"case\":\"%s\",\"nvic_resets\":%u,\"expected\":%u,\"store_held_at_reset\":%u,\"irq_held_at_reset\":%u,\"owner_epoch_before\":%u,\"owner_epoch_after\":%u,\"raw_admitted_flight\":%u,\"phase\":%u,\"evaluator_initializations\":%u,\"eligible\":%u,\"pending_reset\":%u}\n",argv[1],ota_nvics,expected,ota_reset_store_held,ota_reset_irq_held,before,fc_recovery_epoch(),owner.admitted_flight,current(),ascent_initializations,state.reset_eligible,state.reset_requested);fflush(stdout);
 assert(ota_nvics==expected&&!ota_reset_store_held&&ota_reset_irq_held==expected);
 if(!expected){assert(!state.reset_eligible&&!state.reset_requested);ota_now=1000;telemetry_poll_discovery();assert(!ota_nvics);}
 return 0;
}
