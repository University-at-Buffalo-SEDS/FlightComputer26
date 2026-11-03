static void evaluator_entry(void){
 evaluation_control_barrier();
 enter_flight_mode(atomic_load(&g_conf));
}
static void sensor_failure_handoff(void){
 smon.to_abort=TO_ABORT;devctr_if[IMU]=TO_ABORT;process_sensor_report(Bad_Accel_Z);
 if(!fc_recovery_eval_pause_requested())return;
 evaluator_context=true;
 if(setjmp(scheduler)==0){evaluation_control_barrier();assert(!"paused evaluator must yield");}
 evaluator_context=false;
 fc_recovery_control_service();
 assert(terminations&&resets&&evaluation_task.tx_thread_state==TX_SUSPENDED);
 /*ACTUAL_SENSOR_RESTART_BRANCH*/
}
int main(int argc,char **argv){assert(argc==2);
 if(!strcmp(argv[1],"prelaunch")){
  evaluation_task.tx_thread_state=TX_RUNNING;fc_recovery_evaluation_running();sensor_failure_handoff();
  assert((g_conf&option(In_Aborted_State))&&evaluation_task.tx_thread_state==TX_SUSPENDED);
 }else if(!strcmp(argv[1],"accepted_preboost")||!strcmp(argv[1],"accepted_before_eval")){
  enter_launch(false);assert(evaluation_task.tx_thread_state==TX_RUNNING);
  assert(!fc_recovery_abort_latched()&&current()==Armed&&!(g_conf&option(Launch_Requested)));
  const bool entered=strcmp(argv[1],"accepted_before_eval")!=0;
  if(entered){evaluator_entry();assert(ascent_initializations==1&&(g_conf&option(Launch_Requested))&&current()==Armed);}
  else assert(!ascent_initializations&&!(g_conf&option(Launch_Requested)));
  sensor_failure_handoff();
  printf("accepted=1 phase=%u aborted_flag=%u owner_abort=%u eval_state=%u init_count=%u epoch=%u\n",current(),!!(g_conf&option(In_Aborted_State)),fc_recovery_abort_latched(),evaluation_task.tx_thread_state,ascent_initializations,fc_recovery_epoch());fflush(stdout);
  assert(!(g_conf&option(In_Aborted_State))); /* USER CONTRACT: expected old RED */
  assert(evaluation_task.tx_thread_state==TX_RUNNING);
  evaluator_entry();assert(ascent_initializations==1);
  assert(fc_recovery_flight_is_admitted());
 }else if(!strcmp(argv[1],"accepted_then_manual_stop")){
  enter_launch(false);evaluator_entry();fc_recovery_manual_stop();uint32_t epoch=fc_recovery_epoch();
  assert(!fc_recovery_flight_is_admitted());assert(!fc_recovery_restore_admitted_flight());
  abortion_due_failures();assert(fc_recovery_abort_latched()&&fc_recovery_holds_sequence()&&seal&&!gpio.ODR);
  enter_launch(false);assert(fc_recovery_epoch()==epoch&&fc_recovery_abort_latched());
 }else if(!strcmp(argv[1],"accepted_then_reset")){
  enter_launch(false);evaluator_entry();uint32_t epoch=fc_recovery_epoch();assert(fc_recovery_request_startup(epoch)==-14);
  assert(!fc_recovery_flight_is_admitted());assert(!fc_recovery_restore_admitted_flight());
  abortion_due_failures();assert(fc_recovery_holds_sequence()&&seal&&!gpio.ODR);assert(!(g_conf&option(In_Aborted_State)));
 }else if(!strcmp(argv[1],"storage_unready_seed")){
  store_ready=false;assert(!fc_recovery_restore_admitted_flight()&&!fc_recovery_flight_is_admitted());assert(!gpio.ODR);
 }else if(!strcmp(argv[1],"commissioning_seed")){
  fc_recovery_restore_commissioning_hold();assert(!fc_recovery_restore_admitted_flight()&&!fc_recovery_flight_is_admitted());assert(fc_recovery_abort_latched()&&!gpio.ODR);
 }else if(!strcmp(argv[1],"verified_watchdog_seed")){
  g_fc_watchdog_resumed=true;g_conf|=option(Launch_Requested);assert(fc_recovery_restore_admitted_flight());
  evaluation_task.tx_thread_state=TX_RUNNING;evaluator_entry();assert(!ascent_initializations);
  sensor_failure_handoff();assert(!(g_conf&option(In_Aborted_State))&&evaluation_task.tx_thread_state==TX_RUNNING);
  evaluator_entry();assert(!ascent_initializations&&fc_recovery_flight_is_admitted());
 }else if(!strcmp(argv[1],"stale_reset_no_revoke")){
  enter_launch(false);evaluator_entry();assert(fc_recovery_flight_is_admitted());
  assert(fc_recovery_request_startup(0)==-10&&fc_recovery_flight_is_admitted());
 }else if(!strcmp(argv[1],"manual_eval_stop")){
  enter_launch(false);evaluator_entry();fc_recovery_request_eval_stop();assert(!fc_recovery_flight_is_admitted());
 }else if(!strcmp(argv[1],"internal_not_authority")){
  enter_launch(true);assert(!fc_recovery_flight_is_admitted());
 }else if(!strcmp(argv[1],"reset_then_fresh_launch")){
  enter_launch(false);evaluator_entry();uint32_t epoch=fc_recovery_epoch();assert(fc_recovery_request_startup(epoch)==-14);
  evaluator_context=true;if(setjmp(scheduler)==0){evaluation_control_barrier();assert(0);}evaluator_context=false;
  fc_recovery_control_service();fc_recovery_network_service();fc_recovery_control_service();
  assert(current()==Startup&&!fc_recovery_flight_is_admitted()&&fc_recovery_requires_new_flight());
  assert(!(g_conf&option(Launch_Requested))&&seal&&fc_recovery_epoch()==epoch+1);
  sm.flight=Armed;enter_launch(false);assert(fc_recovery_flight_is_admitted()&&fc_recovery_epoch()==epoch+2&&!seal);
  assert(!(g_conf&option(Launch_Requested)));evaluator_entry();assert(ascent_initializations==2);
 }else assert(!"unknown case");
 return 0;
}
