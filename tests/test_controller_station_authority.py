"""Actual FC helper/callsite policy; identity and storage boundaries are modeled."""
from pathlib import Path
import os
import subprocess,tempfile,unittest
ROOT=Path(os.environ.get("FC_RRLS_POLICY_SOURCE",str(Path(__file__).resolve().parents[1])))
def function(text,name):
 i=text.index(name+'(');start=text.rfind('\n',0,i)+1;end=text.index('{',i)+1;depth=1
 while depth:depth+=(text[end]=='{')-(text[end]=='}');end+=1
 return text[start:end]
class ControllerStationAuthorityTests(unittest.TestCase):
 def test_actual_inactive_active_type_identity_freshness_matrix(self):
  transport=(ROOT/'Core/Src/board_recovery_transport.c').read_text();distribution=(ROOT/'Core/Src/distribution.c').read_text()
  code=r'''
#include "sedsnet.h"
#include "sedsnet_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
static int cookie;static SedsRouter*recovery_router=(SedsRouter*)&cookie;
static bool ready=true,healthy=true,active=true,clock_ok=true;
static uint64_t clock_now=10000;static unsigned commands,biases,states;
bool persistent_store_controls_ready(void){return ready;}
uint8_t board_recovery_floor_is_healthy(void){return healthy;}
uint8_t board_recovery_floor_active(void){return active;}
uint8_t board_recovery_command_time_allowed(uint64_t source){return source>100;}
SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){assert(r);*out=clock_now;return clock_ok?SEDS_OK:SEDS_ERR;}
bool seds_router_sender_matches_hostname(const SedsRouter*r,const char*s,size_t n,const char*name,size_t len){assert(r);return s&&n==len&&!memcmp(s,name,len);}
static SedsResult pulse_ground(void){return SEDS_OK;}
static SedsResult implicit_postinit(const uint8_t*p,size_t n){assert(p&&n==1);states++;return SEDS_OK;}
static SedsResult process_gps_packet(const uint8_t*p,size_t n){(void)p;(void)n;return SEDS_HANDLER_ERROR;}
static SedsResult dispatch_flight_cmd(const uint8_t*p,size_t n){assert(p&&n==1);commands++;return SEDS_OK;}
static SedsResult update_ascent_biases(const uint8_t*p,size_t n){assert(p&&n==1);biases++;return SEDS_OK;}
#include "ground_heartbeat_owner.h"
'''
  for name in ['authorized_owner','authorized_primary','board_recovery_manual_stop_source_allowed','board_recovery_command_is_stale','board_recovery_managed_update_allowed','board_recovery_controller_update_allowed']:
   if name+'(' in transport:code+='\n'+function(transport,name)
  code+='\n'+function(distribution,'on_fc_packet_impl')+r'''
int main(void){
 uint8_t value=1;SedsPacketView p={.payload=&value,.payload_len=1};const char*senders[]={"GS","RRLS","PB","@addr:unknown"};const uint32_t types[]={SEDS_DT_FLIGHT_STATE,SEDS_DT_FLIGHT_COMMAND,SEDS_DT_ASCENT_BIASES};const uint64_t times[]={9000,0,UINT64_MAX,1,4999,11001};unsigned count=0;
 for(unsigned r=0;r<2;r++)for(unsigned h=0;h<2;h++)for(unsigned a=0;a<2;a++)for(unsigned s=0;s<4;s++)for(unsigned k=0;k<3;k++)for(unsigned t=0;t<6;t++){
  ready=r;healthy=h;active=a;p.sender=senders[s];p.sender_len=strlen(p.sender);p.ty=types[k];p.timestamp=times[t];
  bool expected=ready&&healthy&&(!active?(k!=0||s==0):((k==0?s==0:s<=1)&&t==0));unsigned before=commands+biases+states;SedsResult result=on_fc_packet_impl(&p,NULL);
  assert((result==SEDS_OK)==expected);assert(commands+biases+states==before+(expected?1U:0U));count++;
 }
 ready=healthy=active=true;p.ty=SEDS_DT_FLIGHT_COMMAND;p.sender="RRLS";p.sender_len=4;p.timestamp=9000;clock_ok=false;unsigned before=commands;assert(on_fc_packet_impl(&p,NULL)==SEDS_PERMISSION_DENIED&&commands==before);clock_ok=true;
 p.ty=SEDS_DT_FLIGHT_STATE;assert(on_fc_packet_impl(&p,NULL)==SEDS_PERMISSION_DENIED);p.ty=SEDS_DT_ASCENT_BIASES;assert(on_fc_packet_impl(&p,NULL)==SEDS_OK);
 assert(on_fc_packet_impl(NULL,NULL)==SEDS_HANDLER_ERROR);p.sender_len=0;assert(on_fc_packet_impl(&p,NULL)==SEDS_HANDLER_ERROR);
 printf("actual FC source/type/readiness/floor/time matrix %u PASS\n",count);return 0;}
'''
  import os
  sdk=Path(os.environ['SEDSNET_TEST_SOURCE'])
  with tempfile.TemporaryDirectory()as tmp:
   d=Path(tmp);p=d/'probe.c';p.write_text(code);exe=d/'probe';subprocess.run(['cc','-std=c17','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-fsanitize=address,undefined','-I'+str(ROOT/'Core/Inc'),'-I'+str(sdk/'C-Headers'),str(p),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)
