"""Actual FC manual marker handler/auth: legacy/AUTO never gain a new effect."""
from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
STUB=r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
typedef int SedsResult;typedef int SedsRouter;
typedef struct {uint32_t ty;const char*sender;size_t sender_len;const uint8_t*payload;size_t payload_len;uint64_t timestamp;} SedsPacketView;
#define SEDS_OK 0
#define SEDS_BAD_ARG -10
#define SEDS_PERMISSION_DENIED -11
#define SEDS_DT_ABORT 135612662U
static SedsRouter router;
static struct {SedsRouter*r;}g_router={&router};
static unsigned stops;
static void fc_recovery_manual_stop(void){++stops;}
static int seds_router_resolve_hostname_address(SedsRouter*r,const char*n,size_t len,uint32_t*out){
 assert(r);if(len==2&&!memcmp(n,"GS",2))*out=1234;else if(len==4&&!memcmp(n,"RRLS",4))*out=2345;else assert(0);return 0;}
'''
MAIN=r'''
int main(void){
 const char marker[]="SEDS_MANUAL_ABORT_V1:";assert(sizeof marker-1==21);
 SedsPacketView p={.ty=SEDS_DT_ABORT,.payload=(const uint8_t*)marker,.payload_len=21,.timestamp=0};
 const char *allowed[]={"GS","RRLS","@addr:1234","@addr:2345"};
 for(unsigned i=0;i<4;i++){p.sender=allowed[i];p.sender_len=strlen(p.sender);unsigned old=stops;assert(Manual_Abort_handler(&p,0)==0&&stops==old+1);}
 const char *rejected[]={"PB","gs","@addr:0","@addr:3456","@addr:+1234","@addr:4294967296"};
 for(unsigned i=0;i<6;i++){p.sender=rejected[i];p.sender_len=strlen(p.sender);unsigned old=stops;assert(Manual_Abort_handler(&p,0)==SEDS_PERMISSION_DENIED&&stops==old);}
 p.sender="GS";p.sender_len=2;
 const char *ignored[]={"Manual Abort Command Issued","SEDS_AUTO_ABORT_V1:","SEDS_MANUAL_ABORT_V1:X","SEDS_MANUAL_ABORT_V1",""};
 for(unsigned i=0;i<5;i++){p.payload=(const uint8_t*)ignored[i];p.payload_len=strlen(ignored[i]);unsigned old=stops;assert(Manual_Abort_handler(&p,0)==0&&stops==old);}
 p.payload=NULL;p.payload_len=21;assert(Manual_Abort_handler(&p,0)==0&&stops==4);
 p.payload=(const uint8_t*)marker;p.payload_len=21;p.sender=NULL;assert(Manual_Abort_handler(&p,0)==SEDS_PERMISSION_DENIED&&stops==4);
 assert(Manual_Abort_handler(NULL,0)==SEDS_BAD_ARG);
 return 0;}
'''
class FCManualAbortOriginTests(unittest.TestCase):
 def test_actual_handler_and_shared_alias_auth(self):
  transport=(ROOT/'Core/Src/board_recovery_transport.c').read_text();auth=transport[transport.index('static uint8_t authorized_owner('):transport.index('uint8_t board_recovery_command_is_stale(')]
  # The recovery-only GS wrapper is excluded; both actual manual role branches remain.
  a=auth.index('static uint8_t authorized_primary(');b=auth.index('uint8_t board_recovery_manual_stop_source_allowed(');auth=auth[:a]+auth[b:]
  telemetry=(ROOT/'Core/Src/telemetry.c').read_text();start=telemetry.index('static SedsResult Manual_Abort_handler(const SedsPacketView *packet, void *user)');end=telemetry.index('\nstatic SedsResult Board_Recovery_handler(',start);handler=telemetry[start:end]
  with tempfile.TemporaryDirectory()as tmp:
   p=Path(tmp);src=p/'main.c';src.write_text(STUB+auth+handler+MAIN);exe=p/'test'
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(src),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)
