"""C model of the SEDSnet 6 network-stamp API for host fixtures.

seds_router_network_stamp_age_ms / seds_router_network_stamp_fresh as the
SDK documents them, judged against a fixture clock: one remembered step from
epoch `prev_epoch` to the current epoch that moved the clock by
`step_delta` ms (new minus old reading), a newer epoch first heard
`newer_age` ms ago, and no network time while `ok` is false. Expressions are
C names or values the fixture defines.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def header():
    """Core/Inc/network_stamp.h for fixtures that strip project includes."""
    return (ROOT / 'Core/Inc/network_stamp.h').read_text()


def model(now, ok='1', epoch='0', prev_epoch='0', step_delta='0', newer_age='0',
          storage='', router='SedsRouter'):
    return f'''
{storage} SedsResult seds_router_network_stamp_age_ms({router}*r,uint16_t e,uint64_t ts,uint8_t*kind,int64_t*age){{
 (void)r;*age=0;
 if(!({ok})){{*kind=3;return SEDS_OK;}}
 if(e>({epoch})){{*kind=1;*age=(int64_t)({newer_age});return SEDS_OK;}}
 if(e==({epoch})){{*kind=0;*age=(int64_t)({now})-(int64_t)ts;return SEDS_OK;}}
 /* The one remembered step: translate the old-epoch stamp onto this clock. */
 if(({epoch})!=0U&&e==({prev_epoch})){{*kind=0;*age=(int64_t)({now})-((int64_t)ts+(int64_t)({step_delta}));return SEDS_OK;}}
 *kind=2;return SEDS_OK;
}}
{storage} SedsResult seds_router_network_stamp_fresh({router}*r,uint16_t e,uint64_t ts,uint64_t max_age,uint64_t max_ahead,bool*out){{
 uint8_t kind=3;int64_t age=0;(void)seds_router_network_stamp_age_ms(r,e,ts,&kind,&age);
 *out=(kind==0U&&(age>=0?(uint64_t)age<=max_age:(uint64_t)(-age)<=max_ahead))||(kind==1U&&(uint64_t)age<=max_age);
 return SEDS_OK;
}}
'''
