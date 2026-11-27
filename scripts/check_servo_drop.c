#include "global_def.h"
#include "sim_drive.h"
#include <stdlib.h>
#include <stdio.h>
uint16_t __real_sim_drive_get_sw(int,int);
void __real_sim_drive_step_axis(int,int,uint16_t,int32_t,uint64_t);
static int first=-1, alarm_seen, held;
uint16_t __wrap_sim_drive_get_sw(int axis,int sub){
 uint16_t sw=__real_sim_drive_get_sw(axis,sub);
 if(axis!=1 || sub!=1) return sw;
 if(first<0 && g_axis[axis].cia_step==3) first=cycle;
 if(first<0) return sw;
 if(cycle==first) return 0x23;
#ifndef BASELINE
 if(g_sys_alarm_state && !(atomic_load(&g_servo_ready_mask)&2) && g_axis[axis].fault_slave==1) alarm_seen=1;
#else
 if(cycle-first>=8){fprintf(stderr,"baseline retained software step %d after slave loss\n",g_axis[axis].cia_step);_Exit(g_axis[axis].cia_step==3?1:2);}
#endif
 if(g_axis[axis].cia_step==1){
    for(int s=0;s<2;s++) g_sim_axis[axis].motor[s].pos=1200;
    g_interpolator.is_moving=1;
    g_interpolator.hold_state=HOLD_PAUSED;
    g_interpolator.time_scale=0;
    g_interpolator.current_pos[axis]=0;
 }
 return sw;
}
void __wrap_sim_drive_step_axis(int i,int s,uint16_t cw,int32_t pos,uint64_t tick){
#ifdef BASELINE
 if(i==1 && s==1 && g_axis[i].cia_step==3){if(first<0)first=cycle; if(cycle-first>=8){fprintf(stderr,"baseline still publishes all-ready=%d after 8 cycles\n",g_all_axis_op_ready);_Exit(g_all_axis_op_ready?1:2);}}
#endif
 if(i==1 && alarm_seen && first>=0 && cycle>first && g_axis[i].cia_step==3 && cw==CW_ENABLE_OP){
    if(!alarm_seen || pos!=1200){fprintf(stderr,"FAIL sub=%d target=%d alarm=%d\n",s,pos,alarm_seen);_Exit(1);}
    if(s==1 && ++held==32){fprintf(stderr,"PASS Y slave loss detected; both targets held at displaced feedback for 32 cycles\n");_Exit(0);}
 }
 __real_sim_drive_step_axis(i,s,cw,pos,tick);
 if(i==1 && s==1 && g_axis[i].cia_step==3 && first<0){
    first=cycle;
    g_sim_axis[i].motor[s].cia_state=SIM_CIA_SWITCHED_ON;
 }
#ifdef BASELINE
 if(i==1 && s==1 && first>=0) g_sim_axis[i].motor[s].cia_state=SIM_CIA_SWITCHED_ON;
#endif
}
