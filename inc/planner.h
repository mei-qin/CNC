#ifndef PLANNER_H
#define PLANNER_H

#include <math.h>
#include "axis_cfg.h"
#include "global_def.h"

double calculate_junction_speed(TrajectorySegment_t *prev,TrajectorySegment_t *curr);
// @Context: Non-RealTime Background Thread
// force_flush=0: normal look-ahead planning (from api_push_trajectory)
// force_flush=1: force-release all segments (from starvation watchdog)
void planner_recalculate(int force_flush);

// @Context: callee MUST hold queue_spinlock. Does NOT acquire/release it.
//           Used by api_flush_planner after spin-wait lock acquisition.
void planner_recalculate_locked(int force_flush);



#endif // PLANNER_H
