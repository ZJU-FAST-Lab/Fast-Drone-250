# the action command for trajectory server.
uint32 ACTION_ADD                  =   1
uint32 ACTION_ABORT                =   2
uint32 ACTION_WARN_START           =   3
uint32 ACTION_WARN_FINAL           =   4
uint32 ACTION_WARN_IMPOSSIBLE      =   5
uint32 action

int32 order
int64 traj_id
float64[] knots
geometry_msgs/Point[] pts
time start_time
float64 time_extra
float64 replan_to_global_time