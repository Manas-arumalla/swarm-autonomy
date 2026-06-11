# Role assigned to a drone by the CBBA layer for the pursuit phase.
std_msgs/Header header
uint8 drone_id
uint8 role                       # see constants below
uint32 assigned_task_id

uint8 ROLE_IDLE=0
uint8 ROLE_SCOUT=1               # explore / search for the target
uint8 ROLE_BLOCKER=2            # cut off an escape corridor
uint8 ROLE_INTERCEPTOR=3       # close on the target's predicted position
