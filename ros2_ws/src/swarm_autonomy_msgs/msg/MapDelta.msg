# Incremental occupancy/ESDF update produced by one drone and merged by others.
# Kept compact so the comms middleware can meter bandwidth realistically.
std_msgs/Header header
uint8 drone_id
uint32 seq                       # monotonically increasing per drone
float32 voxel_size               # metres
geometry_msgs/Point origin       # min-corner of the delta block, map frame
uint16[] voxel_indices           # flattened indices into the delta block
int8[]  voxel_log_odds           # occupancy log-odds, quantised to int8
uint32  uncompressed_bytes       # for bandwidth bookkeeping
