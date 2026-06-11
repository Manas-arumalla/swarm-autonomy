# Per-window telemetry emitted by the comms middleware for bandwidth plots.
std_msgs/Header header
uint8  drone_id
float32 window_s                 # measurement window length
uint32 msgs_sent
uint32 msgs_delivered            # after range/rate/dropout gating
uint32 msgs_dropped
float32 bytes_per_s              # delivered throughput
float32 bandwidth_cap_bytes_per_s
float32 mean_neighbor_range
