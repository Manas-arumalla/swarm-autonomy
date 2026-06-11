# A drone's pose estimate shared with neighbours over the comms layer.
std_msgs/Header header
uint8 drone_id
geometry_msgs/Pose pose          # in the shared map frame
geometry_msgs/Twist twist        # body-frame linear/angular velocity
float32 position_covariance_trace # crude confidence proxy from the estimator
