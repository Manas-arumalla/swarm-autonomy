# A target sighting from one drone's onboard detector, shared for fusion.
std_msgs/Header header
uint8  drone_id
bool   detected
geometry_msgs/Point position     # estimated target position, map frame
geometry_msgs/Vector3 velocity   # estimated target velocity, map frame
float32 confidence               # detector confidence [0,1]
float32 range                    # range from observing drone (m)
