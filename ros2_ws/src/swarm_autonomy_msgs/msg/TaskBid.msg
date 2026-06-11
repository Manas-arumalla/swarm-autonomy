# A single CBBA bid: drone_id claims task_id at a winning score with a timestamp.
# CBBA convergence relies on (winning_score, winning_agent, timestamp) consensus.
std_msgs/Header header
uint8  drone_id
uint32 task_id
float32 winning_score            # bidder's score for the task (higher wins)
uint8  winning_agent             # agent currently believed to win the task
float64 timestamp                # last update time for this task entry
