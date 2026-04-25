# Bid published by a robot to claim a frontier cell in the distributed auction
std_msgs/Header header
string robot_id

# World-frame centroid of the frontier cluster being bid on
geometry_msgs/Point frontier_centroid

# Bid score: lower value = higher priority (energy-weighted cost-to-go + inverse cluster size)
float32 bid_score

# Battery level at time of bid [0.0, 1.0] — informational, not used in auction logic
float32 battery_level

# true = claiming this frontier, false = releasing it
bool claim
