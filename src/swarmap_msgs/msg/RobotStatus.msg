# Periodic health broadcast published at 1 Hz by every active robot
std_msgs/Header header
string robot_id

geometry_msgs/Pose pose

# Battery level: 1.0 = full, 0.0 = empty
float32 battery_level

bool is_active

# Current state string: IDLE | EXPLORING | NAVIGATING | RETURNING | FAILED
string current_state

# Total number of non-unknown cells in this robot's local map
int32 cells_mapped

# IDs of robots currently within communication range
string[] neighbour_ids
