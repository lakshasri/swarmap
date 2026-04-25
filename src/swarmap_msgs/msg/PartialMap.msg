# Compressed occupancy grid chunk shared between neighbouring robots
std_msgs/Header header
string robot_id

# Grid origin offset from world frame origin (in cells)
int32 origin_x
int32 origin_y

# Dimensions of this chunk
int32 width
int32 height

# Metres per cell
float32 resolution

# Cell data: -1 = unknown, 0 = free, 100 = occupied
int8[] data

# Per-cell confidence [0.0, 1.0]; parallel array to data[]
float32[] confidence
