# Periodic position broadcast used by neighbour trackers to detect comm range
std_msgs/Header header
string robot_id
geometry_msgs/Point position

# Maximum communication radius of this robot in metres
float32 comm_radius
