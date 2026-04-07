# Swarmap — ROS2 Topic Naming Reference

All per-robot topics are namespaced under `/{robot_id}/` (e.g., `/robot_0/scan`).
Global swarm topics live under `/swarm/` or `/dashboard/`.

## Per-robot topics

| Topic | Type | Publisher | Subscribers | Notes |
|-------|------|-----------|-------------|-------|
| `/{id}/scan` | `sensor_msgs/LaserScan` | Gazebo bridge | `robot_node` | LiDAR scan at 10 Hz |
| `/{id}/odom` | `nav_msgs/Odometry` | Gazebo bridge | `robot_node` | Wheel odometry |
| `/{id}/cmd_vel` | `geometry_msgs/Twist` | `robot_node` | Gazebo bridge | Velocity command |
| `/{id}/map` | `swarmap_msgs/PartialMap` | `robot_node` | Neighbours' `robot_node`, `map_aggregator_node` | Local occupancy grid chunk at 2 Hz |
| `/{id}/status` | `swarmap_msgs/RobotStatus` | `robot_node` | `swarm_monitor_node`, `map_aggregator_node` | State, battery, neighbour list at 1 Hz |
| `/{id}/frontier_bid` | `swarmap_msgs/FrontierBid` | `robot_node` | Neighbours' `robot_node` | Claim/release messages during auction |
| `/{id}/map_accuracy` | `std_msgs/Float32` | `robot_node` | `map_aggregator_node` | Local map accuracy vs ground truth [0–1] |

## Global swarm topics

| Topic | Type | Publisher | Subscribers | Notes |
|-------|------|-----------|-------------|-------|
| `/swarm/discovery` | `swarmap_msgs/NeighbourDiscovery` | All `robot_node` | All `robot_node` | Broadcast ping at 2 Hz |
| `/swarm/health` | `std_msgs/String` (JSON) | `swarm_monitor_node` | `map_aggregator_node`, dashboard | Active/failed robot list at 1 Hz |
| `/swarm/events` | `std_msgs/String` (JSON) | `failure_injector_node` | `map_aggregator_node` | Failure event stream |
| `/swarm/global_map` | `nav_msgs/OccupancyGrid` | `map_aggregator_node` | RViz, dashboard | Full merged map (Transient Local, 1 Hz) |
| `/swarm/frontier_markers` | `visualization_msgs/MarkerArray` | `map_aggregator_node` | RViz | Frontier cluster visualisation |

## Dashboard topics

| Topic | Type | Publisher | Subscribers | Notes |
|-------|------|-----------|-------------|-------|
| `/dashboard/stats` | `std_msgs/String` (JSON) | `map_aggregator_node` | Frontend via rosbridge | Coverage %, robot table, accuracy at 1 Hz |
| `/dashboard/map_compressed` | `std_msgs/String` (JSON) | `map_aggregator_node` | Frontend via rosbridge | Base64 delta-encoded grid, ≤ 1 MB/s |
| `/dashboard/network_topology` | `std_msgs/String` (JSON) | `swarm_monitor_node` | Frontend via rosbridge | Adjacency list for NetworkGraph |
| `/dashboard/events` | `std_msgs/String` (JSON) | `map_aggregator_node` | Frontend via rosbridge | Human-readable event log |

## Services

| Service | Type | Server | Notes |
|---------|------|--------|-------|
| `/swarm/set_param` | `rcl_interfaces/srv/SetParameters` (wrapped) | `param_service_node` | Broadcasts value to all robot parameter servers |
| `/swarm/set_num_robots` | `std_srvs/srv/SetBool` (wrapped) | `param_service_node` | Spawn or shutdown robots dynamically |

## ROS2 Domain

All nodes run under `ROS_DOMAIN_ID=0` by default. Change via environment variable or `default_params.yaml` to isolate multiple simultaneous experiments.
