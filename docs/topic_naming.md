# Swarmap — ROS2 Topic Naming Reference

All per-robot topics are namespaced under `/{robot_id}/` (e.g., `/robot_0/scan`).
Global swarm topics live under `/swarm/`. World-truth topics under `/world/`.

## Per-robot topics

| Topic | Type | Publisher | Subscribers | Notes |
|-------|------|-----------|-------------|-------|
| `/{id}/scan` | `sensor_msgs/LaserScan` | `world_sim_node` | `robot_node` | 180-ray ray-cast against ground truth at 10 Hz |
| `/{id}/odom` | `nav_msgs/Odometry` | `world_sim_node` | `robot_node` | Pose integrated from `/cmd_vel` at 10 Hz |
| `/{id}/cmd_vel` | `geometry_msgs/Twist` | `robot_node` | `world_sim_node` | Linear + angular velocity command |
| `/{id}/map` | `swarmap_msgs/PartialMap` | `robot_node` | Neighbours' `robot_node`, `map_aggregator_node` | Local occupancy grid delta at 2 Hz |
| `/{id}/status` | `swarmap_msgs/RobotStatus` | `robot_node` | `swarm_monitor_node`, `map_aggregator_node` | State, battery, neighbours at 1 Hz |
| `/{id}/frontier_bid` | `swarmap_msgs/FrontierBid` | `robot_node` | Neighbours' `robot_node` | Auction claim/release |
| `/{id}/map_accuracy` | `std_msgs/Float32` | `robot_node` | `map_aggregator_node` | Local map vs ground truth [0–1] |

## Global swarm topics

| Topic | Type | Publisher | Subscribers | Notes |
|-------|------|-----------|-------------|-------|
| `/swarm/discovery` | `swarmap_msgs/NeighbourDiscovery` | All `robot_node` | All `robot_node` | Broadcast ping at 2 Hz |
| `/swarm/health` | `std_msgs/String` (JSON) | `swarm_monitor_node` | RViz overlays, ops | Active/failed robots at 1 Hz |
| `/swarm/events` | `std_msgs/String` (JSON) | `failure_injector_node` | `map_aggregator_node`, ops | Failure event stream |
| `/swarm/global_map` | `nav_msgs/OccupancyGrid` | `map_aggregator_node` | RViz | Merged exploration map (Transient Local, 1 Hz) |
| `/swarm/frontier_markers` | `visualization_msgs/MarkerArray` | `map_aggregator_node` | RViz | Frontier cluster visualisation |

## World-truth topics (debug / RViz backdrop)

| Topic | Type | Publisher | Subscribers | Notes |
|-------|------|-----------|-------------|-------|
| `/world/ground_truth` | `nav_msgs/OccupancyGrid` | `world_sim_node` | RViz, MATLAB bridge | Static ground-truth grid, republished every 2 s |

## TF tree

```
map
 ├── robot_0/odom  (static, identity, broadcast by world_sim_node)
 │    └── robot_0/base_link  (dynamic, broadcast by world_sim_node + robot_node)
 ├── robot_1/odom
 │    └── robot_1/base_link
 ...
```

## ROS2 Domain

All nodes run under `ROS_DOMAIN_ID=0` by default. Change via the env variable
to isolate multiple simultaneous experiments on the same network.
