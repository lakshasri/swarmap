# Swarmap — Troubleshooting Guide

## 1. ROS_DOMAIN_ID mismatch — nodes can't see each other

**Symptom:** `ros2 node list` or `ros2 topic list` shows fewer nodes than
expected; topics produce no messages.

**Fixes:**
1. Check the domain ID in every terminal: `echo $ROS_DOMAIN_ID`
2. All terminals must share the same ID. Set it: `export ROS_DOMAIN_ID=0`
3. Persist it: `echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc`

---

## 2. colcon build failures

**Symptom:** `colcon build` exits non-zero; errors mention missing packages.

**Fixes:**
1. Source ROS2 first: `source /opt/ros/humble/setup.bash`
2. Run rosdep: `rosdep install --from-paths src --ignore-src -r -y`
3. Clean and rebuild: `rm -rf build/ install/ log/ && colcon build`
4. Check C++17 support: `g++ --version` — must be ≥ 9.

---

## 3. Robots spawn but don't move (all stay IDLE)

**Symptom:** RobotStatus always shows `current_state: IDLE`; no `cmd_vel`.

**Fixes:**
1. Confirm LiDAR data arrives: `ros2 topic echo /robot_0/scan --once`
2. Confirm odometry arrives: `ros2 topic echo /robot_0/odom --once`
3. Increase map resolution so the grid isn't immediately fully known:
   `ros2 launch swarmap_bringup simulation.launch.py map_resolution:=0.2`
4. `world_sim_node` must be running: `ros2 node list | grep world_sim`

---

## 4. RViz shows "No transform" or empty map

**Symptom:** RViz reports `Fixed Frame [map] does not exist` or the
`OccupancyGrids` display stays grey.

**Fixes:**
1. The fixed frame must be `map`. World sim broadcasts `map → robot_i/odom`
   as a static transform, and `robot_node` broadcasts `robot_i/odom →
   robot_i/base_link`.
2. Confirm the static TFs are present: `ros2 run tf2_ros tf2_echo map robot_0/odom`
3. Confirm the global map publishes:
   `ros2 topic hz /swarm/global_map` should show ~1 Hz.
4. If only the ground truth shows up but no exploration map, the aggregator
   may not have started — check `ros2 node list | grep aggregator`.

---

## 5. Failure injector has no effect

**Symptom:** Robots never fail even with `failure_rate > 0`.

**Fixes:**
1. Confirm the node is running: `ros2 node list | grep failure_injector`
2. Check the rate parameter: `ros2 param get /failure_injector_node failure_rate`
3. Set it live: `ros2 param set /failure_injector_node failure_rate 0.1`
4. Monitor events: `ros2 topic echo /swarm/events`

---

## 6. MATLAB can't find ros2 / ros2node errors

**Symptom:** `Undefined function 'ros2node'` or `ros2subscriber` in MATLAB.

**Fixes:**
1. Install the ROS Toolbox: *Home → Add-Ons → ROS Toolbox*.
2. Source ROS2 **before** launching MATLAB so `PYTHONPATH` and `LD_LIBRARY_PATH`
   are set: `source /opt/ros/humble/setup.bash && matlab &`.
3. Verify: `>> ros2 topic list` from the MATLAB command window.
4. Custom messages: rebuild with `ros2genmsg('<path-to-swarmap_msgs>')`.
5. Headless: launch MATLAB with `-nodisplay`.

---

## 7. World sim runs but robots collide with walls non-stop

**Symptom:** Multiple robots stuck against perimeter; no exploration progress.

**Fixes:**
1. Lower `num_robots` so they aren't packed against each other.
2. Increase `linear_speed_cap` / `angular_speed_cap` if mapping needs to
   reach far frontiers faster (params on `world_sim_node`).
3. The procedural world is generated from a fixed seed; if the layout is
   unfortunate, change the seed in `build_world(...)` inside
   `src/swarmap_core/src/world_sim_node.py`.
