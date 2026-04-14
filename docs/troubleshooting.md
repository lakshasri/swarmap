# Swarmap — Troubleshooting Guide

## 1. rosbridge WebSocket connection refused

**Symptom:** Dashboard shows "Disconnected" and browser console shows `WebSocket connection to ws://localhost:9090 failed`.

**Fixes:**
1. Confirm rosbridge is running: `ros2 node list | grep rosbridge`
2. Check the port: `ss -tlnp | grep 9090`
3. Restart rosbridge: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090`
4. In Docker, ensure `network_mode: host` is set so port 9090 is reachable from the host browser.
5. Firewall check: `sudo ufw status` — port 9090 must be allowed.

---

## 2. ROS_DOMAIN_ID mismatch — nodes can't see each other

**Symptom:** `ros2 node list` or `ros2 topic list` shows fewer nodes than expected; topics produce no messages.

**Fixes:**
1. Check the domain ID in every terminal: `echo $ROS_DOMAIN_ID`
2. All terminals (and Docker containers) must share the same ID. Set it: `export ROS_DOMAIN_ID=0`
3. Add to `~/.bashrc` to persist: `echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc`
4. In Docker Compose this is set via the `environment` block — see `docker-compose.yml`.

---

## 3. Gazebo crashes or freezes at startup

**Symptom:** `gzserver` process exits immediately or hangs; robots never appear in the scene.

**Fixes:**
1. Check GPU drivers: `nvidia-smi` (NVIDIA) or `glxinfo | grep renderer` (Mesa).
2. Run headless: add `--headless-rendering` to the `gzserver` arguments in `simulation.launch.py`.
3. Reduce robot count: `ros2 launch swarmap_bringup simulation.launch.py num_robots:=3`
4. Confirm the world file loads: `gz sim src/swarmap_bringup/worlds/warehouse.sdf`
5. Missing Gazebo model path: `export GZ_SIM_RESOURCE_PATH=/path/to/swarmap_bringup/urdf`

---

## 4. colcon build failures

**Symptom:** `colcon build` exits non-zero; errors mention missing packages.

**Fixes:**
1. Source ROS2 first: `source /opt/ros/humble/setup.bash`
2. Run rosdep: `rosdep install --from-paths src --ignore-src -r -y`
3. Clean and rebuild: `rm -rf build/ install/ log/ && colcon build`
4. Check C++17 support: `g++ --version` — must be ≥ 9.

---

## 5. robots spawn but don't move (all stay IDLE)

**Symptom:** RobotStatus always shows `current_state: IDLE`; no cmd_vel messages.

**Fixes:**
1. Confirm LiDAR data arrives: `ros2 topic echo /robot_0/scan --once`
2. Confirm odometry arrives: `ros2 topic echo /robot_0/odom --once`
3. Increase map resolution or map size so the grid isn't immediately fully known:
   `ros2 launch swarmap_bringup simulation.launch.py map_resolution:=0.2`
4. Set `use_sim_time:=true` everywhere — missing sim time causes timer starvation.

---

## 6. Dashboard shows blank map / no data

**Symptom:** MapCanvas is empty; StatsPanel shows 0 robots.

**Fixes:**
1. Confirm `map_aggregator_node` is running: `ros2 node list | grep aggregator`
2. Check `/dashboard/stats` is publishing: `ros2 topic echo /dashboard/stats --once`
3. Verify rosbridge is forwarding: open browser devtools → Network → WS frames.
4. Check `VITE_WS_URL` in the dashboard `.env` or `vite.config.ts` matches the rosbridge port.

---

## 7. npm install / build fails for dashboard

**Symptom:** `npm install` exits with peer dependency errors or `npm run build` fails.

**Fixes:**
1. Ensure Node.js ≥ 20: `node --version`
2. Delete lockfile and retry: `rm package-lock.json && npm install`
3. Inside Docker, run: `docker-compose run --rm frontend bash -c "npm install"`

---

## 8. Failure injector has no effect

**Symptom:** Robots never fail even with `failure_rate > 0`.

**Fixes:**
1. Confirm the node is running: `ros2 node list | grep failure_injector`
2. Check the rate parameter: `ros2 param get /failure_injector_node failure_rate`
3. Set it live: `ros2 param set /failure_injector_node failure_rate 0.1`
4. Monitor events: `ros2 topic echo /swarm/events`

---

## 9. MATLAB can't find ros2 / ros2node errors

**Symptom:** `Undefined function 'ros2node'` or `ros2subscriber` in MATLAB.

**Fixes:**
1. Install the ROS Toolbox: *Home → Add-Ons → ROS Toolbox*.
2. Source ROS2 **before** launching MATLAB so `PYTHONPATH` and `LD_LIBRARY_PATH` are set:
   `source /opt/ros/humble/setup.bash && matlab &`.
3. Verify: `>> ros2 topic list` from the MATLAB command window.
4. Custom messages: rebuild with `ros2genmsg('<path-to-swarmap_msgs>')`.
5. If running headless on a server: start MATLAB with `-nodisplay` and
   set `set(gcf,'Visible','off')` before any plotting.

---

## 10. Integration tests time out

**Symptom:** `colcon test --packages-select swarmap_bringup` fails because
Gazebo takes too long to bring up N robots in CI.

**Fixes:**
1. Run locally — the launch tests need ~4 GB RAM and a real Gazebo.
2. Enable headless rendering: `export LIBGL_ALWAYS_SOFTWARE=1`.
3. Override the default timeout in `CMakeLists.txt` (`add_launch_test(... TIMEOUT 300)`).
4. Leave `SWARMAP_RUN_INTEGRATION` unset in GitHub Actions — only C++ unit
   tests run in CI by default.
