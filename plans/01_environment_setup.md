# Phase 1 — Environment Setup

## Goal
Set up a reproducible development environment with ROS2 Humble, Gazebo Harmonic, and MATLAB so every developer starts from an identical baseline.

---

## Tasks

### 1.1 — OS and ROS2 Humble Installation
**Owner:** All developers  
**Inputs:** Ubuntu 22.04 LTS fresh install  
**Outputs:** Working `ros2 topic list` command

- [ ] Install Ubuntu 22.04 LTS (or verify existing install)
- [ ] Add ROS2 Humble apt repository and install `ros-humble-desktop`
- [ ] Install additional packages:
  ```
  ros-humble-rosbridge-server
  ros-humble-nav2-bringup
  ros-humble-slam-toolbox
  ros-humble-rmw-fastrtps-cpp
  ros-humble-tf2-tools
  ros-humble-robot-state-publisher
  ros-humble-xacro
  python3-colcon-common-extensions
  python3-rosdep
  ```
- [ ] Run `rosdep init && rosdep update`
- [ ] Add `source /opt/ros/humble/setup.bash` to `~/.bashrc`
- [ ] Verify: `ros2 topic list` runs without error

### 1.2 — Gazebo Harmonic Installation
**Inputs:** ROS2 Humble installed  
**Outputs:** Gazebo opens with a basic world

- [ ] Install Gazebo Harmonic via apt (`gz-harmonic`)
- [ ] Install `ros-humble-ros-gz-bridge` and `ros-humble-ros-gz-sim`
- [ ] Verify: `gz sim` launches empty world
- [ ] Verify: ROS2–Gazebo bridge topic echo works for `/clock`

### 1.3 — MATLAB Setup
**Inputs:** MATLAB R2023b licence  
**Outputs:** MATLAB connects to a live ROS2 network

- [ ] Install MATLAB R2023b with:
  - Robotics System Toolbox
  - Navigation Toolbox
  - Parallel Computing Toolbox (for `parfor` benchmarks)
- [ ] Open MATLAB and run `roboticsAddons` — install ROS Toolbox if not present
- [ ] Verify ROS2 connection:
  ```matlab
  ros2 node list   % should show any running nodes
  ```
- [ ] Set `ROS_DOMAIN_ID` in MATLAB to match the ROS2 network

### 1.4 — Node.js and Frontend Tooling
**Inputs:** Ubuntu 22.04  
**Outputs:** `npm run dev` starts a Vite dev server

- [ ] Install Node.js 20 via `nvm` or NodeSource apt repo
- [ ] Verify: `node -v` prints v20.x, `npm -v` prints 10.x

### 1.5 — Workspace Bootstrap Script
**Inputs:** All above tools installed  
**Outputs:** Single script that builds and sources the workspace

- [ ] Create `scripts/bootstrap.sh`:
  ```bash
  #!/usr/bin/env bash
  source /opt/ros/humble/setup.bash
  cd $(git rev-parse --show-toplevel)
  rosdep install --from-paths src --ignore-src -y
  colcon build --symlink-install
  source install/setup.bash
  echo "Swarmap workspace ready."
  ```
- [ ] Make script executable and commit it

### 1.6 — Docker Compose (Optional but Recommended)
**Purpose:** Reproducible environment for CI and new team members

- [ ] Write `docker/Dockerfile` based on `osrf/ros:humble-desktop`
- [ ] Add Gazebo Harmonic layer
- [ ] Write `docker-compose.yml` with services:
  - `ros_core` — master ROS2 daemon
  - `gazebo` — headless simulation
  - `dashboard_backend` — rosbridge server
  - `dashboard_frontend` — Vite dev server (port 5173)
- [ ] Verify: `docker compose up` starts all services with no manual steps

---

## Verification Checklist

- [ ] `ros2 run demo_nodes_cpp talker` publishes `/chatter`
- [ ] `ros2 run demo_nodes_cpp listener` receives messages
- [ ] `gz sim shapes.sdf` opens without errors
- [ ] ROS2–Gazebo clock bridge works: `ros2 topic echo /clock`
- [ ] MATLAB: `ros2 node list` returns results
- [ ] `npm create vite@latest test-app` completes successfully

---

## Notes
- Lock ROS2 and Gazebo versions in `docker/Dockerfile` — do not use `:latest`
- Set `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` in all terminal sessions for consistent DDS behaviour
- MATLAB requires a GUI session — document how to use a VNC or X11 forward for headless servers
