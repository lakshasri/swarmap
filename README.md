# Swarmap — Decentralised Robot Exploration and Mapping

A decentralised swarm robotics system where autonomous robots collaborate to explore and map unknown environments without any central coordinator.

## What is Swarmap?

Swarmap enables a fleet of mobile robots to work together as a swarm, each exploring different parts of an unknown environment simultaneously. The robots share partial maps only with their immediate neighbours, never requiring a central server or master controller. Through distributed frontier auctions, each robot autonomously decides where to explore next, avoiding wasteful duplication of effort.

The system is designed to be **robust to failures** — even when nearly half the robots fail mid-mission, the remaining swarm adapts and continues mapping. The live browser dashboard lets you watch the map emerge in real-time and adjust swarm parameters dynamically.

## Why It Matters

- **Decentralised**: No single point of failure. Communication is peer-to-peer.
- **Autonomous**: Robots make local decisions without a master control loop.
- **Scalable**: Add more robots and coverage increases proportionally (until overcrowded).
- **Fault-tolerant**: Surviving robots restructure and keep working when neighbours fail.
- **Real-time visibility**: Browser dashboard shows live exploration progress and network topology.

## Core Technologies

- **ROS2 Humble** — inter-robot communication and middleware
- **Gazebo Harmonic** — physics simulation and sensor simulation
- **React + Canvas** — browser-based live dashboard
- **MATLAB** — algorithm prototyping and benchmark analysis
- **C++17** — high-performance robot nodes

## Quick Start

```bash
git clone https://github.com/lakshasri/Swarmap.git
cd Swarmap
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch swarmap_bringup simulation.launch.py num_robots:=10
```

Open http://localhost:5173 in your browser to see the dashboard.

---

**Status**: Early development — core infrastructure in progress
