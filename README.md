## Swarmap
A decentralised swarm robotics system where autonomous robots collaborate to explore and map unknown environments without any central coordinator.

## What is Swarmap?

Swarmap enables a fleet of mS

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
