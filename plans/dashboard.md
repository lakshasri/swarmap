# SWARMAP Dashboard Guide

The browser dashboard at **http://localhost:5173/** is the primary way to monitor and control a running SWARMAP simulation. It connects to ROS2 over a WebSocket bridge (`rosbridge_websocket` on port 9090) and exchanges live data with the swarm.

To launch everything together:

```bash
ros2 launch swarmap_bringup simulation.launch.py num_robots:=5 dashboard:=true rviz:=false
```

This brings up the world simulator, the robots, the map aggregator, the failure injector, the dashboard bridge node, the rosbridge server, and the Vite dev server in one command.

---

## Layout

The dashboard is divided into three vertical regions:

```
┌─────────────────────────────────────────────────────────────┐
│  SWARMAP        [ Live | Replay ]        rosbridge connected│
├──────────────┬──────────────────────────┬───────────────────┤
│              │                          │                   │
│   CONTROL    │      MAP CANVAS          │     STATS         │
│   PANEL      │      (occupancy grid)    │     PANEL         │
│              │                          │                   │
│              ├──────────────────────────┤                   │
│              │      NETWORK GRAPH       │                   │
│              │      (neighbour links)   │                   │
└──────────────┴──────────────────────────┴───────────────────┘
```

The **header** has the Live/Replay tab switcher and a connection indicator (green = rosbridge connected, red = disconnected).

---

## Centre: Map Canvas

This is the merged global occupancy grid published by `map_aggregator_node` on `/swarm/global_map`. Each robot's local map is fused into this single view through confidence-weighted averaging.

- **Light grey** = free space that has been observed.
- **Dark grey / black** = occupied (walls and obstacles).
- **Background** = unknown / unmapped.
- **Coloured dots** with labels (R0, R1, R2 ...) = current robot positions.

The **CONF OFF / CONF ON** button in the top-right of the map toggles a confidence heatmap overlay. When CONF is ON, the map is tinted by how many robots have observed each cell — more visits means brighter colour.

You can pan the map by clicking and dragging, and zoom with the mouse wheel.

---

## Centre Bottom: Network Graph

This shows the live communication topology, published by `swarm_monitor_node` on `/dashboard/network_topology`.

- Each circle is a robot, positioned according to its world coordinates.
- A line between two circles means those two robots are within communication range of each other and are currently exchanging map data.
- Failed robots appear in a different colour (or disappear if the layout doesn't include them).

This view is useful for spotting when the swarm has fragmented into disconnected groups.

---

## Right: Stats Panel

This panel reads from `/dashboard/stats` (published by `map_aggregator_node`) and `/swarm/health` (published by `swarm_monitor_node`).

- **COVERAGE** -- a ring chart showing the percentage of the world that has been mapped, plus the active and failed robot counts.
- **COVERAGE OVER TIME** -- a small line chart showing how coverage has grown over the last minute or so.
- **ROBOTS** -- a list of every robot, each with:
  - Its ID (`robot_0`, `robot_1`, ...)
  - Its current state chip: **EXPLORING**, **NAVIGATING**, **RETURNING**, **IDLE**, or **FAILED**
  - A horizontal battery bar (full = green, draining = yellow/red).

---

## Left: Control Panel

This is where the interactive controls live. Every action here publishes a message that `dashboard_bridge_node.py` translates into a real ROS2 operation.

### ROBOT STATUS

- **robots active** -- a count of robots currently in any non-failed state.
- **Select robot to kill / Kill button** -- pick a robot from the dropdown and click **Kill**. The bridge node calls the lifecycle `change_state` service on `/robot_i/robot_node` with a SHUTDOWN transition. The robot disappears from the swarm cleanly, and the surviving robots detect the loss through their neighbour heartbeat timeout. Useful for testing fault tolerance manually.

### LIVE PARAMETERS

- **Noise level** (0 -- 1) -- adds Gaussian noise to the simulated laser scans. Higher values produce a noisier map. Routed to `world_sim_node` via SetParameters.
- **Failure rate** (0 -- 0.45 / min) -- expected failures per minute for the random/progressive/cascade failure injector. Set this above zero and the failure injector will start stochastically removing robots. Routed to `failure_injector_node`.

Slider changes are applied when you release the slider (mouse-up).

### FAILURE MODE

A dropdown that selects how the failure injector picks its next victim:

- **random** -- uniform random pick from currently active robots.
- **progressive** -- always fails the first active robot in order (`robot_0`, then `robot_1`, ...).
- **cascade** -- like progressive but timed to make failures cluster.

The selection is sent to `failure_injector_node` via SetParameters.

The **Inject Failure Now** button publishes to `/swarm/inject_failure_now`, which tells the bridge node to immediately kill a random active robot. Useful when you want a failure on demand instead of waiting for the stochastic process.

---

## Replay Tab

Switching to the **Replay** tab opens `ReplayPanel`, which lets you load a previously recorded mission JSON file (drag-and-drop or file picker). It provides a timeline scrubber, play/pause controls, and speed adjustment (0.5x to 4x). This is a passive viewer -- it does not affect any running simulation.

---

## Behind the Scenes: Topics and Services

The dashboard talks to the swarm only through standard ROS2 messages, so you can verify everything from the command line.

| Direction | Topic | Type | Purpose |
|-----------|-------|------|---------|
| ROS2 → UI | `/swarm/global_map` | `nav_msgs/OccupancyGrid` | merged map for the canvas |
| ROS2 → UI | `/dashboard/confidence_map` | `nav_msgs/OccupancyGrid` | confidence heatmap |
| ROS2 → UI | `/dashboard/stats` | `std_msgs/String` (JSON) | per-robot state, coverage |
| ROS2 → UI | `/swarm/health` | `std_msgs/String` (JSON) | active/failed counts |
| ROS2 → UI | `/dashboard/network_topology` | `std_msgs/String` (JSON) | nodes + edges for graph |
| UI → ROS2 | `/swarm/kill_robot` | `std_msgs/String` | robot id to shut down |
| UI → ROS2 | `/swarm/set_param_request` | `std_msgs/String` (JSON) | `{"param":"...","value":...}` |
| UI → ROS2 | `/swarm/inject_failure_now` | `std_msgs/String` | trigger immediate failure |

All of the UI → ROS2 topics are consumed by `dashboard_bridge_node.py`, which converts each message into the appropriate lifecycle call or `SetParameters` service request to the relevant nodes.

---

## Things That Are Not Wired Up

A few possible features intentionally do not exist in the dashboard, because they would require a heavier orchestration layer than ROS2 lifecycle alone provides:

- **Spawning new robots at runtime.** ROS2 nodes are launched as separate OS processes by the launch system; spawning a new one mid-mission would require a process supervisor outside of rosbridge. The number of robots is fixed at launch time via the `num_robots` argument.
- **Pausing / resuming exploration.** The robots always run their exploration tick. To "pause", stop the simulation and relaunch.
- **Tuning sensor_range / comm_radius live.** These values are cached in the robot node at startup and used to size internal data structures, so changing them mid-mission would not have a clean effect. Set them at launch time instead.

If a button or slider for one of these existed in an earlier version, it has been removed so the UI only exposes things that genuinely work.

---

## Troubleshooting

- **"rosbridge connected" stays red.** The rosbridge server is not running. Make sure the launch was started with `dashboard:=true`, or start it manually with `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`.
- **Robot dropdown is empty.** No `/dashboard/stats` messages have arrived yet. The map aggregator publishes once a second; give it a few seconds after launch.
- **Kill button does nothing.** Check the dashboard bridge node's log -- if it says "Lifecycle service not ready for robot_X", the target robot may already be down or never came up.
- **Vite is on a different port.** If port 5173 was already in use, Vite will fall back to 5174 (or higher). The launch output shows the actual URL.
