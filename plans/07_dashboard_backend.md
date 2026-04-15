# Phase 7 — Dashboard Backend (rosbridge + Aggregation)

## Goal
Bridge ROS2 data to the browser via WebSocket. The backend must aggregate swarm-wide data into clean JSON streams the frontend can render without knowing anything about ROS2.

---

## Tasks

### 7.1 — rosbridge_server Setup

- [ ] Install `ros-humble-rosbridge-server` (already listed in Phase 1)
- [ ] Create `config/rosbridge_params.yaml`:
  ```yaml
  rosbridge_websocket:
    ros__parameters:
      port: 9090
      address: "0.0.0.0"
      retry_startup_delay: 5.0
      fragment_timeout: 600
      delay_between_messages: 0
      max_message_size: 10000000    # 10 MB for large maps
      unregister_timeout: 10.0
      use_compression: true
  ```
- [ ] Add rosbridge launch to `dashboard.launch.py`
- [ ] Verify: browser-side `ROSLIB.Ros({ url: 'ws://localhost:9090' })` connects

### 7.2 — Dashboard Aggregation Node (`aggregator_node.cpp`)
Converts raw ROS2 swarm data into simplified, browser-friendly topics.

- [ ] Subscribe to:
  - All `/robot_*/status` (wildcard via regex over topic list)
  - `/swarm/health`
  - `/swarm/global_map`
  - `/swarm/events`
  - `/swarm/coverage`
- [ ] Compute and publish `/dashboard/stats` (`std_msgs/String` JSON) at 2 Hz:
  ```json
  {
    "active_robots": 8,
    "total_robots": 10,
    "failed_robots": 2,
    "coverage_pct": 73.4,
    "map_accuracy": 0.91,
    "mission_elapsed_s": 142,
    "robots": [
      {
        "id": "robot_0",
        "x": 3.2, "y": 1.1, "theta": 0.45,
        "state": "EXPLORING",
        "battery": 0.72,
        "cells_mapped": 1240,
        "neighbours": ["robot_2", "robot_5"]
      }
    ]
  }
  ```
- [ ] Publish `/dashboard/map` (`nav_msgs/OccupancyGrid`) at 1 Hz — the global merged map
- [ ] Publish `/dashboard/events` (`std_msgs/String` JSON array) at 1 Hz — last 50 events

### 7.3 — Map Serialisation for Browser
Raw `OccupancyGrid` is large — serialise efficiently for WebSocket.

- [ ] Implement map compressor in aggregator:
  - Convert `OccupancyGrid` to a flat `uint8` array with values `0=unknown, 1=free, 2=occupied`
  - Base64-encode and send as JSON field `map_data`
  - Include `width`, `height`, `resolution`, `origin_x`, `origin_y`
- [ ] Only send map delta (changed cells) on subsequent publishes:
  - Maintain previous map state
  - Send `{full: false, changed_cells: [{idx, val}, ...]}` when delta < 30% of full map
  - Send `{full: true, map_data: "..."}` otherwise
- [ ] Publish on `/dashboard/map_compressed` (`std_msgs/String` JSON)

### 7.4 — Parameter Control Service
Allows the dashboard to adjust swarm parameters at runtime.

- [ ] Create ROS2 service `/swarm/set_param`:
  - Request: `{param_name: string, value: float64}`
  - Response: `{success: bool, message: string}`
- [ ] Supported parameters:
  - `sensor_range` — broadcast to all robot nodes via parameter event
  - `comm_radius` — broadcast to all neighbour trackers
  - `failure_rate` — update failure injector
  - `num_robots` — spawn or despawn robots dynamically
- [ ] Implement dynamic robot spawning service `/swarm/set_num_robots`:
  - Spawning: launch a new `robot_node` with the next available ID
  - Despawning: gracefully shut down the highest-ID active robot (SHUTDOWN lifecycle)
- [ ] Expose both services through rosbridge so the frontend can call them via ROSLIB

### 7.5 — Network Topology Publisher
Powers the comm-graph visualisation in the dashboard.

- [ ] Subscribe to all `/robot_*/status` to collect `neighbour_ids` arrays
- [ ] Build adjacency list: `{robot_id: [neighbour_ids]}`
- [ ] Publish as JSON on `/dashboard/network_topology` (`std_msgs/String`) at 1 Hz:
  ```json
  {
    "nodes": [{"id": "robot_0", "x": 3.2, "y": 1.1, "active": true}, ...],
    "edges": [["robot_0", "robot_2"], ["robot_0", "robot_5"], ...]
  }
  ```

### 7.6 — Ground Truth Map Provider (Simulation Mode)

- [ ] Load ground truth map from `worlds/warehouse.pgm` on startup
- [ ] Publish on `/ground_truth/map` (`nav_msgs/OccupancyGrid`) at 0.1 Hz
- [ ] The aggregator uses this to compute `map_accuracy` in `/dashboard/stats`
- [ ] In hardware mode (no Gazebo), skip this — accuracy metric is unavailable

---

## Verification Checklist

- [ ] rosbridge WebSocket accepts connections on port 9090
- [ ] `/dashboard/stats` updates at 2 Hz and contains all robot states
- [ ] `/dashboard/map_compressed` correctly reconstructs to a valid map in the browser
- [ ] Calling `/swarm/set_param` with `failure_rate=0.3` updates the failure injector
- [ ] Spawning a robot via `/swarm/set_num_robots` adds a new robot to Gazebo
- [ ] `/dashboard/network_topology` correctly reflects which robots are in range of each other
- [ ] Dashboard receives data within 200 ms of a robot state change

---

## Notes
- rosbridge can become a bottleneck with many robots — use `use_compression: true` and the delta map scheme to reduce WebSocket traffic
- The aggregator node is the only node allowed to subscribe to ALL robot topics — individual robots must never aggregate on behalf of others
- Set rosbridge `max_message_size` to at least 10 MB — a 50×50 m map at 0.1 m/cell is 250,000 cells
