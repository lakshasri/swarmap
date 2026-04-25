# Phase 8 — Browser Dashboard Frontend

## Goal
Build a React dashboard that renders the live swarm map, displays real-time stats, shows the robot communication graph, and lets the user adjust simulation parameters via sliders and buttons.

---

## Tasks

### 8.1 — ROS Bridge Hook (`useRosBridge.ts`)

- [ ] Implement `useRosBridge(url: string)`:
  - Connect to `ws://localhost:9090` via `ROSLIB.Ros`
  - Return `{ ros, connected, error }`
  - Auto-reconnect with exponential backoff on disconnect
- [ ] Implement `useRosTopic<T>(ros, topicName, messageType)`:
  - Subscribe to a ROS2 topic and return latest message as React state
  - Unsubscribe on component unmount
  - Handle message deserialization from rosbridge JSON format
- [ ] Implement `callRosService(ros, serviceName, request)`:
  - Returns a Promise resolving to the service response
  - Used for parameter adjustment calls

### 8.2 — App Layout

- [ ] Implement responsive 3-column layout:
  ```
  ┌──────────────┬────────────────────────┬──────────────┐
  │ Control Panel│     Map Canvas         │ Stats Panel  │
  │ (left 20%)   │     (centre 55%)       │ (right 25%)  │
  │              │                        │              │
  │              ├────────────────────────┤              │
  │              │   Network Graph        │              │
  │              │   (bottom 30%)         │              │
  └──────────────┴────────────────────────┴──────────────┘
  ```
- [ ] Dark theme using CSS variables (dark background, bright robot colours)
- [ ] Responsive down to 1280 px width

### 8.3 — MapCanvas Component (`MapCanvas.tsx`)
Primary visual — a live-updating occupancy grid.

- [ ] Use HTML5 `<canvas>` element (not SVG — performance matters at 250k cells)
- [ ] Implement `renderMap(ctx, mapData, robots)`:
  - Cell colours:
    - `unknown` → `#1a1a2e` (dark blue-grey)
    - `free` → `#e8e8e8` (light grey)
    - `occupied` → `#2d2d2d` (dark)
    - `frontier` → `#ffa500` (orange highlight)
  - Robot markers: filled circle coloured by robot ID (colour palette of 30 distinct colours)
  - Robot label: ID number inside the circle
  - Trajectory trail: last 50 poses as fading dots
- [ ] Subscribe to `/dashboard/map_compressed` and apply delta updates
- [ ] Implement pan and zoom with mouse wheel + drag
- [ ] Show grid scale bar in bottom-left corner (e.g. "5 m")
- [ ] Add a "fit to screen" button to reset zoom/pan

### 8.4 — StatsPanel Component (`StatsPanel.tsx`)

- [ ] Subscribe to `/dashboard/stats` and display:
  - **Coverage** — large circular progress ring showing % explored
  - **Active Robots** — number / total (e.g. "8 / 10")
  - **Failed Robots** — count with red indicator
  - **Map Accuracy** — percentage (0–100%) with colour coding (green >80%, yellow >50%, red <50%)
  - **Mission Time** — elapsed time in mm:ss format
- [ ] Per-robot table:
  - Columns: ID | State | Battery | Cells Mapped | Neighbours
  - Colour-code rows: green=active, orange=returning, red=failed
  - Battery shown as a mini progress bar
- [ ] Coverage over time line chart (last 60 seconds of data using `recharts`)
- [ ] Event log — scrolling list of last 20 events from `/dashboard/events`:
  - e.g. "00:45 — robot_3 FAILED (cascade)", "01:12 — robot_7 reached frontier"

### 8.5 — ControlPanel Component (`ControlPanel.tsx`)

Sliders call `/swarm/set_param` via the ROS service on `mouseup` (not on every tick).

- [ ] **Swarm Size** — integer slider 2–30, calls `/swarm/set_num_robots`
- [ ] **Sensor Range** — float slider 1–15 m, step 0.5
- [ ] **Noise Level** — float slider 0–50%, step 1
- [ ] **Failure Rate** — float slider 0–45%, step 1
- [ ] **Comm Radius** — float slider 0.5–10 m, step 0.5
- [ ] Each slider shows current value live as the user drags
- [ ] Each slider shows "Applying..." spinner while the ROS service call is in flight
- [ ] Mission control buttons:
  - **Start Mission** — publishes to `/swarm/control` (`std_msgs/String` = `"START"`)
  - **Pause** — `"PAUSE"`
  - **Reset** — `"RESET"` (clears all maps, resets all robots to start)
  - **Save Map** — calls `/swarm/save_map` service
- [ ] **Failure Mode** dropdown: Random / Progressive / Cascade
- [ ] **Run Fault Tolerance Demo** button — calls pre-built scenario launch

### 8.6 — NetworkGraph Component (`NetworkGraph.tsx`)

- [ ] Subscribe to `/dashboard/network_topology`
- [ ] Render using Canvas (not a graph library — too slow for real-time):
  - Nodes: circles at real-world (x, y) coordinates scaled to canvas size
  - Edges: lines between communicating robots
  - Edge opacity = signal strength (inverse of distance / comm_radius)
  - Failed robots: ✕ overlay, grey colour
  - Animate new/dropped edges with a 300 ms fade

### 8.7 — Mission Replay Panel

- [ ] Add a **Replay** tab in the dashboard
- [ ] Load a recorded mission JSON file (drag-and-drop or file picker)
- [ ] Implement a timeline scrubber (0 to mission end)
- [ ] Play/pause/speed (0.5× 1× 2× 4×) controls
- [ ] Replay renders the same MapCanvas and StatsPanel using historical data

### 8.8 — Build and Deployment

- [ ] Configure Vite `vite.config.ts`:
  - Proxy `/ws` to `ws://localhost:9090` (avoids CORS issues in dev)
  - Build output to `dist/` — served by a simple Express server in production
- [ ] Add `package.json` scripts:
  - `npm run dev` — development with hot reload
  - `npm run build` — production build
  - `npm run preview` — preview production build
- [ ] Add to `dashboard.launch.py`: start `npm run preview` as a subprocess after build

---

## Verification Checklist

- [ ] Dashboard connects to rosbridge and shows "Connected" indicator
- [ ] MapCanvas renders the live map with correct cell colours
- [ ] Robot markers appear at correct positions and move as robots navigate
- [ ] Coverage ring updates in real time as map grows
- [ ] Dragging the "Failure Rate" slider to 40% and clicking causes robots to fail visibly
- [ ] Swarm size slider: moving to 15 spawns 5 additional robots visible in Gazebo
- [ ] NetworkGraph shows edges appearing/disappearing as robots move in/out of range
- [ ] Event log shows robot failure events within 500 ms of occurrence
- [ ] Dashboard renders at 30+ fps on a mid-range laptop

---

## Notes
- Do not use Three.js or WebGL — the Canvas 2D API is sufficient and much simpler to maintain
- Avoid React re-renders on every WebSocket message — use `useRef` for canvas drawing and only re-render stat components
- Cap the event log at 100 entries to avoid unbounded memory growth
- Test in Chrome and Firefox — Safari has WebSocket quirks with rosbridge
