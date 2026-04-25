# Novel Features for SWARMAP

Six research-grade ideas that extend the current system in genuinely new directions.
Each has a difficulty rating, which files to touch, and a concrete implementation sketch.

---

## 1. Energy-Aware Frontier Bidding

**What it is**
The current auction assigns frontiers purely by distance. Add battery level and estimated travel cost to the bid formula so low-battery robots never win far frontiers and can return to a charging zone before dying.

**Why it's novel**
Most swarm papers treat battery as a hard cutoff. Weighting it continuously in the auction produces emergent behaviour — robots naturally self-sort by energy without a central planner.

**Bid formula**
```
bid = 1 / (distance + λ * (1 - battery) * distance_to_charger)
```
`λ` is a tunable urgency weight. When battery is high `λ` has no effect. As battery drops, the penalty term grows and the robot's bids become uncompetitive on distant frontiers.

**Files to touch**
- `src/swarmap_core/src/frontier_explorer.cpp` — modify `computeBid()` to pull `battery_level` from the latest `RobotStatus` msg
- `src/swarmap_msgs/msg/FrontierBid.msg` — add `float32 battery_level`
- `src/swarmap_bringup/config/default_params.yaml` — add `bid_battery_weight: 2.0`
- `ControlPanel.tsx` — add a `bid_battery_weight` slider

**Difficulty** ★★☆☆☆

---

## 2. Multi-Hop Communication Relay

**What it is**
Robots currently only share maps with direct neighbours (within `comm_radius`). Add a relay layer so Robot A can forward Robot C's partial map through Robot B when A and C are out of direct range. Builds a self-healing mesh rather than isolated point-to-point links.

**Why it's novel**
Distributed relay in swarms is an active research area. The key challenge is preventing relay storms — you need a TTL counter on each forwarded message.

**Message flow**
```
robot_0 publishes /swarm/relay  { origin: "robot_0", ttl: 3, payload: <PartialMap> }
robot_1 (in range of 0) receives it, decrements ttl, re-publishes if ttl > 0
robot_2 (out of range of 0 but in range of 1) now receives robot_0's map
```

**Files to touch**
- `src/swarmap_msgs/msg/RelayedMap.msg` — new message: `string origin_id`, `uint8 ttl`, `PartialMap payload`
- `src/swarmap_core/src/robot_node.cpp` — subscribe to `/swarm/relay`, forward if ttl > 0 and not seen before (cache origin+seq to deduplicate)
- `src/swarmap_core/include/swarmap_core/neighbour_tracker.hpp` — add `seen_relay_ids` dedup cache
- `NetworkGraph.tsx` — draw relayed edges differently (dashed line) to show indirect links

**Difficulty** ★★★☆☆

---

## 3. Predictive Rescue Routing

**What it is**
A background monitor watches each robot's battery drain rate and current distance from the start zone. If it predicts a robot will run out before returning, it broadcasts a rescue signal and re-routes the nearest high-battery robot to shadow it and take over its frontier.

**Why it's novel**
This is proactive fault tolerance, not reactive. The swarm recovers before the failure happens, not after — a meaningful distinction for coverage continuity guarantees.

**Prediction model**
```
time_to_empty   = battery / drain_rate
time_to_return  = distance_to_origin / avg_speed
if time_to_return > 0.85 * time_to_empty:  → trigger rescue
```

**Files to touch**
- `src/swarmap_core/src/swarm_monitor_node.py` — add `_check_rescue()` called every 2s, publish `/swarm/rescue_needed` (String, robot_id)
- `src/swarmap_core/src/robot_node.cpp` — subscribe to `/swarm/rescue_needed`, nearest robot volunteers by publishing to `/swarm/rescue_ack`
- `src/swarmap_dashboard/src/components/MapCanvas.tsx` — draw a rescue path line between rescuer and at-risk robot in orange
- `src/swarmap_dashboard/src/components/StatsPanel.tsx` — add a `AT RISK` badge on the robot row

**Difficulty** ★★★☆☆

---

## 4. Map Confidence Heatmap

**What it is**
The occupancy grid currently stores only log-odds (free/occupied/unknown). Add a second layer that counts how many times each cell has been observed. Cells seen by 3+ robots are high-confidence; cells seen once are low-confidence. Visualise this as a colour overlay on the map canvas.

**Why it's novel**
Standard SLAM papers report coverage percentage. Confidence-weighted coverage is a stricter and more honest metric — a cell seen by one noisy sensor is not the same as a cell confirmed by three robots independently.

**Implementation**
- Each `PartialMap` already carries cell data. Add a `uint16[]` visit_counts field.
- During merge, add visit counts per cell (not overwrite).
- Dashboard reads the second array and renders a blue-to-green overlay: blue = 1 visit, green = 3+ visits, transparent = unknown.

**Files to touch**
- `src/swarmap_msgs/msg/PartialMap.msg` — add `uint16[] visit_counts`
- `src/swarmap_core/include/swarmap_core/occupancy_grid.hpp` — add `visit_count_` grid
- `src/swarmap_core/src/occupancy_grid.cpp` — increment count on update
- `src/swarmap_core/src/map_aggregator_node.cpp` — include `visit_counts` in global map publish
- `src/swarmap_dashboard/src/components/MapCanvas.tsx` — add confidence overlay toggle button
- `ControlPanel.tsx` — toggle switch for confidence overlay

**Difficulty** ★★☆☆☆

---

## 5. Semantic Zone Classification

**What it is**
After a region is mapped, analyse its shape to classify it: narrow corridor (width < 2m), open room (area > 10m², convex), junction (3+ corridors meeting), or dead end. Paint these zones onto the map and let the dashboard filter frontiers by zone type.

**Why it's novel**
Purely geometric SLAM gives you occupancy. Semantic labelling turns it into a floor plan that a human can reason about — and lets robots make smarter exploration decisions (e.g. avoid sending 3 robots down the same dead-end corridor).

**Classification logic**
```
For each connected free region:
  width = min bounding box dimension
  area  = cell count * resolution²
  exits = number of corridor openings leading out

  if exits == 1 and area < 4:    → DEAD_END
  elif width < 1.5:              → CORRIDOR
  elif exits >= 3:               → JUNCTION
  else:                          → ROOM
```

**Files to touch**
- `src/swarmap_core/src/map_aggregator_node.cpp` — add `classifyZones()` run after each merge cycle, publish `/dashboard/zones` as JSON
- `src/swarmap_dashboard/src/components/MapCanvas.tsx` — render zone polygons as coloured overlays with labels
- `src/swarmap_dashboard/src/components/StatsPanel.tsx` — add zone count summary (X rooms, Y corridors, Z junctions)
- `src/swarmap_bringup/config/default_params.yaml` — add `corridor_width_threshold: 1.5`, `room_min_area: 10.0`

**Difficulty** ★★★★☆

---

## 6. Ghost Trail Replay on Live Map

**What it is**
Record every robot's pose over time during exploration. Draw fading trail lines on the map canvas showing where each robot has been. Add a scrubber that lets you rewind the live map to any point in the mission and watch the exploration unfold from that moment.

**Why it's novel**
The existing ReplayPanel replays saved JSON files offline. This feature records live and lets you scrub backward *during* an ongoing mission — useful for post-incident analysis when a robot fails mid-run.

**Implementation**
- `map_aggregator_node` appends `{t, robot_id, x, y}` to a circular buffer (last 10 min).
- Publishes the buffer as `/dashboard/trail` every 5s (compressed JSON).
- Dashboard stores it in a ref, renders fading polylines per robot (opacity = 1 - age/600s).
- A mini scrubber below the map canvas lets you freeze the rendered time and step through it.

**Files to touch**
- `src/swarmap_core/src/map_aggregator_node.cpp` — add trail buffer, publish `/dashboard/trail`
- `src/swarmap_dashboard/src/components/MapCanvas.tsx` — draw trail polylines, handle scrub time prop
- `src/swarmap_dashboard/src/App.tsx` — add trail scrubber bar below the map panel
- `src/swarmap_dashboard/src/hooks/useRosBridge.ts` — subscribe to `/dashboard/trail`

**Difficulty** ★★★☆☆

---

## Recommendation

If you're demoing soon, go in this order:

| Priority | Feature | Payoff | Effort |
|---|---|---|---|
| 1 | Map Confidence Heatmap | Strong visual wow factor | Low |
| 2 | Energy-Aware Bidding | Measurable algorithmic contribution | Low |
| 3 | Ghost Trail Replay | Very impressive in a live demo | Medium |
| 4 | Predictive Rescue | Research-grade fault tolerance claim | Medium |
| 5 | Multi-Hop Relay | Hard networking problem, high academic value | Medium |
| 6 | Semantic Zone Classification | Most complex, most publishable | High |
