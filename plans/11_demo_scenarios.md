# Phase 11 — Demo Scenarios and Final Deliverables

## Goal
Package the project into ready-to-run demo scenarios that clearly show each project outcome, and produce the final deliverables.

---

## Tasks

### 11.1 — Demo Scenario 1: Basic Swarm Exploration
**Proves:** Robots explore and map without central coordination.

- [ ] Create `launch/demo_basic.launch.py`:
  - Environment: `warehouse.sdf` (30×30 m, shelving rows)
  - Robots: 10, sensor range: 5 m, comm radius: 8 m
  - Failure rate: 0% (no failures — clean demonstration)
  - Duration: 4 minutes (timer stops mission automatically)
- [ ] Expected outcome:
  - Dashboard shows map filling in from multiple robot starting positions simultaneously
  - No robot duplicates another's path (auction prevents overlap)
  - Coverage reaches ≥ 90% within 4 minutes
- [ ] Record a 2-minute GIF/MP4 of the dashboard during this run
- [ ] Save final map to `results/demo_basic_map.pgm`

### 11.2 — Demo Scenario 2: Fault Tolerance Under Fire
**Proves:** Swarm keeps working when nearly half the robots fail.

- [ ] Create `launch/demo_fault_tolerance.launch.py`:
  - Environment: `large_office.sdf` (50×40 m, rooms + corridors)
  - Robots: 20, sensor range: 4 m, comm radius: 7 m
  - Failure rate: 40%, failure mode: progressive
  - Mission duration: 5 minutes
- [ ] Expected outcome:
  - Dashboard shows robots going dark progressively
  - Failure event log fills with entries
  - Map continues growing despite failures
  - Final coverage ≥ 85% with only 12 survivors
- [ ] Record full 5-minute session
- [ ] Export `failure_log.csv` showing each failure with coverage at that moment
- [ ] Save coverage-over-time chart as `results/fault_tolerance_chart.png`

### 11.3 — Demo Scenario 3: Parameter Exploration
**Proves:** User controls affect swarm behaviour in real time.

- [ ] No fixed launch file — interactive demo:
  1. Start with 5 robots, low noise, 0% failures
  2. Live: slide noise level up to 30% — show map accuracy drop in stats panel
  3. Live: add 10 more robots (slider to 15) — show coverage rate spike
  4. Live: set failure rate to 20% — watch robots fail and recover
  5. Live: reduce comm radius to 2 m — watch network topology fragment
- [ ] Prepare a demo script (text) for presenting this scenario in order
- [ ] Record as a 3-minute narrated screen capture

### 11.4 — Demo Scenario 4: MATLAB Benchmark Results
**Proves:** Rigorous quantitative analysis of swarm performance.

- [ ] Run `RunBenchmarks.m` with full parameter sweep (may take 30–60 min)
- [ ] Produce 4 benchmark plots (as described in Phase 9):
  - Coverage vs. swarm size
  - Time-to-80%-coverage vs. failure rate
  - Map accuracy vs. noise level
  - Heat map: swarm size × failure rate → final coverage
- [ ] Export `results/benchmark_report.pdf`
- [ ] Key result to highlight: at 40% failure rate, 20-robot swarm still achieves >85% coverage; 5-robot swarm drops to ~60%

### 11.5 — Results Folder Structure

Ensure all demo outputs land in a clean `results/` directory:

```
results/
├── demo_basic/
│   ├── final_map.pgm
│   ├── final_map.yaml
│   ├── coverage_timeline.csv
│   └── demo_basic.mp4
├── demo_fault_tolerance/
│   ├── final_map.pgm
│   ├── failure_log.csv
│   ├── fault_tolerance_chart.png
│   └── demo_fault_tolerance.mp4
├── benchmark/
│   ├── benchmark_{timestamp}.csv
│   ├── benchmark_{timestamp}.mat
│   └── benchmark_report.pdf
└── README.md   ← explains what each file is
```

- [ ] Create `results/README.md` documenting all output files
- [ ] Add `results/` to `.gitignore` (results are large; share via Google Drive or GitHub Release)
- [ ] Add a GitHub Release workflow that uploads the PDF report as a release asset on `git tag`

### 11.6 — Project Documentation

- [ ] **Architecture diagram** — update `docs/architecture.png` to match final implementation
- [ ] **Topic naming table** — verify `docs/topic_naming.md` matches all implemented topics
- [ ] **Quick Start** — verify README quick-start commands work on a fresh Ubuntu 22.04 install (test in a VM)
- [ ] **Troubleshooting guide** — `docs/troubleshooting.md`:
  - rosbridge connection refused → check port 9090 and firewall
  - Robots not communicating → check `ROS_DOMAIN_ID` matches
  - Gazebo crash → check GPU driver and Gazebo Harmonic version
  - MATLAB can't find ros2 → check ROS2 environment sourced before MATLAB launch

### 11.7 — Final Checklist (Project Done)

- [ ] Scenario 1 runs unattended and produces ≥ 90% coverage
- [ ] Scenario 2 runs unattended and produces ≥ 85% coverage with ≥ 40% failures
- [ ] Browser dashboard renders at 30+ fps with 20 robots
- [ ] User can change any parameter via slider and see the effect within 2 seconds
- [ ] MATLAB benchmark PDF contains 4 plots with labelled axes and clear takeaways
- [ ] All C++ unit and integration tests pass
- [ ] GitHub Actions CI is green
- [ ] README quick-start works on a fresh VM
- [ ] All three demo recordings are available in `results/`

---

## Presentation Script (Outline)

For demos and final presentations, present in this order:

1. **Problem statement** (1 min) — why decentralised mapping matters
2. **Architecture overview** (2 min) — show the diagram, explain peer-to-peer design
3. **Scenario 1 live demo** (3 min) — run the dashboard live, narrate what's happening
4. **Scenario 2 live demo** (3 min) — start killing robots, show the swarm adapting
5. **MATLAB benchmark plots** (2 min) — show the coverage vs. swarm size chart
6. **Live parameter tuning** (2 min) — adjust sliders live, show real-time effects
7. **Q&A / code walkthrough** — open to questions

Total: ~13 minutes for a full technical demo.
