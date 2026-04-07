# Results

This directory stores outputs from simulation runs, benchmarks, and fault-tolerance demos.

## Directory structure

```
results/
├── demo_basic/            # Basic 10-robot exploration run
│   ├── coverage.csv       # Time vs coverage % (cols: time_s, coverage_pct)
│   ├── map_final.pgm      # Final merged occupancy grid (PGM image)
│   └── rosbag/            # Optional: full ROS2 bag for replay
│
├── demo_fault_tolerance/  # Progressive failure injection run
│   ├── coverage.csv
│   ├── failure_log.csv    # One row per failure event (cols: time_s, robot_id, mode)
│   ├── map_final.pgm
│   └── rosbag/
│
└── benchmark/             # Variable-N sweep (N=5,10,15,20 robots)
    ├── sweep_results.csv  # cols: n_robots, time_to_90pct_s, map_accuracy
    └── plots/             # Generated PNG charts
```

## File formats

| File | Description |
|------|-------------|
| `coverage.csv` | `time_s,coverage_pct` — sampled every 5 s |
| `failure_log.csv` | `time_s,robot_id,failure_mode` — written by `failure_injector_node` |
| `map_final.pgm` | Standard ROS map-saver output; pair with a `.yaml` metadata file |
| `sweep_results.csv` | Aggregated benchmark across robot counts |

## Generating results

```bash
# Record a run
ros2 bag record -o results/demo_basic/rosbag /swarm/global_map /swarm/health

# Export coverage log (from swarm_monitor_node stdout redirect)
ros2 launch swarmap_bringup simulation.launch.py 2>&1 | grep COVERAGE > results/demo_basic/coverage.csv

# Save final map
ros2 run nav2_map_server map_saver_cli -f results/demo_basic/map_final
```

> The `results/` directory (except this README) is excluded from git via `.gitignore`.
