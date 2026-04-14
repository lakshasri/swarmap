# Results

Artifacts produced by demo runs and MATLAB benchmarks.
Everything here (except this README and `.gitkeep` markers) is excluded
from git.

## Directory structure

```
results/
├── demo_basic/                       Demo 1 outputs
│   ├── run.bag/                      rosbag2 of /swarm/global_map + /swarm/health
│   └── coverage_timeline.csv         time, coverage (post-processed)
│
├── demo_fault_tolerance/             Demo 2 outputs
│   ├── run.bag/                      rosbag2 with /swarm/events included
│   ├── failure_log.csv               t, robot_id, mode
│   └── fault_tolerance_chart.png     coverage vs. time with failure markers
│
└── benchmark/                        MATLAB sweeps
    ├── benchmark_<timestamp>.mat     raw results
    ├── benchmark_<timestamp>.csv     tabular export
    └── benchmark_report_*.pdf        coverage vs. swarm size + failure rate
```

## How outputs are produced

- `demo_basic/` and `demo_fault_tolerance/` are written by the
  corresponding launch files (`demo_basic.launch.py`,
  `demo_fault_tolerance.launch.py`); they spawn `ros2 bag record`
  automatically.
- `benchmark/` is written by MATLAB:
  `addpath(genpath('matlab/src')); RunBenchmarks('results/benchmark')`.

## Post-processing

Open a rosbag with:
```bash
ros2 bag info results/demo_basic/run.bag
ros2 bag play results/demo_basic/run.bag
```
Then re-launch RViz to visualise the playback against the same config.
