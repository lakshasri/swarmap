# Results

Artifacts produced by demo runs, benchmarks, and MATLAB sweeps.
Everything in this directory (except this README) is excluded from
git — the large binary outputs are shared via a GitHub Release
(see `.github/workflows/release.yml`).

## Directory structure

```
results/
├── demo_basic/                       Phase 11 scenario 1
│   ├── run.bag/                      rosbag2 of /dashboard/* topics
│   ├── final_map.pgm + .yaml         saved occupancy grid
│   ├── coverage_timeline.csv         time, coverage, accuracy
│   └── demo_basic.mp4                screen capture of the dashboard
│
├── demo_fault_tolerance/             Phase 11 scenario 2
│   ├── run.bag/
│   ├── final_map.pgm + .yaml
│   ├── failure_log.csv               t, robot_id, mode, coverage_at_failure
│   ├── fault_tolerance_chart.png     coverage vs. time, overlaid failure events
│   └── demo_fault_tolerance.mp4
│
└── benchmark/                        Phase 9 & Phase 10 outputs
    ├── benchmark_<timestamp>.mat     MATLAB raw results
    ├── benchmark_<timestamp>.csv     tabular export (Python/Excel friendly)
    ├── benchmark_report_*.pdf        multi-page plots (coverage, heatmap, ...)
    ├── throughput_n<N>.csv           per-robot latency at N robots
    ├── scalability.csv               aggregate CPU / latency / pass vs. N
    └── fault_tolerance.csv           10-run validation sweep
```

## Regenerating the artifacts

| Artifact | Command |
|---|---|
| `demo_basic/*` | `scripts/run_demos.sh` (phase 11) |
| `demo_fault_tolerance/*` | `scripts/run_demos.sh` |
| `benchmark/throughput_*` | `scripts/benchmarks/perf_throughput.py` |
| `benchmark/scalability.csv` | `scripts/benchmarks/perf_scalability.sh` |
| `benchmark/fault_tolerance.csv` | `scripts/benchmarks/fault_tolerance_validation.py` |
| `benchmark/benchmark_*.{mat,csv,pdf}` | `matlab: RunBenchmarks('results/benchmark')` |

## Coverage chart generation

```bash
python3 scripts/benchmarks/plot_coverage.py \
    --bag results/demo_fault_tolerance/run.bag \
    --out results/demo_fault_tolerance/fault_tolerance_chart.png
```
