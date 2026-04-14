#!/usr/bin/env bash
# Run the throughput benchmark across swarm sizes and collate results.
set -euo pipefail
cd "$(dirname "$0")/../.."

OUT=results/benchmark/scalability.csv
mkdir -p "$(dirname "$OUT")"
echo "num_robots,cpu_avg,max_p95_latency_ms,passed" > "$OUT"

for N in 5 10 15 20 25 30; do
    echo "==> N=$N"
    ros2 launch swarmap_bringup simulation.launch.py \
        num_robots:=$N failure_rate:=0.0 &
    SIM_PID=$!
    sleep 15
    python3 scripts/benchmarks/perf_throughput.py \
        --num-robots $N --duration 60 \
        --out "results/benchmark/throughput_n${N}.csv" \
        | tee /tmp/perf.log || true
    LINE=$(grep -E '^num_robots=' /tmp/perf.log || echo "")
    PASS=$(grep -E '^(PASS|FAIL)$' /tmp/perf.log | tail -n1 || echo "FAIL")
    CPU=$(echo "$LINE" | sed -n 's/.*cpu_avg=\([0-9.]*\)%.*/\1/p')
    LAT=$(echo "$LINE" | sed -n 's/.*max_p95_latency=\([0-9]*\)ms/\1/p')
    echo "$N,$CPU,$LAT,$PASS" >> "$OUT"
    kill -INT $SIM_PID 2>/dev/null || true
    wait $SIM_PID 2>/dev/null || true
    sleep 5
done

echo "Scalability results written to $OUT"
