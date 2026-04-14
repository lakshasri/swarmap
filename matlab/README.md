# Swarmap MATLAB

Offline simulator, algorithm prototypes, and live ROS2 co-simulation bridge.

## Layout

    matlab/
    ├── src/          classes: MapGenerator, SwarmSimulator,
    │                 FrontierExplorer, MapMerger, RunBenchmarks,
    │                 ExportResults, ROS2Bridge
    ├── tests/        matlab.unittest cases (run via `runtests`)
    ├── worlds/       generated .pgm / .sdf worlds
    └── results/      benchmark outputs

## Quick start

```matlab
addpath(genpath('src'));

% 1. Generate a test map
[grid, meta] = MapGenerator.generate('warehouse', 30, 30, 0.25, 42);

% 2. Simulate 10 robots for 300 s
sim = SwarmSimulator(grid, 'NumRobots', 10, 'FailureRate', 0.2);
stats = sim.run(300);
fprintf('coverage=%.2f accuracy=%.2f\n', stats.coverage(end), stats.accuracy(end));

% 3. Full benchmark sweep (takes 30-60 min)
RunBenchmarks('results');
```

## Live co-simulation with ROS2

```matlab
bridge = ROS2Bridge('DomainID', 0);
bridge.connect('worlds/warehouse.pgm', 0.1);
bridge.runLive(300);
bridge.exportReport('live_run.csv');
```

The MATLAB ROS Toolbox must be installed and ROS2 Humble must be sourced
in the shell that launches MATLAB.

## Running tests

```matlab
runtests('tests');
```
