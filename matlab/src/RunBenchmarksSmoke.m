function results = RunBenchmarksSmoke(out_dir)
%RUNBENCHMARKSSMOKE Tiny sweep for a quick (~1 min) sanity run.
%   Use RunBenchmarks for the full ~720-trial sweep. This one runs
%   2 swarm sizes x 2 failure rates x 1 noise x 2 trials = 8 runs at 60 s
%   each so the whole thing finishes in under two minutes on a laptop.

    if nargin < 1
        out_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', ...
                           'results', 'benchmark');
    end
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    num_robots_range   = [5, 10];
    failure_rate_range = [0.0, 0.3];
    noise_range        = 0.05;
    num_trials         = 2;
    sim_seconds        = 60;

    [env, ~] = MapGenerator.generate('warehouse', 30, 30, 0.25, 42);

    rows = {};
    idx = 0;
    total = numel(num_robots_range) * numel(failure_rate_range) * ...
            numel(noise_range) * num_trials;

    for n = num_robots_range
        for fr = failure_rate_range
            for nz = noise_range
                for t = 1:num_trials
                    idx = idx + 1;
                    sim = SwarmSimulator(env, ...
                        'NumRobots', n, 'FailureRate', fr, ...
                        'NoiseLevel', nz, 'Seed', idx);
                    s = sim.run(sim_seconds);
                    t80 = find(s.coverage >= 0.8, 1);
                    if isempty(t80), t80 = NaN; else, t80 = s.time(t80); end
                    rows{end+1} = struct( ...
                        'num_robots', n, 'failure_rate', fr, 'noise', nz, ...
                        'trial', t, 'final_coverage', s.coverage(end), ...
                        'final_accuracy', s.accuracy(end), ...
                        'time_to_80', t80, 'alive_end', s.alive(end)); %#ok<AGROW>
                    fprintf('[%d/%d] N=%d FR=%.2f NZ=%.2f trial=%d cov=%.2f\n', ...
                        idx, total, n, fr, nz, t, s.coverage(end));
                end
            end
        end
    end

    results = [rows{:}]';
    timestamp = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<DATST>
    save(fullfile(out_dir, sprintf('benchmark_smoke_%s.mat', timestamp)), 'results');
    ExportResults(results, out_dir, ['smoke_' timestamp]);
    fprintf('\nSmoke benchmark complete. Output: %s\n', out_dir);
end
