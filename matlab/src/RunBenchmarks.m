function results = RunBenchmarks(out_dir)
%RUNBENCHMARKS Parameter sweep over swarm size × failure rate × noise.
%   results = RUNBENCHMARKS(OUT_DIR) runs SwarmSimulator across the sweep
%   defined below and writes .mat / .csv / plots into OUT_DIR.

    if nargin < 1
        out_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
    end
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    num_robots_range   = [2, 5, 10, 15, 20, 30];
    failure_rate_range = [0.0, 0.1, 0.2, 0.3, 0.4, 0.45];
    noise_range        = [0.0, 0.05, 0.1, 0.2];
    num_trials         = 5;
    sim_seconds        = 300;

    [env, ~] = MapGenerator.generate('warehouse', 30, 30, 0.25, 42);

    combos = combvec(1:numel(num_robots_range), ...
                     1:numel(failure_rate_range), ...
                     1:numel(noise_range))';
    total = size(combos, 1) * num_trials;
    rows = cell(total, 1);

    idx = 0;
    for c = 1:size(combos, 1)
        n  = num_robots_range(combos(c, 1));
        fr = failure_rate_range(combos(c, 2));
        nz = noise_range(combos(c, 3));
        for t = 1:num_trials
            idx = idx + 1;
            sim = SwarmSimulator(env, ...
                'NumRobots', n, 'FailureRate', fr, ...
                'NoiseLevel', nz, 'Seed', idx);
            s = sim.run(sim_seconds);
            t80 = find(s.coverage >= 0.8, 1);
            if isempty(t80), t80 = NaN; else, t80 = s.time(t80); end
            rows{idx} = struct( ...
                'num_robots', n, 'failure_rate', fr, 'noise', nz, ...
                'trial', t, 'final_coverage', s.coverage(end), ...
                'final_accuracy', s.accuracy(end), 'time_to_80', t80, ...
                'alive_end', s.alive(end));
            fprintf('[%d/%d] N=%d FR=%.2f NZ=%.2f trial=%d cov=%.2f\n', ...
                idx, total, n, fr, nz, t, s.coverage(end));
        end
    end

    results = [rows{:}]';
    timestamp = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<DATST>
    save(fullfile(out_dir, sprintf('benchmark_%s.mat', timestamp)), 'results');
    ExportResults(results, out_dir, timestamp);
end

function M = combvec(varargin)
    % Minimal combvec replacement (avoids Neural Network Toolbox).
    grids = cell(1, nargin);
    [grids{:}] = ndgrid(varargin{:});
    M = cell2mat(cellfun(@(x) x(:)', grids, 'UniformOutput', false)');
end
