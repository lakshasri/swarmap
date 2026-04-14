classdef SwarmSimulator < handle
    %SWARMSIMULATOR Pure-MATLAB offline swarm exploration simulator.
    %
    %   Models N robots performing frontier-driven exploration of a
    %   known occupancy grid.  No ROS2 required.  The update loop
    %   mirrors the high-level behaviour of the C++ stack so the
    %   benchmark results are representative.

    properties
        Env              % ground-truth grid (0 free, 1 occupied)
        Resolution       % metres per cell
        NumRobots
        SensorRange      % metres
        CommRadius       % metres
        FailureRate      % probability a robot dies during mission
        NoiseLevel       % probability a sensed cell is wrong
        MapResolution
        Robots           % struct array
        GlobalMap        % fused ground-truth estimate
        TimeStep = 0.2;  % seconds
        SimTime  = 0;
    end

    methods
        function obj = SwarmSimulator(env, varargin)
            p = inputParser;
            addParameter(p, 'NumRobots', 10);
            addParameter(p, 'SensorRange', 5.0);
            addParameter(p, 'CommRadius', 8.0);
            addParameter(p, 'FailureRate', 0.0);
            addParameter(p, 'NoiseLevel', 0.05);
            addParameter(p, 'MapResolution', 0.1);
            addParameter(p, 'Seed', 0);
            parse(p, varargin{:});
            obj.Env = env;
            obj.Resolution = p.Results.MapResolution;
            obj.NumRobots  = p.Results.NumRobots;
            obj.SensorRange = p.Results.SensorRange;
            obj.CommRadius  = p.Results.CommRadius;
            obj.FailureRate = p.Results.FailureRate;
            obj.NoiseLevel  = p.Results.NoiseLevel;
            obj.MapResolution = p.Results.MapResolution;
            try, rng(p.Results.Seed); catch, rand('state', p.Results.Seed); randn('state', p.Results.Seed); end

            obj.GlobalMap = -1 * ones(size(env));
            obj.Robots = obj.spawnRobots_();
        end

        function stats = run(obj, duration_s)
            steps = round(duration_s / obj.TimeStep);
            stats = struct( ...
                'coverage', zeros(steps, 1), ...
                'accuracy', zeros(steps, 1), ...
                'alive',    zeros(steps, 1), ...
                'time',     zeros(steps, 1));
            failures_scheduled = obj.scheduleFailures_(steps);
            for s = 1:steps
                obj.SimTime = obj.SimTime + obj.TimeStep;
                obj.step_(s, failures_scheduled);
                obj.fuseGlobalMap_();
                stats.coverage(s) = obj.coverage();
                stats.accuracy(s) = obj.accuracy();
                stats.alive(s)    = sum([obj.Robots.alive]);
                stats.time(s)     = obj.SimTime;
            end
        end

        function c = coverage(obj)
            known = obj.GlobalMap ~= -1;
            c = sum(known(:)) / numel(obj.GlobalMap);
        end

        function a = accuracy(obj)
            a = MapMerger.accuracy(obj.GlobalMap, obj.Env);
        end
    end

    methods (Access = private)
        function robots = spawnRobots_(obj)
            robots(obj.NumRobots) = struct( ...
                'id', 0, 'pose', [0 0], 'map', [], 'conf', [], ...
                'alive', true, 'battery', 1.0, 'target', []);
            free = find(obj.Env == 0);
            pick = free(randperm(numel(free), obj.NumRobots));
            [rr, cc] = ind2sub(size(obj.Env), pick);
            for i = 1:obj.NumRobots
                robots(i).id = i;
                robots(i).pose = [cc(i) * obj.Resolution, rr(i) * obj.Resolution];
                robots(i).map  = -1 * ones(size(obj.Env));
                robots(i).conf = zeros(size(obj.Env));
                robots(i).alive = true;
                robots(i).battery = 1.0;
            end
        end

        function sched = scheduleFailures_(obj, steps)
            num_fail = round(obj.FailureRate * obj.NumRobots);
            sched = zeros(obj.NumRobots, 1);
            if num_fail == 0, return; end
            ids = randperm(obj.NumRobots, num_fail);
            sched(ids) = randi(steps, [num_fail, 1]);
        end

        function step_(obj, s, failures)
            for i = 1:numel(obj.Robots)
                if ~obj.Robots(i).alive, continue; end
                if failures(i) == s
                    obj.Robots(i).alive = false;
                    continue;
                end
                obj.sense_(i);
                obj.plan_(i);
                obj.move_(i);
                obj.Robots(i).battery = max(0, obj.Robots(i).battery - 1e-4);
                if obj.Robots(i).battery <= 0
                    obj.Robots(i).alive = false;
                end
            end
            obj.shareMaps_();
        end

        function sense_(obj, i)
            r = obj.Robots(i);
            [rows, cols] = size(obj.Env);
            cx = round(r.pose(1) / obj.Resolution);
            cy = round(r.pose(2) / obj.Resolution);
            rng_cells = round(obj.SensorRange / obj.Resolution);
            for a = 0:5:355
                [dx, dy] = pol2cart(deg2rad(a), rng_cells);
                [xs, ys] = bresenham_(cx, cy, cx+round(dx), cy+round(dy));
                for k = 1:numel(xs)
                    xx = xs(k); yy = ys(k);
                    if xx < 1 || yy < 1 || xx > cols || yy > rows, break; end
                    truth = obj.Env(yy, xx);
                    if rand() < obj.NoiseLevel
                        truth = 1 - truth;
                    end
                    obj.Robots(i).map(yy, xx) = truth;
                    obj.Robots(i).conf(yy, xx) = obj.Robots(i).conf(yy, xx) + 1;
                    if truth == 1, break; end
                end
            end
        end

        function plan_(obj, i)
            target = obj.Robots(i).target;
            if ~isempty(target)
                d = norm([target(1) - obj.Robots(i).pose(1), ...
                          target(2) - obj.Robots(i).pose(2)]);
                if d > 2 * obj.Resolution, return; end
            end
            f = FrontierExplorer.pickFrontier(obj.Robots(i).map, ...
                obj.Robots(i).pose, 4, obj.Resolution);
            if isempty(f)
                obj.Robots(i).target = [];
            else
                obj.Robots(i).target = [f.centroid(2) * obj.Resolution, ...
                                        f.centroid(1) * obj.Resolution];
            end
        end

        function move_(obj, i)
            target = obj.Robots(i).target;
            if isempty(target), return; end
            step = 0.5 * obj.TimeStep;  % 0.5 m/s
            dir = target - obj.Robots(i).pose;
            d = norm(dir);
            if d < 1e-6, return; end
            new_pose = obj.Robots(i).pose + (dir / d) * min(step, d);
            rc = round(new_pose(2) / obj.Resolution);
            cc = round(new_pose(1) / obj.Resolution);
            if rc >= 1 && cc >= 1 && rc <= size(obj.Env,1) && cc <= size(obj.Env,2) ...
                    && obj.Env(rc, cc) == 0
                obj.Robots(i).pose = new_pose;
            else
                % blocked -> abandon target so we replan
                obj.Robots(i).target = [];
            end
        end

        function shareMaps_(obj)
            alive_idx = find([obj.Robots.alive]);
            for a = alive_idx
                for b = alive_idx
                    if b <= a, continue; end
                    d = norm(obj.Robots(a).pose - obj.Robots(b).pose);
                    if d <= obj.CommRadius
                        [m, c] = MapMerger.mergeGrids( ...
                            obj.Robots(a).map, obj.Robots(a).conf, ...
                            obj.Robots(b).map, obj.Robots(b).conf);
                        obj.Robots(a).map = m; obj.Robots(a).conf = c;
                        [m, c] = MapMerger.mergeGrids( ...
                            obj.Robots(b).map, obj.Robots(b).conf, ...
                            obj.Robots(a).map, obj.Robots(a).conf);
                        obj.Robots(b).map = m; obj.Robots(b).conf = c;
                    end
                end
            end
        end

        function fuseGlobalMap_(obj)
            obj.GlobalMap = -1 * ones(size(obj.Env));
            conf = zeros(size(obj.Env));
            for i = 1:numel(obj.Robots)
                [obj.GlobalMap, conf] = MapMerger.mergeGrids( ...
                    obj.GlobalMap, conf, obj.Robots(i).map, obj.Robots(i).conf);
            end
        end
    end
end

function [xs, ys] = bresenham_(x0, y0, x1, y1)
    dx = abs(x1 - x0); dy = abs(y1 - y0);
    sx = sign(x1 - x0); sy = sign(y1 - y0);
    err = dx - dy;
    xs = zeros(1, dx+dy+1); ys = xs;
    n = 0;
    while true
        n = n + 1; xs(n) = x0; ys(n) = y0;
        if x0 == x1 && y0 == y1, break; end
        e2 = 2*err;
        if e2 > -dy, err = err - dy; x0 = x0 + sx; end
        if e2 <  dx, err = err + dx; y0 = y0 + sy; end
        if n > 5000, break; end
    end
    xs = xs(1:n); ys = ys(1:n);
end
