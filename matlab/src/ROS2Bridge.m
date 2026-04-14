classdef ROS2Bridge < handle
    %ROS2BRIDGE MATLAB <-> live ROS2 co-simulation bridge.
    %
    %   Subscribes to /dashboard/map_compressed and /swarm/health,
    %   computes live accuracy against a ground-truth map, and can
    %   inject noisy LaserScan messages into a running robot for
    %   robustness testing.

    properties
        Node
        MapSub
        HealthSub
        ScanPub
        GroundTruth
        Resolution = 0.1;
        History = struct('time', [], 'coverage', [], 'accuracy', []);
        Running = false;
    end

    methods
        function obj = ROS2Bridge(varargin)
            p = inputParser;
            addParameter(p, 'DomainID', 0);
            addParameter(p, 'NodeName', 'swarmap_matlab_bridge');
            parse(p, varargin{:});
            obj.Node = ros2node(p.Results.NodeName, p.Results.DomainID);
        end

        function connect(obj, ground_truth_pgm, resolution)
            if nargin >= 2 && ~isempty(ground_truth_pgm)
                img = imread(ground_truth_pgm);
                obj.GroundTruth = double(img < 100);  % occupied < 100
            end
            if nargin >= 3, obj.Resolution = resolution; end
            obj.MapSub    = ros2subscriber(obj.Node, '/dashboard/map_compressed', ...
                'nav_msgs/OccupancyGrid', @(m) obj.onMap_(m));
            obj.HealthSub = ros2subscriber(obj.Node, '/swarm/health', ...
                'std_msgs/String', @(m) obj.onHealth_(m));
            obj.ScanPub   = ros2publisher(obj.Node, '/robot_0/scan_override', ...
                'sensor_msgs/LaserScan');
            obj.Running = true;
        end

        function runLive(obj, duration_s)
            fig = figure('Name', 'Swarmap live accuracy');
            ax = axes('Parent', fig); grid(ax, 'on'); hold(ax, 'on');
            xlabel(ax, 't (s)'); ylabel(ax, '%');
            t0 = tic;
            while toc(t0) < duration_s && obj.Running
                if ~isempty(obj.History.time)
                    cla(ax);
                    plot(ax, obj.History.time, obj.History.coverage*100, '-', ...
                         obj.History.time, obj.History.accuracy*100, '--');
                    legend(ax, {'coverage', 'accuracy'}, 'Location', 'southeast');
                    drawnow limitrate;
                end
                pause(0.5);
            end
        end

        function injectNoisyScan(obj, ranges, noise)
            msg = ros2message(obj.ScanPub);
            msg.ranges = single(ranges + noise .* randn(size(ranges)));
            msg.angle_min = single(-pi);
            msg.angle_max = single( pi);
            msg.angle_increment = single(2*pi / numel(ranges));
            send(obj.ScanPub, msg);
        end

        function exportReport(obj, filename)
            T = struct2table(obj.History);
            writetable(T, filename);
        end

        function shutdown(obj)
            obj.Running = false;
            clear obj.MapSub obj.HealthSub obj.ScanPub;
        end
    end

    methods (Access = private)
        function onMap_(obj, msg)
            w = double(msg.info.width);
            h = double(msg.info.height);
            grid = reshape(double(msg.data), w, h)';
            % ROS OccupancyGrid: -1 unknown, 0 free, 100 occupied.
            est = -1 * ones(size(grid));
            est(grid == 0)   = 0;
            est(grid >= 50)  = 1;
            cov = sum(est(:) ~= -1) / numel(est);
            if ~isempty(obj.GroundTruth) && isequal(size(obj.GroundTruth), size(est))
                acc = MapMerger.accuracy(est, obj.GroundTruth);
            else
                acc = NaN;
            end
            t = posixtime(datetime('now'));
            obj.History.time(end+1, 1)     = t;
            obj.History.coverage(end+1, 1) = cov;
            obj.History.accuracy(end+1, 1) = acc;
        end

        function onHealth_(~, msg)
            fprintf('[health] %s\n', char(msg.data));
        end
    end
end
