classdef FrontierExplorer
    %FRONTIEREXPLORER MATLAB port of the C++ frontier detection + scoring.
    %
    %   Grid convention (matches swarmap_core):
    %       -1  = unknown
    %        0  = free
    %        1  = occupied
    %   A frontier cell is a free cell with at least one unknown neighbour.

    methods (Static)
        function frontiers = detectFrontiers(grid)
            [rows, cols] = size(grid);
            frontiers = false(rows, cols);
            for r = 2:rows-1
                for c = 2:cols-1
                    if grid(r, c) ~= 0, continue; end
                    nb = grid(r-1:r+1, c-1:c+1);
                    if any(nb(:) == -1)
                        frontiers(r, c) = true;
                    end
                end
            end
        end

        function clusters = clusterFrontiers(frontier_mask, min_size)
            if nargin < 2, min_size = 4; end
            cc = bwconncomp(frontier_mask, 8);
            clusters = struct('idx', {}, 'centroid', {}, 'size', {});
            for i = 1:cc.NumObjects
                px = cc.PixelIdxList{i};
                if numel(px) < min_size, continue; end
                [r, c] = ind2sub(size(frontier_mask), px);
                clusters(end+1) = struct( ...
                    'idx', px, ...
                    'centroid', [mean(r), mean(c)], ...
                    'size', numel(px)); %#ok<AGROW>
            end
        end

        function score = scoreFrontier(cluster, robot_pose, resolution)
            % Lower score = more attractive.  Matches C++ weighting:
            % distance (m) dominates, discounted by log(size).
            if nargin < 3, resolution = 0.1; end
            dx = (cluster.centroid(2) - robot_pose(1)/resolution);
            dy = (cluster.centroid(1) - robot_pose(2)/resolution);
            dist_m = sqrt(dx^2 + dy^2) * resolution;
            score = dist_m - 0.5 * log(max(cluster.size, 1));
        end

        function best = pickFrontier(grid, robot_pose, min_size, resolution)
            if nargin < 3, min_size = 4; end
            if nargin < 4, resolution = 0.1; end
            mask = FrontierExplorer.detectFrontiers(grid);
            clusters = FrontierExplorer.clusterFrontiers(mask, min_size);
            if isempty(clusters), best = []; return; end
            scores = arrayfun(@(k) FrontierExplorer.scoreFrontier( ...
                clusters(k), robot_pose, resolution), 1:numel(clusters));
            [~, i] = min(scores);
            best = clusters(i);
        end
    end
end
