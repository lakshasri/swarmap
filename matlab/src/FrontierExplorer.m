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
            % 8-connected BFS labelling — pure-MATLAB so we don't depend on
            % the Image Processing Toolbox or octave-image's bwconncomp.
            [rows, cols] = size(frontier_mask);
            visited = false(rows, cols);
            clusters = struct('idx', {}, 'centroid', {}, 'size', {});
            dr = [-1 -1 -1 0 0 1 1 1];
            dc = [-1 0 1 -1 1 -1 0 1];
            for r0 = 1:rows
                for c0 = 1:cols
                    if ~frontier_mask(r0, c0) || visited(r0, c0), continue; end
                    queue = [r0 c0];
                    qhead = 1;
                    px_r = []; px_c = [];
                    while qhead <= size(queue, 1)
                        rc = queue(qhead, :); qhead = qhead + 1;
                        r = rc(1); c = rc(2);
                        if visited(r, c), continue; end
                        visited(r, c) = true;
                        px_r(end+1) = r; %#ok<AGROW>
                        px_c(end+1) = c; %#ok<AGROW>
                        for k = 1:8
                            nr = r + dr(k); nc = c + dc(k);
                            if nr < 1 || nr > rows || nc < 1 || nc > cols, continue; end
                            if frontier_mask(nr, nc) && ~visited(nr, nc)
                                queue(end+1, :) = [nr nc]; %#ok<AGROW>
                            end
                        end
                    end
                    if numel(px_r) < min_size, continue; end
                    px = sub2ind([rows cols], px_r(:), px_c(:));
                    clusters(end+1) = struct( ...
                        'idx', px, ...
                        'centroid', [mean(px_r), mean(px_c)], ...
                        'size', numel(px_r)); %#ok<AGROW>
                end
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
