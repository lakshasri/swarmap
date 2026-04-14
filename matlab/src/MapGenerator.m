classdef MapGenerator
    %MAPGENERATOR Procedurally generate occupancy grids for simulation.
    %
    %   Supports 'warehouse', 'office', 'random', and 'maze' map types.
    %   Seeded RNG is used so that calling generate(...) with the same
    %   seed produces identical maps across runs.

    methods (Static)
        function [grid, metadata] = generate(type, width, height, resolution, seed)
            if nargin < 5, seed = 0; end
            if nargin < 4, resolution = 0.1; end
            rng(seed);

            cols = round(width / resolution);
            rows = round(height / resolution);

            switch lower(type)
                case 'warehouse'
                    grid = MapGenerator.warehouse_(rows, cols);
                case 'office'
                    grid = MapGenerator.office_(rows, cols);
                case 'random'
                    grid = MapGenerator.randomNoise_(rows, cols);
                case 'maze'
                    grid = MapGenerator.maze_(rows, cols);
                otherwise
                    error('MapGenerator:BadType', 'Unknown map type "%s"', type);
            end

            metadata = struct( ...
                'type', type, ...
                'width_m', width, ...
                'height_m', height, ...
                'resolution_m', resolution, ...
                'seed', seed);
        end

        function savePGM(grid, pgm_path)
            % Save occupancy grid to a .pgm file (ROS/nav2 compatible).
            img = uint8(254 * (1 - grid));  % free=254, occupied=0
            imwrite(img, pgm_path);
        end

        function toGazeboSDF(grid, resolution, output_path)
            % Convert occupancy grid into an SDF world with box primitives.
            % Adjacent occupied cells are grouped into rectangles (greedy)
            % to keep the primitive count reasonable.
            rects = MapGenerator.rectangles_(grid);
            fid = fopen(output_path, 'w');
            cu = onCleanup(@() fclose(fid));
            fprintf(fid, '<?xml version="1.0" ?>\n');
            fprintf(fid, '<sdf version="1.9">\n  <world name="generated">\n');
            fprintf(fid, '    <include><uri>model://ground_plane</uri></include>\n');
            fprintf(fid, '    <include><uri>model://sun</uri></include>\n');
            for i = 1:size(rects, 1)
                r = rects(i, :);  % [row, col, h, w]
                cx = (r(2) + r(4)/2) * resolution;
                cy = (r(1) + r(3)/2) * resolution;
                sx = r(4) * resolution;
                sy = r(3) * resolution;
                fprintf(fid, ['    <model name="wall_%d"><static>true</static>' ...
                    '<pose>%.3f %.3f 0.5 0 0 0</pose><link name="l">' ...
                    '<collision name="c"><geometry><box><size>%.3f %.3f 1.0</size>' ...
                    '</box></geometry></collision><visual name="v"><geometry><box>' ...
                    '<size>%.3f %.3f 1.0</size></box></geometry></visual></link>' ...
                    '</model>\n'], i, cx, cy, sx, sy, sx, sy);
            end
            fprintf(fid, '  </world>\n</sdf>\n');
        end
    end

    methods (Static, Access = private)
        function g = warehouse_(rows, cols)
            g = zeros(rows, cols);
            g(1, :) = 1; g(end, :) = 1; g(:, 1) = 1; g(:, end) = 1;
            shelf_rows = round(linspace(rows*0.2, rows*0.8, 4));
            for r = shelf_rows
                g(r:r+1, round(cols*0.15):round(cols*0.85)) = 1;
            end
        end

        function g = office_(rows, cols)
            g = zeros(rows, cols);
            g(1, :) = 1; g(end, :) = 1; g(:, 1) = 1; g(:, end) = 1;
            mid_r = round(rows/2); mid_c = round(cols/2);
            g(mid_r, :) = 1;
            g(:, mid_c) = 1;
            % doorways
            g(mid_r, round(cols*0.25)) = 0;
            g(mid_r, round(cols*0.75)) = 0;
            g(round(rows*0.25), mid_c) = 0;
            g(round(rows*0.75), mid_c) = 0;
        end

        function g = randomNoise_(rows, cols)
            % Simple smoothed-random stand-in for Perlin noise.
            n = rand(rows, cols);
            k = ones(5)/25;
            n = conv2(n, k, 'same');
            g = double(n > 0.55);
            g(1,:) = 1; g(end,:) = 1; g(:,1) = 1; g(:,end) = 1;
        end

        function g = maze_(rows, cols)
            g = ones(rows, cols);
            stack = [2 2];
            g(2,2) = 0;
            while ~isempty(stack)
                cur = stack(end, :);
                dirs = [-2 0; 2 0; 0 -2; 0 2];
                dirs = dirs(randperm(4), :);
                moved = false;
                for i = 1:4
                    nr = cur(1) + dirs(i, 1);
                    nc = cur(2) + dirs(i, 2);
                    if nr > 1 && nr < rows && nc > 1 && nc < cols && g(nr, nc) == 1
                        g((cur(1)+nr)/2, (cur(2)+nc)/2) = 0;
                        g(nr, nc) = 0;
                        stack(end+1, :) = [nr nc]; %#ok<AGROW>
                        moved = true;
                        break;
                    end
                end
                if ~moved
                    stack(end, :) = [];
                end
            end
        end

        function rects = rectangles_(grid)
            % Greedy rectangle cover of occupied cells.
            visited = false(size(grid));
            rects = zeros(0, 4);
            for r = 1:size(grid, 1)
                for c = 1:size(grid, 2)
                    if grid(r, c) == 1 && ~visited(r, c)
                        w = 0;
                        while c + w <= size(grid, 2) && grid(r, c+w) == 1 && ~visited(r, c+w)
                            w = w + 1;
                        end
                        h = 1;
                        expanding = true;
                        while expanding && r + h <= size(grid, 1)
                            if all(grid(r+h, c:c+w-1) == 1) && all(~visited(r+h, c:c+w-1))
                                h = h + 1;
                            else
                                expanding = false;
                            end
                        end
                        visited(r:r+h-1, c:c+w-1) = true;
                        rects(end+1, :) = [r-1, c-1, h, w]; %#ok<AGROW>
                    end
                end
            end
        end
    end
end
