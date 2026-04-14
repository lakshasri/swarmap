classdef MapMerger
    %MAPMERGER Weighted-confidence occupancy grid merge (MATLAB port).
    %
    %   Mirrors src/swarmap_core/src/map_merger.cpp so that the two
    %   implementations can be cross-validated on identical inputs.

    methods (Static)
        function [merged, conf] = mergeGrids(grid_a, conf_a, grid_b, conf_b)
            if ~isequal(size(grid_a), size(grid_b))
                error('MapMerger:SizeMismatch', 'Grids must share shape.');
            end
            known_a = grid_a ~= -1;
            known_b = grid_b ~= -1;
            both    = known_a & known_b;
            only_b  = ~known_a & known_b;

            merged = grid_a;
            conf   = conf_a;

            % Both known -> weighted average (rounded to {0,1}).
            wa = conf_a(both); wb = conf_b(both);
            mixed = (wa .* grid_a(both) + wb .* grid_b(both)) ./ max(wa + wb, eps);
            merged(both) = round(mixed);
            conf(both)   = wa + wb;

            % Only B known -> adopt B.
            merged(only_b) = grid_b(only_b);
            conf(only_b)   = conf_b(only_b);
        end

        function merged = applyOffset(grid, offset_rc)
            % Translate a partial grid by integer (row, col) offset.
            merged = -1 * ones(size(grid));
            rs = max(1, 1 + offset_rc(1));
            re = min(size(grid, 1), size(grid, 1) + offset_rc(1));
            cs = max(1, 1 + offset_rc(2));
            ce = min(size(grid, 2), size(grid, 2) + offset_rc(2));
            src_rs = rs - offset_rc(1);
            src_re = re - offset_rc(1);
            src_cs = cs - offset_rc(2);
            src_ce = ce - offset_rc(2);
            merged(rs:re, cs:ce) = grid(src_rs:src_re, src_cs:src_ce);
        end

        function acc = accuracy(estimated, ground_truth)
            % Fraction of known estimated cells matching ground truth.
            known = estimated ~= -1;
            if ~any(known(:)), acc = 0; return; end
            acc = sum(estimated(known) == ground_truth(known)) / sum(known(:));
        end
    end
end
