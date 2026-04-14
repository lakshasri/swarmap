classdef TestMapMerger < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSrc(~)
            here = fileparts(mfilename('fullpath'));
            addpath(fullfile(here, '..', 'src'));
        end
    end

    methods (Test)
        function identicalGridsDoubleConfidence(tc)
            a = zeros(5); b = zeros(5);
            ca = ones(5); cb = ones(5);
            [m, c] = MapMerger.mergeGrids(a, ca, b, cb);
            tc.verifyEqual(m, zeros(5));
            tc.verifyEqual(c, 2*ones(5));
        end

        function knownWinsOverUnknown(tc)
            a = -1 * ones(3); ca = zeros(3);
            b = zeros(3);     cb = ones(3);
            [m, ~] = MapMerger.mergeGrids(a, ca, b, cb);
            tc.verifyEqual(m, zeros(3));
        end

        function weightedAverageOnConflict(tc)
            a = zeros(1); ca = 1;
            b = ones(1);  cb = 3;   % b should dominate -> merged = 1
            [m, ~] = MapMerger.mergeGrids(a, ca, b, cb);
            tc.verifyEqual(m, 1);
        end

        function accuracyFullMatch(tc)
            est = zeros(5); gt = zeros(5);
            tc.verifyEqual(MapMerger.accuracy(est, gt), 1.0);
        end
    end
end
