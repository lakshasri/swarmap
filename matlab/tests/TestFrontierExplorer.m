classdef TestFrontierExplorer < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSrc(~)
            here = fileparts(mfilename('fullpath'));
            addpath(fullfile(here, '..', 'src'));
        end
    end

    methods (Test)
        function emptyGridNoFrontiers(tc)
            grid = -1 * ones(20);
            mask = FrontierExplorer.detectFrontiers(grid);
            tc.verifyFalse(any(mask(:)));
        end

        function singleFreeCellIsFrontier(tc)
            grid = -1 * ones(20);
            grid(10, 10) = 0;
            mask = FrontierExplorer.detectFrontiers(grid);
            tc.verifyTrue(mask(10, 10));
        end

        function clusterBelowMinSizeDiscarded(tc)
            grid = -1 * ones(20);
            grid(10, 10) = 0;
            mask = FrontierExplorer.detectFrontiers(grid);
            clusters = FrontierExplorer.clusterFrontiers(mask, 4);
            tc.verifyEmpty(clusters);
        end

        function roomWithDoorProducesFrontier(tc)
            grid = ones(20);
            grid(5:15, 5:15) = 0;  % explored room
            grid(10, 16:20) = -1;  % unknown corridor
            grid(10, 15) = 0;      % doorway
            mask = FrontierExplorer.detectFrontiers(grid);
            clusters = FrontierExplorer.clusterFrontiers(mask, 1);
            tc.verifyNotEmpty(clusters);
        end
    end
end
