classdef TestMapGenerator < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSrc(~)
            here = fileparts(mfilename('fullpath'));
            addpath(fullfile(here, '..', 'src'));
        end
    end

    methods (Test)
        function seedIsDeterministic(tc)
            a = MapGenerator.generate('random', 10, 10, 0.5, 7);
            b = MapGenerator.generate('random', 10, 10, 0.5, 7);
            tc.verifyEqual(a, b);
        end

        function borderIsOccupied(tc)
            g = MapGenerator.generate('warehouse', 8, 8, 0.5, 0);
            tc.verifyTrue(all(g(1, :) == 1));
            tc.verifyTrue(all(g(end, :) == 1));
        end
    end
end
