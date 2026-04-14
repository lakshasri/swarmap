classdef TestSwarmSimulator < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSrc(~)
            here = fileparts(mfilename('fullpath'));
            addpath(fullfile(here, '..', 'src'));
        end
    end

    methods (Test)
        function coverageGrowsOverTime(tc)
            env = MapGenerator.generate('warehouse', 15, 15, 0.5, 1);
            sim = SwarmSimulator(env, 'NumRobots', 4, 'Seed', 1, ...
                'FailureRate', 0.0, 'NoiseLevel', 0.0);
            stats = sim.run(20);
            tc.verifyGreaterThan(stats.coverage(end), stats.coverage(1));
        end

        function failuresKillSomeRobots(tc)
            env = MapGenerator.generate('warehouse', 10, 10, 0.5, 2);
            sim = SwarmSimulator(env, 'NumRobots', 10, 'Seed', 2, ...
                'FailureRate', 0.5, 'NoiseLevel', 0.0);
            stats = sim.run(30);
            tc.verifyLessThan(stats.alive(end), 10);
        end
    end
end
