classdef MinimumSOC < Metric
    % The lowest battery state of charge [%] of any agent so far in the run (100 at the start).
    % With battery-aware policies it should stay above their reserve (e.g. socMin = 5 %)
    properties (Access = private)
        lowest = 100
    end

    methods
        function obj = MinimumSOC()
            obj.label = "lowest SOC so far [%]";
            obj.plotMax = 100;
        end

        function reset(obj, ~)
            obj.lowest = 100;
        end

        function update(obj, model, ~, ~)
            if isempty(model.agents), return; end
            obj.lowest = min([obj.lowest, arrayfun(@(a) 100 * a.battery.soc, model.agents)]);
        end

        function v = value(obj, ~, ~)
            v = obj.lowest;
        end
    end
end
