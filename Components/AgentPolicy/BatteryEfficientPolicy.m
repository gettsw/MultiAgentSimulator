classdef BatteryEfficientPolicy < handle
    properties
        criticalBatteryThreshold = 20.0 % Percentage
        dwellRange = [0.5 1.5]          % dwell at each target ~ U(dwellRange) seconds; [0 0] = no dwell
        lowBatteryDwellRange = [5 10]   % dwell when the battery is below the threshold
    end

    methods
        function cmd = plan(obj, agent, model, adj, curIdx, simTime) %#ok<INUSD>
            cmd = struct(); % default empty

            if isempty(adj) || curIdx < 1
                return;
            end

            nbrs = find(adj(curIdx, :));
            if isempty(nbrs)
                return;
            end

            bestIdx = nbrs(1);
            minDist = inf;
            
            for i = 1:numel(nbrs)
                targetPos = model.targets(nbrs(i)).position;
                d = norm(targetPos - agent.state.pos);
                if d < minDist
                    minDist = d;
                    bestIdx = nbrs(i);
                end
            end

            cmd.kind = "move";
            cmd.targetIdx = bestIdx;

            range = obj.dwellRange;
            if isprop(agent, 'battery_percentage') && agent.battery_percentage < obj.criticalBatteryThreshold
                range = obj.lowBatteryDwellRange;
            end
            cmd.dwellSeconds = range(1) + diff(range)*rand();
        end
    end
end