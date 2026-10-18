classdef BatteryEfficientPolicy < handle
    properties
        criticalBatteryThreshold = 20.0 % Percentage
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

            if isprop(agent, 'battery_percentage') && agent.battery_percentage < obj.criticalBatteryThreshold
                cmd.dwellSeconds = 5.0 + 5.0*rand(); 
            else
                cmd.dwellSeconds = 0.5 + 1.0*rand(); 
            end
        end
    end
end