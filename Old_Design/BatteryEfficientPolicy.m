classdef BatteryEfficientPolicy < Policy
    % RETIRED on 2026-10-21 and kept here for reference only (not registered). Gabriel Fanshteyn's
    % heuristic: always move to the nearest neighbor (straight-line distance) after a short random
    % dwell, a longer one below criticalBatteryThreshold SOC. It never recharges and has no formal
    % basis; Battery-Aware ED-RHC (BatteryAwareRHCPolicy, with charging stations) replaces it.
    properties
        criticalBatteryThreshold = 20.0 % Percentage
        dwellRange = [0.5 1.5]          % dwell at each target ~ U(dwellRange) seconds; [0 0] = no dwell
        lowBatteryDwellRange = [5 10]   % dwell when the battery is below the threshold
    end

    properties (Constant)
        agentAssumption = "any"   % what it assumes of the agents' motion (see ComponentRegistry.assumptionOf)
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