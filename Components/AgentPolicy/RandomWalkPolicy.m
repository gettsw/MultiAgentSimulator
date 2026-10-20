classdef RandomWalkPolicy < handle
    properties
        dwellRange = [0.25 1.25]   % dwell at each target ~ U(dwellRange) seconds; [0 0] = no dwell
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

            nextIdx = nbrs(randi(numel(nbrs)));

            cmd.kind = "move";
            cmd.targetIdx = nextIdx;

            % Dwell at the current target before leaving
            cmd.dwellSeconds = obj.dwellRange(1) + diff(obj.dwellRange)*rand();
        end
    end
end
