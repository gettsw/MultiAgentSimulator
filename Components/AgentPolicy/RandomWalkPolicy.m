classdef RandomWalkPolicy < handle
    methods
        function cmd = plan(~, agent, model, adj, curIdx, simTime) %#ok<INUSD>
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

            % Optional dwell time after arrival
            cmd.dwellSeconds = 0.25 + 1.0*rand(); 
        end
    end
end