classdef RandomWalkPolicy < handle
    % Random walk with clearing dwells. Event-driven like ED-RHC, but an agent acts only at two
    % of its own events:
    %   arrival:    it dwells for U[1 - dwellSpread, 1 + dwellSpread] times the time to empty
    %               its target, R_i / (N_i B_i - A_i), where N_i counts the agents already there
    %               (assumed to stay until then; they keep their own plans)
    %   departure:  when the dwell is over, it moves to a neighbor drawn uniformly at random
    % Agents may share targets. If the agents present cannot empty the target at its current
    % growth rate (a spike), the baseline rate is used; if they cannot empty it even then, the
    % agent leaves at once.
    properties
        dwellSpread = 0.2   % half-width of the dwell range, as a fraction of the time to empty
        dwellScale = 1      % mean dwell as a multiple of the time to empty (0 = no dwell)
    end

    properties (Access = private)
        dwelt               % agent index -> true once the dwell at its current target is decided
    end

    methods
        function obj = RandomWalkPolicy()
            obj.dwelt = containers.Map('KeyType', 'double', 'ValueType', 'logical');
        end

        function resetReservations(obj)
            % Called by ScenarioModel on every reset: every agent starts with an arrival
            obj.dwelt = containers.Map('KeyType', 'double', 'ValueType', 'logical');
        end

        function cmd = plan(obj, agent, model, adj, curIdx, simTime)
            % Called when the agent has no path and no dwell left: on arrival (or at t = 0), and
            % when its dwell is over (departure)
            cmd = struct();
            if isempty(adj) || curIdx < 1, return; end
            if ~isKey(obj.dwelt, agent.index) || ~obj.dwelt(agent.index)   % arrival
                obj.dwelt(agent.index) = true;
                d = obj.dwellScale * obj.timeToEmpty(model, curIdx, simTime) ...
                    * (1 + obj.dwellSpread * (2 * rand() - 1));
                if d > 0
                    agent.dwellRemaining = d;
                    return;
                end
            end
            obj.dwelt(agent.index) = false;                                   % departure
            nbrs = find(adj(curIdx, :));
            if isempty(nbrs), return; end      % nowhere to go: the next call is a new arrival
            cmd.kind = "move";
            cmd.targetIdx = nbrs(randi(numel(nbrs)));
        end
    end

    methods (Access = private)
        function tau = timeToEmpty(~, model, i, simTime)
            tg = model.targets(i);
            n = max(numel(tg.residingAgents), 1);    % agents at the target, this one included
            [A, baseA] = tg.growthRateAt(simTime, model.uncertaintyMode);
            if n * tg.B <= A, A = baseA; end         % spike: use the baseline rate
            if tg.R <= 0 || n * tg.B <= A
                tau = 0;                             % nothing to clear, or it cannot be cleared
            else
                tau = tg.R / (n * tg.B - A);
            end
        end
    end
end
