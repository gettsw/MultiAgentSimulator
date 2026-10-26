classdef SimplePolicy < Policy
    % SIMPLEPOLICY  A minimal new movement policy: chase the events.
    %
    % Part of the beginner's example in Examples/EventDetection (a new monitoring problem built
    % from four small subclasses: SimpleTarget, SimpleAgent, SimplePolicy and SimpleMetric).
    %
    % The rule: an agent at target i goes to the neighbor j with the largest R_j (with
    % SimpleTarget, the most events in progress) among the free neighbors, i.e. those that no
    % other agent is at or heading to. When every free neighbor has R = 0, it goes to the free
    % neighbor visited longest ago. When no neighbor is free, it stays and asks again at the
    % next step. It never dwells: SimpleTarget detects the events as soon as an agent rests at
    % the target. The rule only uses R, so it also runs on the default Uncertainty Target
    % (greedy on the uncertainty).
    %
    % How it plugs in:
    %   - It subclasses Policy, the abstract base of the movement policies, and implements
    %     plan(agent, model, adj, i, simTime), its one required method. The model calls it
    %     whenever an agent at target i has no path and no dwell left; adj is the adjacency
    %     matrix of the targets. plan returns cmd.kind = "move" and cmd.targetIdx = j (the
    %     model then gives the agent the edge's path), or struct() to stay. A policy may also
    %     return cmd.dwellSeconds (wait at i before leaving) or set agent.dwellRemaining.
    %   - plan sees the whole model: the targets (model.targets: R, position, lastVisitedTime,
    %     and the properties of the target type), the edges and the other agents.
    %   - Registered in Components/ComponentRegistry.m as
    %       obj.policies('Simple Policy (example)') = @() SimplePolicy();
    %     so the GUI's Active Policy dropdown and runTrial's Policy parameter can select it.
    %
    % To make it your own: weigh the events by their age or by the travel time, or plan over
    % two hops. RandomWalkPolicy and EDRHCPolicy (Components/AgentPolicy) are larger examples.
    methods
        function cmd = plan(~, agent, model, adj, i, ~)
            cmd = struct();                                  % stay (no free neighbor)
            idx = [model.agents.index];
            others = model.agents(idx ~= agent.index);
            busy = [others.current_target_idx];              % targets other agents hold
            free = setdiff(find(adj(i, :)), busy);
            if isempty(free), return; end
            tg = model.targets(free);
            [R, k] = max([tg.R]);
            if R == 0
                [~, k] = min([tg.lastVisitedTime]);
            end
            cmd.kind = "move";
            cmd.targetIdx = free(k);
        end
    end
end
