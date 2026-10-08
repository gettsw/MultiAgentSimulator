classdef RHCPolicy < handle
    properties
        H_lower = 1.0;
        H_upper = 6.0;
        H0 = 1.0;
        table                   % ReservationTable (created per instance)
        dwellRule = "optimize"; % "optimize": fmincon on the RHC cost | "clear": dwell until R = 0
        clearDwellHere = 1.0;   % clear-rule dwell for the current plan() call
    end

    methods
        function obj = RHCPolicy(H_lower, H_upper, H0)
            if nargin > 0 && ~isempty(H_lower), obj.H_lower = H_lower; end
            if nargin > 1 && ~isempty(H_upper), obj.H_upper = H_upper; end
            if nargin > 2 && ~isempty(H0),      obj.H0 = H0;      end
            obj.table = ReservationTable();
        end

        function cmd = plan(obj, agent, model, adj, curIdx, simTime)
            cmd = struct();
            if isempty(adj) || curIdx < 1
                return;
            end

            % Safely extract agent identifier across different agent class property schemas
            agentID = obj.getAgentID(agent);

            % Clean stale global reservations & clear this agent's past plan
            obj.table.manage("clearStale", simTime);
            obj.table.manage("clearAgent", agentID);

            % BUSY GUARD: Maintain active dwell
            if isprop(agent, 'dwellRemaining') && agent.dwellRemaining > 0.05
                cmd.kind = "move";
                cmd.targetIdx = curIdx;
                obj.table.manage("reserveNode", curIdx, simTime, simTime + agent.dwellRemaining, agentID);
                return;
            end

            neighbors = find(adj(curIdx, :));
            if isempty(neighbors), return; end

            % Clear-dwell rule: the model dwells at the CURRENT node before leaving, so the dwell
            % is the time to clear this node (live R, all agents present), whatever the destination
            if obj.dwellRule == "clear"
                nHere = max(1, numel(model.targets(curIdx).residingAgents));
                obj.clearDwellHere = model.targets(curIdx).clearTimeHere(nHere, obj.H_lower, obj.H_upper);
            end

            minJ = inf;
            bestNeighborIdx = [];
            optimalDwellForBest = 2.0;
            bestTransitTime = 0.0;

            for i = 1:length(neighbors)
                neighborIdx = neighbors(i);

                edge = model.findEdge(curIdx, neighborIdx);
                if isempty(edge), continue; end

                edgeDist = obj.getEdgeDistance(edge, model, curIdx, neighborIdx);
                speed = 1.0; % Adjust speed parameter as needed
                transitTime = edgeDist / speed;

                % Dwell optimization
                processedTargets = obj.buildClusterTargets(curIdx, neighborIdx, neighbors, model.targets, simTime, edgeDist);
                if isempty(processedTargets), continue; end

                [currentH, currentJ] = obj.optimizeClusterDwellTime(processedTargets);

                % Space-Time Conflict Checks
                tArrival = simTime + transitTime;
                [occStart, occEnd] = obj.table.occupancyWindow(agent, edge, simTime, currentH, obj.H_upper);

                % Check edge (traversal) and node (arrival + dwell) availability
                if ~obj.table.manage("isEdgeFree", [curIdx, neighborIdx], simTime, tArrival, agentID)
                    continue;
                end
                if ~obj.table.manage("isNodeFree", neighborIdx, occStart, occEnd, agentID)
                    continue;
                end
                % Agents that have not planned yet (t = 0) sit on a node with no reservation
                if obj.table.isOccupiedByUnplanned(model, neighborIdx, agent, @obj.getAgentID)
                    continue;
                end

                if currentJ < minJ
                    minJ = currentJ;
                    bestNeighborIdx = neighborIdx;
                    optimalDwellForBest = currentH;
                    bestTransitTime = transitTime;
                end
            end

            % Assign Command & Lock Reservations inside RHC
            if ~isempty(bestNeighborIdx)
                cmd.kind = "move";
                cmd.targetIdx = bestNeighborIdx;

                tArrival = simTime + bestTransitTime;
                [occStart, occEnd] = obj.table.occupancyWindow(agent, model.findEdge(curIdx, bestNeighborIdx), ...
                    simTime, optimalDwellForBest, obj.H_upper);

                % Reserve where the agent will REALLY be: it dwells at curIdx first, then
                % travels at its real speed and sits at the destination until its next dwell ends
                obj.table.manage("reserveEdge", [curIdx, bestNeighborIdx], simTime, tArrival, agentID);
                obj.table.manage("reserveNode", curIdx, simTime, simTime + optimalDwellForBest, agentID);
                obj.table.manage("reserveNode", bestNeighborIdx, occStart, occEnd, agentID);

                if isprop(agent, 'dwellRemaining')
                    agent.dwellRemaining = optimalDwellForBest;
                end
            else
                % FALLBACK: Hold current node if all targets are reserved
                % ponytail: an agent stuck here past its reserved window can
                % overlap an agent that reserved this node for later; add yielding if seen in practice
                holdTime = 0.2;
                cmd.kind = "move";
                cmd.targetIdx = curIdx;

                obj.table.manage("reserveNode", curIdx, simTime, simTime + holdTime, agentID);

                if isprop(agent, 'dwellRemaining')
                    agent.dwellRemaining = holdTime;
                end
            end
        end

        function resetReservations(obj)
            % Call this to clear static reservations when starting a fresh run
            obj.table.manage("resetAll", 0);
        end
    end

    methods (Access = private)
        function agentID = getAgentID(~, agent)
            if isprop(agent, 'id') && ~isempty(agent.id)
                agentID = uint64(agent.id);
            elseif isprop(agent, 'ID') && ~isempty(agent.ID)
                agentID = uint64(agent.ID);
            elseif isprop(agent, 'index') && ~isempty(agent.index)
                agentID = uint64(agent.index);
            elseif isprop(agent, 'agentID') && ~isempty(agent.agentID)
                agentID = uint64(agent.agentID);
            else
                % Unique fallback handle address if no identifier property exists
                agentID = uint64(feature('GetHandleAddress', agent));
            end
        end

        function edgeDist = getEdgeDistance(~, edge, model, curIdx, neighborIdx)
            if isprop(edge, 'weight') || isfield(edge, 'weight')
                edgeDist = edge.weight;
            elseif isprop(edge, 'distance') || isfield(edge, 'distance')
                edgeDist = edge.distance;
            elseif isprop(edge, 'length') || isfield(edge, 'length')
                edgeDist = edge.length;
            else
                edgeDist = norm(model.targets(neighborIdx).position - model.targets(curIdx).position);
            end
        end

        function [H_opt, J_min] = optimizeClusterDwellTime(obj, cluster)
            visited = []; avoided = {};
            for i = 1:length(cluster)
                t = cluster{i};
                if isprop(t, 'travelTime') && ~isempty(t.travelTime)
                    visited = t;
                else
                    avoided{end+1} = t; %#ok<AGROW>
                end
            end

            function costVal = cost(H)
                costVal = 0;
                if ~isempty(visited)
                    costVal = costVal + visited.objectiveVisited_numeric(H);
                end
                for k = 1:length(avoided)
                    avoided{k}.objectiveAvoided_numeric(H);
                    costVal = costVal + avoided{k}.J_i;
                end
            end

            if obj.dwellRule == "clear" && ~isempty(visited)
                H_opt = obj.clearDwellHere;
                J_min = cost(H_opt);
                return;
            end
            options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', ...
                'TolFun', 1e-6, 'TolX', 1e-6);
            [H_opt, J_min] = fmincon(@cost, obj.H0, [], [], [], [], obj.H_lower, obj.H_upper, [], options);
        end

        function processedTargets = buildClusterTargets(~, ~, neighborIdx, neighbors, targets, simTime, actualEdgeLength)
            processedTargets = {};
            mockEdge.length = actualEdgeLength;

            tempVisited = Target(targets(neighborIdx).index, targets(neighborIdx).position);
            tempVisited.initializeAsVisited(targets(neighborIdx), simTime, mockEdge);
            processedTargets{end+1} = tempVisited;

            for j = 1:length(neighbors)
                otherIdx = neighbors(j);
                if otherIdx ~= neighborIdx
                    tempAvoided = Target(targets(otherIdx).index, targets(otherIdx).position);
                    tempAvoided.initializeAsAvoided(targets(otherIdx), simTime);
                    processedTargets{end+1} = tempAvoided;
                end
            end
        end
    end
end