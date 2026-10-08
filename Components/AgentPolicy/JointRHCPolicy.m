classdef JointRHCPolicy < handle
    properties
        H_lower = 1.0;          % Lower bound for optimization window
        H_upper = 6.0;          % Upper bound for optimization window
        H0 = 1.0;               % Initial optimization guess
        table                   % ReservationTable: blocks accidental convergence (merges are explicit)

        % Ablation switches (set from experiments via Policy_<name> parameters)
        useMergeDiverge = true;  % false: skip the pairwise planner, plan as single agents
        useReservations = true;  % false: ignore/skip reservations entirely
        divergeHorizon = "own";  % avoided-target horizon when diverging: "own" = H1, "max" = max(H1,H2)
        mergeGate = "off";       % which merges to allow:
                                 %  "useful":   R still > 0 after the second arrival (simulated schedule)
                                 %  "capacity": only targets whose max growth can exceed one agent's decay
                                 %  "off":      every merge the cost prefers
        divergeScoring = "pair"; % "classical": diverge options scored exactly like Classical RHC
                                      % "pair": old pairCost bookkeeping (partner target excluded)

        dwellRule = "clear";     % DEFAULT "clear": dwell until this node's R = 0 (best in ablation)
                                 % "optimize": fmincon on the RHC cost (set this to switch back)
        clearDwellHere = 1.0;    % clear-rule dwell for the current plan() call
        shadowCompare = false;   % diagnostics: also evaluate the other divergeScoring, record disagreements

        % Decision counters (diagnostics), cleared with the reservations
        stats = struct('coLocated', 0, 'merge', 0, 'diverge', 0, 'pairHold', 0, 'single', 0, 'singleHold', 0, ...
            'dwellN', 0, 'dwellSum', 0, 'dwellAtLower', 0, 'dwellAtUpper', 0, ...
            'shadowN', 0, 'shadowTargetDiff', 0, 'shadowMergeDiff', 0, 'shadowDwellSum', 0, 'shadowChosenDwellSum', 0);
    end

    methods
        function obj = JointRHCPolicy()
            obj.table = ReservationTable();
        end

        function resetReservations(obj)
            obj.table.manage("resetAll", 0);
            f = fieldnames(obj.stats);
            for i = 1:numel(f), obj.stats.(f{i}) = 0; end
        end

        function cmd = plan(obj, agent, model, adj, curIdx, simTime)
            cmd = struct();
            if isempty(adj) || curIdx < 1, return; end

            neighbors = find(adj(curIdx, :));
            if isempty(neighbors), return; end

            % Clear-dwell rule: the model dwells at the CURRENT node before leaving, so the dwell
            % is the time to clear this node (live R, all agents present), whatever the destination
            if obj.dwellRule == "clear"
                nHere = max(1, numel(model.targets(curIdx).residingAgents));
                obj.clearDwellHere = model.targets(curIdx).clearTimeHere(nHere, obj.H_lower, obj.H_upper);
            end

            % Drop expired reservations and this agent's previous plan
            obj.table.manage("clearStale", simTime);
            obj.table.manage("clearAgent", obj.getAgentID(agent));

            % -------------------------------------------------------------
            % STATE 1: Check Co-location at Current Node
            % -------------------------------------------------------------
            residing = model.targets(curIdx).residingAgents;
            if iscell(residing)
                coLocated = [residing{:}];
            else
                coLocated = residing;
            end

            if numel(coLocated) > 1
                % Co-located agents MUST trigger divergent planning to split up
                obj.stats.coLocated = obj.stats.coLocated + 1;
                cmd = obj.planCoLocatedDivergent(agent, coLocated, neighbors, model, curIdx, simTime);
                return;
            end

            % -------------------------------------------------------------
            % STATE 2: Check Neighbor Overlap with Nearby Agents
            % -------------------------------------------------------------
            [otherAgent, otherDestIdx] = obj.findNearbyInteractingAgent(agent, model, neighbors, curIdx);

            if ~isempty(otherAgent) && obj.useMergeDiverge
                % Other agent is committed to a node we can reach: Merge (a1 = its target) vs Diverge
                cmd = obj.planJointMergeDivergent(agent, otherAgent, otherDestIdx, neighbors, model, curIdx, simTime);
                return;
            end

            % -------------------------------------------------------------
            % STATE 3: Default Single-Agent Planning with Commitments
            % -------------------------------------------------------------
            cmd = obj.planSingleAgent(agent, neighbors, model, curIdx, simTime);
        end
    end

    methods (Access = private)
        % =================================================================
        % STATE 1 ROUTINE: Co-Located Divergent Planner
        % =================================================================
        function cmd = planCoLocatedDivergent(obj, agent, coLocatedAgents, neighbors, model, curIdx, simTime)
            cmd = struct();
            m = numel(coLocatedAgents);
            n = length(neighbors);

            agentIDs = zeros(1, m);
            for k = 1:m
                aObj = obj.getElement(coLocatedAgents, k);
                agentIDs(k) = obj.getAgentID(aObj);
            end

            myID = obj.getAgentID(agent);
            [sortedIDs, ~] = sort(agentIDs);
            myRank = find(sortedIDs == myID, 1);
            if isempty(myRank), myRank = 1; end

            % Force unique/divergent neighbor assignments when n >= m
            if n >= m
                combos = nchoosek(1:n, m);
                permsList = [];
                for i = 1:size(combos, 1)
                    permsList = [permsList; perms(combos(i, :))]; %#ok<AGROW>
                end
            else
                gridArgs = cell(1, m);
                [gridArgs{:}] = ndgrid(1:n);
                permsList = zeros(n^m, m);
                for k = 1:m
                    permsList(:, k) = gridArgs{k}(:);
                end
            end

            minJointCost = inf;
            bestJointAction = repmat(neighbors(1), 1, m);
            bestDwells = repmat(2.0, 1, m);

            for p = 1:size(permsList, 1)
                candidateIndices = neighbors(permsList(p, :));

                dists = zeros(1, m);
                clusters = cell(1, m);
                for a = 1:m
                    tIdx = candidateIndices(a);
                    edge = model.findEdge(curIdx, tIdx);
                    dists(a) = obj.getEdgeDistance(edge, model, curIdx, tIdx);
                    clusters{a} = obj.buildClusterTargets(curIdx, tIdx, neighbors, model.targets, simTime, dists(a));
                end

                if m == 2
                    [candidateDwells, jointCost] = obj.pairCost(neighbors, candidateIndices(1), dists(1), ...
                        candidateIndices(2), dists(2), model.targets, simTime);
                else
                    jointCost = 0;
                    candidateDwells = zeros(1, m);
                    for a = 1:m
                        [H_opt, J_val] = obj.optimizeClusterDwellTime(clusters{a});
                        jointCost = jointCost + J_val;
                        candidateDwells(a) = H_opt;
                    end
                end

                if jointCost < minJointCost
                    minJointCost = jointCost;
                    bestJointAction = candidateIndices;
                    bestDwells = candidateDwells;
                end
            end

            cmd.kind = "move";
            cmd.targetIdx = bestJointAction(myRank);
            if isprop(agent, 'dwellRemaining')
                agent.dwellRemaining = bestDwells(myRank);
            end
            % ponytail: split targets are reserved but not checked against agents outside the
            % co-located group; filter permsList with isFree if that ever causes convergence
            obj.commit(agent, model, curIdx, cmd.targetIdx, bestDwells(myRank), simTime);
        end

        % =================================================================
        % STATE 2 ROUTINE: Merge onto the other agent's committed target vs Diverge
        % =================================================================
        function cmd = planJointMergeDivergent(obj, agent, otherAgent, otherDestIdx, neighbors, model, curIdx, simTime)
            % Only this agent decides. The other agent is already committed to otherDestIdx,
            % so its action is fixed and we only choose a1 (a1 == otherDestIdx means merge).
            [bestAction, bestDwell, minCost] = obj.chooseJointAction(obj.divergeScoring, agent, otherAgent, ...
                otherDestIdx, neighbors, model, curIdx, simTime);

            if obj.shadowCompare && ~isinf(minCost)
                % Diagnostics only: what would the other scoring have picked here?
                alt = "pair";
                if obj.divergeScoring == "pair", alt = "classical"; end
                [altAction, altDwell, altCost] = obj.chooseJointAction(alt, agent, otherAgent, ...
                    otherDestIdx, neighbors, model, curIdx, simTime);
                if ~isinf(altCost)
                    obj.stats.shadowN = obj.stats.shadowN + 1;
                    obj.stats.shadowTargetDiff = obj.stats.shadowTargetDiff + (altAction ~= bestAction);
                    obj.stats.shadowMergeDiff = obj.stats.shadowMergeDiff + ...
                        ((altAction == otherDestIdx) ~= (bestAction == otherDestIdx));
                    obj.stats.shadowDwellSum = obj.stats.shadowDwellSum + altDwell;
                    obj.stats.shadowChosenDwellSum = obj.stats.shadowChosenDwellSum + bestDwell;
                end
            end

            if isinf(minCost)
                obj.stats.pairHold = obj.stats.pairHold + 1;
                cmd = obj.holdPosition(agent, curIdx, simTime);
                return;
            end
            if bestAction == otherDestIdx
                obj.stats.merge = obj.stats.merge + 1;
            else
                obj.stats.diverge = obj.stats.diverge + 1;
            end
            cmd = struct('kind', "move", 'targetIdx', bestAction);
            if isprop(agent, 'dwellRemaining'), agent.dwellRemaining = bestDwell; end
            obj.commit(agent, model, curIdx, bestAction, bestDwell, simTime);
        end

        function [bestAction, bestDwell, minCost] = chooseJointAction(obj, scoring, agent, otherAgent, ...
                otherDestIdx, neighbors, model, curIdx, simTime)
            % Best a1 among neighbors under the given diverge scoring; merge = a1 == otherDestIdx
            dOther = norm(otherAgent.state.pos - model.targets(otherDestIdx).position);
            minCost = inf;
            bestAction = neighbors(1);
            bestDwell = 2.0;
            for i = 1:length(neighbors)
                t1 = neighbors(i);
                d1 = obj.getEdgeDistance(model.findEdge(curIdx, t1), model, curIdx, t1);
                if t1 ~= otherDestIdx && scoring == "classical"
                    % Partner's action is fixed, so its terms are the same for every diverge
                    % option: score like Classical RHC. Merge (below) is that same cost with 2B.
                    [h, jointCost] = obj.optimizeClusterDwellTime( ...
                        obj.buildClusterTargets(curIdx, t1, neighbors, model.targets, simTime, d1));
                    dwells = [h, h];
                else
                    [dwells, jointCost] = obj.pairCost(neighbors, t1, d1, otherDestIdx, dOther, model.targets, simTime);
                end
                % Merging onto the other agent's target is a deliberate choice; anything else must be free
                if t1 ~= otherDestIdx && ~obj.isFree(agent, model, curIdx, t1, dwells(1), simTime)
                    continue;
                end
                % Skip useless merges: the partner alone drives R to 0 before we get there
                if t1 == otherDestIdx && ~obj.mergeAllowed(agent, otherAgent, model, curIdx, t1, dwells(1), simTime)
                    continue;
                end
                if jointCost < minCost
                    minCost = jointCost;
                    bestAction = t1;
                    bestDwell = dwells(1);
                end
            end
        end

        % =================================================================
        % RESERVATION HELPERS (same rules as Classical RHC)
        % =================================================================
        function tf = isFree(obj, agent, model, curIdx, t, dwellHere, simTime)
            edge = model.findEdge(curIdx, t);
            if isempty(edge), tf = false; return; end
            if ~obj.useReservations, tf = true; return; end
            [s0, s1] = obj.table.occupancyWindow(agent, edge, simTime, dwellHere, obj.H_upper);
            id = obj.getAgentID(agent);
            tf = obj.table.manage("isNodeFree", t, s0, s1, id) && ...
                 ~obj.table.isOccupiedByUnplanned(model, t, agent, @obj.getAgentID);
        end

        function tf = mergeAllowed(obj, agent, partner, model, curIdx, t, myDwellHere, simTime)
            switch obj.mergeGate
                case "off"
                    tf = true;
                case "capacity"
                    tgt = model.targets(t);
                    tf = tgt.maxGrowthRate(model.uncertaintyMode) >= tgt.B;
                otherwise
                    tf = obj.mergeIsUseful(agent, partner, model, curIdx, t, myDwellHere, simTime);
            end
        end

        function tf = mergeIsUseful(obj, agent, partner, model, curIdx, t, myDwellHere, simTime)
            % A second agent only helps if the first one alone cannot keep R at 0 once both could
            % be there. Timeline (model dwells BEFORE leaving): I dwell here, then travel to t;
            % the partner finishes any dwell and travels the rest of its path.
            % Simulate R with the target's real spike schedule: growth A(t), one agent's decay
            % from the first arrival, clamped at 0. Useful if R > 0 at any point between the
            % second arrival and a full dwell (H_upper) after it.
            % ponytail: assumes the first agent stays for the whole window; fine at dwell scale
            myArrival = myDwellHere + obj.pathLength(model.findEdge(curIdx, t).curvePoints) / agent.state.maxSpeed;
            partnerArrival = partner.dwellRemaining;
            if ~isempty(partner.path)
                rest = [partner.state.pos; partner.path(partner.pathIndex:end, :)];
                partnerArrival = partnerArrival + obj.pathLength(rest) / partner.state.maxSpeed;
            end
            tgt = model.targets(t);
            tFirst = min(myArrival, partnerArrival);
            tSecond = max(myArrival, partnerArrival);
            step = 0.1;
            R = tgt.R;
            tf = false;
            for tau = 0:step:(tSecond + obj.H_upper)
                A = tgt.growthRateAt(simTime + tau, model.uncertaintyMode);
                R = max(0, R + (A - tgt.B * (tau >= tFirst)) * step);
                if tau >= tSecond && R > 0
                    tf = true;
                    return;
                end
            end
        end

        function L = pathLength(~, pts)
            L = sum(vecnorm(diff(pts, 1, 1), 2, 2));
        end

        function commit(obj, agent, model, curIdx, t, dwellHere, simTime)
            % Reserve the dwell at curIdx and the real occupancy window at the destination
            obj.stats.dwellN = obj.stats.dwellN + 1;
            obj.stats.dwellSum = obj.stats.dwellSum + dwellHere;
            obj.stats.dwellAtLower = obj.stats.dwellAtLower + (dwellHere <= obj.H_lower + 1e-3);
            obj.stats.dwellAtUpper = obj.stats.dwellAtUpper + (dwellHere >= obj.H_upper - 1e-3);
            if ~obj.useReservations, return; end
            id = obj.getAgentID(agent);
            obj.table.manage("reserveNode", curIdx, simTime, simTime + dwellHere, id);
            edge = model.findEdge(curIdx, t);
            if isempty(edge), return; end
            [s0, s1] = obj.table.occupancyWindow(agent, edge, simTime, dwellHere, obj.H_upper);
            obj.table.manage("reserveNode", t, s0, s1, id);
        end

        function cmd = holdPosition(obj, agent, curIdx, simTime)
            % Every candidate is reserved: wait briefly at the current node
            holdTime = 0.5;
            cmd = struct('kind', "move", 'targetIdx', curIdx);
            if isprop(agent, 'dwellRemaining'), agent.dwellRemaining = holdTime; end
            obj.table.manage("reserveNode", curIdx, simTime, simTime + holdTime, obj.getAgentID(agent));
        end

        % =================================================================
        % Shared merge/diverge cost: both options score the SAME target set,
        % each target counted exactly once (visited or avoided)
        % =================================================================
        function [dwells, J] = pairCost(obj, candidateSet, t1, d1, t2, d2, targets, simTime)
            S = unique([candidateSet(:); t1; t2])';
            options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', 'TolFun', 1e-6, 'TolX', 1e-6);

            if t1 == t2
                % MERGE: both agents decay the same target -> 2B while present.
                % ponytail: assumes simultaneous arrival (min distance). max() (later arrival) was
                % tried and killed merges in Dynamic mode; a piecewise B -> 2B model is the real fix
                v = obj.makeVisited(targets, t1, min(d1, d2), simTime);
                v.B_val = 2 * v.B_val;
                avoided = obj.makeAvoided(targets, setdiff(S, t1), simTime);
                mergeCost = @(H) v.objectiveVisited_numeric(H) + obj.avoidedCost(avoided, H);
                if obj.dwellRule == "clear"
                    H = obj.clearDwellHere;
                    J = mergeCost(H);
                else
                    [H, J] = fmincon(mergeCost, obj.H0, [], [], [], [], obj.H_lower, obj.H_upper, [], options);
                end
                dwells = [H, H];
            else
                % DIVERGE: each agent visits its own target, the rest are avoided once
                v1 = obj.makeVisited(targets, t1, d1, simTime);
                v2 = obj.makeVisited(targets, t2, d2, simTime);
                avoided = obj.makeAvoided(targets, setdiff(S, [t1 t2]), simTime);
                % Avoided horizon is the planner's own dwell H(1): with max(H1,H2) the optimizer
                % could inflate H2 and make H1 ignore neglected neighbors entirely
                if obj.divergeHorizon == "max"
                    horizon = @(H) max(H(1), H(2));
                else
                    horizon = @(H) H(1);
                end
                costFn = @(H) v1.objectiveVisited_numeric(H(1)) + v2.objectiveVisited_numeric(H(2)) + ...
                    obj.avoidedCost(avoided, horizon(H));
                if obj.dwellRule == "clear"
                    H = [obj.clearDwellHere; obj.clearDwellHere];
                    J = costFn(H);
                else
                    [H, J] = fmincon(costFn, [obj.H0; obj.H0], [], [], [], [], ...
                        [obj.H_lower; obj.H_lower], [obj.H_upper; obj.H_upper], [], options);
                end
                dwells = H(:)';
            end
        end

        function v = makeVisited(~, targets, idx, dist, simTime)
            mockEdge.length = dist;
            v = Target(targets(idx).index, targets(idx).position);
            v.initializeAsVisited(targets(idx), simTime, mockEdge);
        end

        function list = makeAvoided(~, targets, idxs, simTime)
            list = cell(1, numel(idxs));
            for k = 1:numel(idxs)
                list{k} = Target(targets(idxs(k)).index, targets(idxs(k)).position);
                list{k}.initializeAsAvoided(targets(idxs(k)), simTime);
            end
        end

        function c = avoidedCost(~, avoided, H)
            c = 0;
            for k = 1:numel(avoided), c = c + avoided{k}.objectiveAvoided_numeric(H); end
        end

        function [H_opt, J_min] = optimizeClusterDwellTime(obj, cluster)
            visited = []; avoided = {};
            for i = 1:length(cluster)
                t = cluster{i};
                if isprop(t, 'travelTime') && ~isempty(t.travelTime)
                    visited = t; else, avoided{end+1} = t; %#ok<AGROW>
                end
            end

            function costVal = cost(H)
                costVal = 0;
                if ~isempty(visited), costVal = costVal + visited.objectiveVisited_numeric(H); end
                for k = 1:length(avoided)
                    avoided{k}.objectiveAvoided_numeric(H); costVal = costVal + avoided{k}.J_i;
                end
            end

            if obj.dwellRule == "clear" && ~isempty(visited)
                H_opt = obj.clearDwellHere;
                J_min = cost(H_opt);
                return;
            end
            options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', 'TolFun', 1e-6, 'TolX', 1e-6);
            [H_opt, J_min] = fmincon(@cost, obj.H0, [], [], [], [], obj.H_lower, obj.H_upper, [], options);
        end

        % =================================================================
        % STATE 3 ROUTINE: Single-Agent Planning
        % =================================================================
        function cmd = planSingleAgent(obj, agent, neighbors, model, curIdx, simTime)
            cmd = struct();
            minJ = inf;
            bestNeighborIdx = [];
            optimalDwellForBest = 2.0;

            otherPlans = obj.getOtherAgentCommitments(agent, model, simTime);

            for i = 1:length(neighbors)
                neighborIdx = neighbors(i);
                edge = model.findEdge(curIdx, neighborIdx);
                edgeDist = obj.getEdgeDistance(edge, model, curIdx, neighborIdx);

                processedTargets = obj.buildClusterTargetsShared(curIdx, neighborIdx, neighbors, model.targets, simTime, edgeDist, otherPlans);

                if ~isempty(processedTargets)
                    [currentH, currentJ] = obj.optimizeClusterDwellTime(processedTargets);
                    if ~obj.isFree(agent, model, curIdx, neighborIdx, currentH, simTime)
                        continue;
                    end
                    if currentJ < minJ
                        minJ = currentJ;
                        bestNeighborIdx = neighborIdx;
                        optimalDwellForBest = currentH;
                    end
                end
            end

            if ~isempty(bestNeighborIdx)
                cmd.kind = "move";
                cmd.targetIdx = bestNeighborIdx;
                if isprop(agent, 'dwellRemaining')
                    agent.dwellRemaining = optimalDwellForBest;
                end
                obj.commit(agent, model, curIdx, bestNeighborIdx, optimalDwellForBest, simTime);
                obj.stats.single = obj.stats.single + 1;
            else
                obj.stats.singleHold = obj.stats.singleHold + 1;
                cmd = obj.holdPosition(agent, curIdx, simTime);
            end
        end

        % =================================================================
        % TOPOLOGY & AGENT INTERACTION UTILITIES
        % =================================================================
        function [otherAgent, otherIdx] = findNearbyInteractingAgent(obj, currentAgent, model, neighbors, curIdx)
            % Returns another agent whose committed destination is 1 hop from curIdx
            otherAgent = []; otherIdx = [];
            myID = obj.getAgentID(currentAgent);

            for i = 1:numel(model.agents)
                other = obj.getElement(model.agents, i);
                if obj.getAgentID(other) == myID, continue; end

                if isprop(other, 'current_target_idx') && ~isempty(other.current_target_idx)
                    oIdx = other.current_target_idx;
                    if oIdx ~= curIdx && any(neighbors == oIdx)
                        otherAgent = other;
                        otherIdx = oIdx;
                        return;
                    end
                end
            end
        end

        function plans = getOtherAgentCommitments(obj, currentAgent, model, simTime)
            plans = struct('targetIdx', {}, 'arrTime', {}, 'dwell', {});
            currentID = obj.getAgentID(currentAgent);

            for i = 1:numel(model.agents)
                other = obj.getElement(model.agents, i);
                if obj.getAgentID(other) == currentID, continue; end

                if isprop(other, 'current_target_idx') && ~isempty(other.current_target_idx)
                    tIdx = other.current_target_idx;
                    remTravel = 0;
                    if isprop(other, 'timeToTarget') && ~isempty(other.timeToTarget)
                        remTravel = other.timeToTarget;
                    end
                    dwellVal = 2.0;
                    if isprop(other, 'dwellRemaining') && ~isempty(other.dwellRemaining)
                        dwellVal = other.dwellRemaining;
                    end

                    plans(end+1) = struct('targetIdx', tIdx, ...
                                          'arrTime', simTime + remTravel, ...
                                          'dwell', dwellVal); %#ok<AGROW>
                end
            end
        end

        function processedTargets = buildClusterTargetsShared(~, ~, neighborIdx, neighbors, targets, simTime, actualEdgeLength, otherPlans)
            processedTargets = {};
            mockEdge.length = actualEdgeLength;

            tempVisited = Target(targets(neighborIdx).index, targets(neighborIdx).position);
            tempVisited.initializeAsVisited(targets(neighborIdx), simTime, mockEdge);

            for p = 1:length(otherPlans)
                if otherPlans(p).targetIdx == neighborIdx
                    if otherPlans(p).arrTime < (simTime + actualEdgeLength)
                        deltaT = (simTime + actualEdgeLength) - otherPlans(p).arrTime;
                        tempVisited.lastVisitedTime = otherPlans(p).arrTime + min(deltaT, otherPlans(p).dwell);
                    end
                end
            end

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

        function edgeDist = getEdgeDistance(~, edge, model, curIdx, targetIdx)
            if isempty(edge)
                edgeDist = sqrt(sum((model.targets(targetIdx).position - model.targets(curIdx).position).^2));
            elseif isprop(edge, 'weight') || isfield(edge, 'weight')
                edgeDist = edge.weight;
            elseif isprop(edge, 'distance') || isfield(edge, 'distance')
                edgeDist = edge.distance;
            elseif isprop(edge, 'length') || isfield(edge, 'length')
                edgeDist = edge.length;
            else
                edgeDist = sqrt(sum((model.targets(targetIdx).position - model.targets(curIdx).position).^2));
            end
        end

        function idVal = getAgentID(~, agentObj)
            if isprop(agentObj, 'index') || isfield(agentObj, 'index')
                idVal = agentObj.index;
            elseif isprop(agentObj, 'id') || isfield(agentObj, 'id')
                idVal = agentObj.id;
            else
                idVal = 1;
            end
        end

        function elem = getElement(~, collection, idx)
            if iscell(collection)
                elem = collection{idx};
            else
                elem = collection(idx);
            end
        end
    end
end
