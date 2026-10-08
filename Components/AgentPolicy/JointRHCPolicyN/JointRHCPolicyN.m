classdef JointRHCPolicyN < handle
    properties
        H_lower = 1.0;          % Lower bound for optimization window
        H_upper = 6.0;          % Upper bound for optimization window
        H0 = 1.0;               % Initial optimization guess
        
        % Communication & Local State Buffers
        inbox = struct('senderID', {}, 'curIdx', {}, 'targetIdx', {}, 'arrTime', {}, 'dwell', {}, 'priority', {});
        maxHopDistance = 1;     % Communication boundary horizon
    end
    
    methods
        % -----------------------------------------------------------------
        % COMMUNICATION INTERFACE
        % -----------------------------------------------------------------
        function receiveMessage(obj, msg)
            % Appends incoming state token to local inbox buffer
            obj.inbox(end+1) = msg;
        end
        
        function clearInbox(obj)
            obj.inbox = struct('senderID', {}, 'curIdx', {}, 'targetIdx', {}, 'arrTime', {}, 'dwell', {}, 'priority', {});
        end
        
        function msg = generateToken(obj, agent, curIdx, targetIdx, arrTime, dwell, priority)
            msg = struct(...
                'senderID', obj.getAgentID(agent), ...
                'curIdx', curIdx, ...
                'targetIdx', targetIdx, ...
                'arrTime', arrTime, ...
                'dwell', dwell, ...
                'priority', priority ...
            );
        end

        % -----------------------------------------------------------------
        % MAIN DECISION ROUTINE
        % -----------------------------------------------------------------
        function [cmd, broadcastMsg] = plan(obj, agent, model, adj, curIdx, simTime)
            cmd = struct('kind', 'move', 'targetIdx', curIdx);
            broadcastMsg = [];
            if isempty(adj) || curIdx < 1, return; end
            
            neighbors = find(adj(curIdx, :));
            if isempty(neighbors), return; end
            
            myID = obj.getAgentID(agent);
            
            % Process local inbox for co-located and 1-hop interacting agents
            coLocatedMsgs = [];
            oneHopMsgs = [];
            
            for k = 1:length(obj.inbox)
                msg = obj.inbox(k);
                if msg.curIdx == curIdx
                    coLocatedMsgs(end+1) = msg; %#ok<AGROW>
                elseif ismember(msg.curIdx, neighbors)
                    oneHopMsgs(end+1) = msg; %#ok<AGROW>
                end
            end
            
            % -------------------------------------------------------------
            % STATE 1: Scalable Co-Location Resolution (m Agents)
            % -------------------------------------------------------------
            if ~isempty(coLocatedMsgs)
                % Parse all co-located agent IDs (including self)
                allCoLocatedIDs = [myID, [coLocatedMsgs.senderID]];
                cmd = obj.planCoLocatedAuction(agent, allCoLocatedIDs, neighbors, model, curIdx, simTime);
                
                dwellVal = 2.0;
                if isprop(agent, 'dwellRemaining'), dwellVal = agent.dwellRemaining; end
                broadcastMsg = obj.generateToken(agent, curIdx, cmd.targetIdx, simTime, dwellVal, 2.0);
                return;
            end
            
            % -------------------------------------------------------------
            % STATE 2: Scalable Pairwise/Group Joint Overlap
            % -------------------------------------------------------------
            if ~isempty(oneHopMsgs)
                % Select highest-priority interacting neighbor to form joint pair
                [~, maxIdx] = max([oneHopMsgs.priority]);
                primaryPartner = oneHopMsgs(maxIdx);
                
                otherNeighbors = find(adj(primaryPartner.curIdx, :));
                sharedNeighbors = intersect(neighbors, otherNeighbors);
                
                if ~isempty(sharedNeighbors)
                    cmd = obj.planJointPairwiseDecoupled(agent, primaryPartner, neighbors, otherNeighbors, ...
                                                         sharedNeighbors, model, curIdx, primaryPartner.curIdx, simTime);
                    
                    dwellVal = 2.0;
                    if isprop(agent, 'dwellRemaining'), dwellVal = agent.dwellRemaining; end
                    broadcastMsg = obj.generateToken(agent, curIdx, cmd.targetIdx, simTime, dwellVal, 1.5);
                    return;
                end
            end
            
            % -------------------------------------------------------------
            % STATE 3: Decentralized Single-Agent Planning
            % -------------------------------------------------------------
            cmd = obj.planSingleAgentDecentralized(agent, neighbors, model, curIdx, simTime);
            
            dwellVal = 2.0;
            if isprop(agent, 'dwellRemaining'), dwellVal = agent.dwellRemaining; end
            broadcastMsg = obj.generateToken(agent, curIdx, cmd.targetIdx, simTime, dwellVal, 1.0);
        end
    end
    
    methods (Access = private)
        % =================================================================
        % STATE 1 ROUTINE: Auction/Greedy Assignment for Co-Located Agents
        % =================================================================
        function cmd = planCoLocatedAuction(obj, agent, allAgentIDs, neighbors, model, curIdx, simTime)
            cmd = struct();
            m = numel(allAgentIDs);
            n = length(neighbors);
            myID = obj.getAgentID(agent);
            
            % Deterministic rank based on ID sorting
            sortedIDs = sort(allAgentIDs);
            
            % Evaluate marginal cost reduction for each neighbor target
            marginalCosts = zeros(1, n);
            optimalDwells = zeros(1, n);
            
            for i = 1:n
                tIdx = neighbors(i);
                edge = model.findEdge(curIdx, tIdx);
                edgeDist = obj.getEdgeDistance(edge, model, curIdx, tIdx);
                cluster = obj.buildClusterTargets(curIdx, tIdx, neighbors, model.targets, simTime, edgeDist);
                
                [H_opt, J_val] = obj.optimizeClusterDwellTime(cluster);
                marginalCosts(i) = J_val;
                optimalDwells(i) = H_opt;
            end
            
            % Greedy sequential assignment by agent rank (O(m * n))
            assignedTargets = zeros(1, m);
            assignedDwells = zeros(1, m);
            availableNeighbors = 1:n;
            
            for rank = 1:m
                if isempty(availableNeighbors)
                    % Over-subscribed node: wrap around or assign least costly
                    [~, bestRelIdx] = min(marginalCosts);
                    chosenNeighborIdx = bestRelIdx;
                else
                    [~, bestLocalIdx] = min(marginalCosts(availableNeighbors));
                    chosenNeighborIdx = availableNeighbors(bestLocalIdx);
                    % Remove chosen neighbor to ensure spatial divergence
                    availableNeighbors(bestLocalIdx) = [];
                end
                
                assignedTargets(rank) = neighbors(chosenNeighborIdx);
                assignedDwells(rank) = optimalDwells(chosenNeighborIdx);
            end
            
            myRank = find(sortedIDs == myID, 1);
            cmd.kind = "move";
            cmd.targetIdx = assignedTargets(myRank);
            if isprop(agent, 'dwellRemaining')
                agent.dwellRemaining = assignedDwells(myRank);
            end
        end
        
        % =================================================================
        % STATE 2 ROUTINE: Decoupled Pairwise Optimization
        % =================================================================
        function cmd = planJointPairwiseDecoupled(obj, agent, partnerMsg, neighbors1, neighbors2, sharedNeighbors, model, curIdx1, curIdx2, simTime)
            cmd = struct();
            myID = obj.getAgentID(agent);
            isFirstAgent = (myID < partnerMsg.senderID);
            
            minCost = inf;
            bestAction1 = neighbors1(1);
            bestAction2 = neighbors2(1);
            bestDwells = [2.0, 2.0];
            
            for i = 1:length(neighbors1)
                t1 = neighbors1(i);
                e1 = model.findEdge(curIdx1, t1);
                dist1 = obj.getEdgeDistance(e1, model, curIdx1, t1);
                
                for j = 1:length(neighbors2)
                    t2 = neighbors2(j);
                    e2 = model.findEdge(curIdx2, t2);
                    dist2 = obj.getEdgeDistance(e2, model, curIdx2, t2);
                    
                    if t1 == t2 && ismember(t1, sharedNeighbors)
                        [candidateDwells, jointCost] = obj.evaluateMergeScenario(curIdx1, curIdx2, t1, dist1, dist2, ...
                                                                                 neighbors1, neighbors2, model.targets, simTime);
                    else
                        cluster1 = obj.buildClusterTargets(curIdx1, t1, neighbors1, model.targets, simTime, dist1);
                        cluster2 = obj.buildClusterTargets(curIdx2, t2, neighbors2, model.targets, simTime, dist2);
                        [candidateDwells, jointCost] = obj.optimizeDivergentClusterDwellTime(cluster1, cluster2, false);
                    end
                    
                    if jointCost < minCost
                        minCost = jointCost;
                        bestAction1 = t1;
                        bestAction2 = t2;
                        bestDwells = candidateDwells;
                    end
                end
            end
            
            cmd.kind = "move";
            if isFirstAgent
                cmd.targetIdx = bestAction1;
                if isprop(agent, 'dwellRemaining'), agent.dwellRemaining = bestDwells(1); end
            else
                cmd.targetIdx = bestAction2;
                if isprop(agent, 'dwellRemaining'), agent.dwellRemaining = bestDwells(2); end
            end
        end

        % =================================================================
        % STATE 3 ROUTINE: Inbox-Informed Single Agent Optimization
        % =================================================================
        function cmd = planSingleAgentDecentralized(obj, agent, neighbors, model, curIdx, simTime)
            cmd = struct();
            minJ = inf;
            bestNeighborIdx = [];
            optimalDwellForBest = 2.0;
            
            % Extract commitment structures directly from inbox
            otherPlans = struct('targetIdx', {}, 'arrTime', {}, 'dwell', {});
            for k = 1:length(obj.inbox)
                msg = obj.inbox(k);
                otherPlans(end+1) = struct('targetIdx', msg.targetIdx, ...
                                           'arrTime', msg.arrTime, ...
                                           'dwell', msg.dwell); %#ok<AGROW>
            end
            
            for i = 1:length(neighbors)
                neighborIdx = neighbors(i);
                edge = model.findEdge(curIdx, neighborIdx);
                edgeDist = obj.getEdgeDistance(edge, model, curIdx, neighborIdx);
                
                processedTargets = obj.buildClusterTargetsShared(curIdx, neighborIdx, neighbors, model.targets, simTime, edgeDist, otherPlans);
                
                if ~isempty(processedTargets)
                    [currentH, currentJ] = obj.optimizeClusterDwellTime(processedTargets);
                    if currentJ < minJ
                        minJ = currentJ;
                        bestNeighborIdx = neighborIdx;
                        optimalDwellForBest = currentH;
                    end
                end
            end
            
            cmd.kind = "move";
            if ~isempty(bestNeighborIdx)
                cmd.targetIdx = bestNeighborIdx;
                if isprop(agent, 'dwellRemaining'), agent.dwellRemaining = optimalDwellForBest; end
            else
                cmd.targetIdx = curIdx;
                if isprop(agent, 'dwellRemaining'), agent.dwellRemaining = 0.5; end
            end
        end

        % =================================================================
        % OPTIMIZATION & TOPOLOGY UTILITIES
        % =================================================================
        function [candidateDwells, jointCost] = evaluateMergeScenario(obj, curIdx1, curIdx2, sharedTargetIdx, dist1, dist2, n1, n2, targets, simTime)
            mockEdge1.length = dist1;
            tempVisited = Target(targets(sharedTargetIdx).index, targets(sharedTargetIdx).position);
            tempVisited.initializeAsVisited(targets(sharedTargetIdx), simTime, mockEdge1);
            
            allNeighbors = unique([n1(:); n2(:)]);
            avoidedList = {};
            for k = 1:length(allNeighbors)
                idx = allNeighbors(k);
                if idx ~= sharedTargetIdx
                    tempAvoided = Target(targets(idx).index, targets(idx).position);
                    tempAvoided.initializeAsAvoided(targets(idx), simTime);
                    avoidedList{end+1} = tempAvoided; %#ok<AGROW>
                end
            end
            
            function costVal = mergeCost(H)
                H1 = H(1); H2 = H(2);
                arr1 = simTime + dist1;
                arr2 = simTime + dist2;
                effectiveDwell = max(H1, H2) + 0.5 * min(H1, H2);
                arrOffset = abs(arr1 - arr2);
                effectiveDwell = max(effectiveDwell - arrOffset, min(H1, H2));
                
                costVal = tempVisited.objectiveVisited_numeric(effectiveDwell);
                for a = 1:length(avoidedList)
                    avoidedList{a}.objectiveAvoided_numeric(mean([H1, H2]));
                    costVal = costVal + avoidedList{a}.J_i;
                end
            end
            
            options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', 'TolFun', 1e-6, 'TolX', 1e-6);
            [H_opt, jointCost] = fmincon(@mergeCost, [obj.H0; obj.H0], [], [], [], [], [obj.H_lower; obj.H_lower], [obj.H_upper; obj.H_upper], [], options);
            candidateDwells = H_opt(:)';
        end

        function [H_opt, J_min] = optimizeDivergentClusterDwellTime(obj, cluster1, cluster2, isCoLocated)
            visited1 = []; visited2 = []; avoided1 = {}; avoided2 = {};
            for i = 1:length(cluster1)
                t = cluster1{i};
                if isprop(t, 'travelTime') && ~isempty(t.travelTime), visited1 = t; else, avoided1{end+1} = t; %#ok<AGROW>
                end
            end
            for i = 1:length(cluster2)
                t = cluster2{i};
                if isprop(t, 'travelTime') && ~isempty(t.travelTime), visited2 = t; else, avoided2{end+1} = t; %#ok<AGROW>
                end
            end
            
            function costVal = jointCost(H)
                H1 = H(1); H2 = H(2); costVal = 0;
                if isCoLocated
                    effectiveDwell = max(H1, H2) + 0.5 * min(H1, H2);
                    if ~isempty(visited1), costVal = costVal + visited1.objectiveVisited_numeric(effectiveDwell); end
                else
                    if ~isempty(visited1), costVal = costVal + visited1.objectiveVisited_numeric(H1); end
                    if ~isempty(visited2), costVal = costVal + visited2.objectiveVisited_numeric(H2); end
                end
                for k = 1:length(avoided1), avoided1{k}.objectiveAvoided_numeric(H1); costVal = costVal + avoided1{k}.J_i; end
                for k = 1:length(avoided2), avoided2{k}.objectiveAvoided_numeric(H2); costVal = costVal + avoided2{k}.J_i; end
            end
            
            options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', 'TolFun', 1e-6, 'TolX', 1e-6);
            [H_opt, J_min] = fmincon(@jointCost, [obj.H0; obj.H0], [], [], [], [], [obj.H_lower; obj.H_lower], [obj.H_upper; obj.H_upper], [], options);
            H_opt = H_opt(:)';
        end

        function [H_opt, J_min] = optimizeClusterDwellTime(obj, cluster)
            visited = []; avoided = {};
            for i = 1:length(cluster)
                t = cluster{i};
                if isprop(t, 'travelTime') && ~isempty(t.travelTime), visited = t; else, avoided{end+1} = t; %#ok<AGROW>
                end
            end
            
            function costVal = cost(H)
                costVal = 0;
                if ~isempty(visited), costVal = costVal + visited.objectiveVisited_numeric(H); end
                for k = 1:length(avoided), avoided{k}.objectiveAvoided_numeric(H); costVal = costVal + avoided{k}.J_i; end
            end
            
            options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', 'TolFun', 1e-6, 'TolX', 1e-6);
            [H_opt, J_min] = fmincon(@cost, obj.H0, [], [], [], [], obj.H_lower, obj.H_upper, [], options);
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
    end
end