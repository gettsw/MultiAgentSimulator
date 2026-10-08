classdef DivergentRHCPolicy < handle
    properties
        H_lower = 1.0;          % Lower bound for optimization window
        H_upper = 6.0;          % Upper bound for optimization window
        H0 = 1.0;               % Initial optimization guess
    end
    
    methods
        function cmd = plan(obj, agent, model, adj, curIdx, simTime)
            cmd = struct(); 
            if isempty(adj) || curIdx < 1, return; end
            
            neighbors = find(adj(curIdx, :));
            if isempty(neighbors), return; end
            
            % Handle co-located agents whether residingAgents is a cell or object array
            residing = model.targets(curIdx).residingAgents;
            if iscell(residing)
                coLocated = [residing{:}];
            else
                coLocated = residing;
            end
            
            if numel(coLocated) > 1
                cmd = obj.planJointDivergent(agent, coLocated, neighbors, model, curIdx, simTime);
            else
                cmd = obj.planSingleAgent(agent, neighbors, model, curIdx, simTime);
            end
        end
    end
    
    methods (Access = private)
        function cmd = planSingleAgent(obj, agent, neighbors, model, curIdx, simTime)
            cmd = struct();
            minJ = inf;
            bestNeighborIdx = [];
            optimalDwellForBest = 2.0;
            
            % Collect active trajectory commitments from all other agents
            otherPlans = obj.getOtherAgentCommitments(agent, model, simTime);
            
            for i = 1:length(neighbors)
                neighborIdx = neighbors(i);
                edge = model.findEdge(curIdx, neighborIdx);
                edgeDist = obj.getEdgeDistance(edge, model, curIdx, neighborIdx);
                
                % Build cluster targets with shared temporal information
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
            
            if ~isempty(bestNeighborIdx)
                cmd.kind = "move";
                cmd.targetIdx = bestNeighborIdx;
                if isprop(agent, 'dwellRemaining')
                    agent.dwellRemaining = optimalDwellForBest;
                end
            else
                cmd.kind = "move";
                cmd.targetIdx = curIdx;
                if isprop(agent, 'dwellRemaining')
                    agent.dwellRemaining = 0.5;
                end
            end
        end
        
        function plans = getOtherAgentCommitments(obj, currentAgent, model, simTime)
            % Extract committed future arrival/dwell intervals from other agents
            plans = struct('targetIdx', {}, 'arrTime', {}, 'dwell', {});
            currentID = obj.getAgentID(currentAgent);
            
            for i = 1:numel(model.agents)
                other = model.agents(i);
                if obj.getAgentID(other) == currentID, continue; end
                
                if isprop(other, 'current_target_idx') && ~isempty(other.current_target_idx)
                    tIdx = other.current_target_idx;
                    
                    % Calculate estimated arrival time and dwell
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
            
            % Primary target being evaluated for visit
            tempVisited = Target(targets(neighborIdx).index, targets(neighborIdx).position);
            tempVisited.initializeAsVisited(targets(neighborIdx), simTime, mockEdge);
            
            % Update initial uncertainty state using intermediate visits from other agents
            for p = 1:length(otherPlans)
                if otherPlans(p).targetIdx == neighborIdx
                    % Account for another agent clearing/reducing uncertainty prior to arrival
                    if otherPlans(p).arrTime < (simTime + actualEdgeLength)
                        deltaT = (simTime + actualEdgeLength) - otherPlans(p).arrTime;
                        tempVisited.lastVisitedTime = otherPlans(p).arrTime + min(deltaT, otherPlans(p).dwell);
                    end
                end
            end
            
            processedTargets{end+1} = tempVisited;
            
            % Unvisited neighbors in local cluster
            for j = 1:length(neighbors)
                otherIdx = neighbors(j);
                if otherIdx ~= neighborIdx
                    tempAvoided = Target(targets(otherIdx).index, targets(otherIdx).position);
                    tempAvoided.initializeAsAvoided(targets(otherIdx), simTime);
                    processedTargets{end+1} = tempAvoided;
                end
            end
        end

        function cmd = planJointDivergent(obj, agent, coLocatedAgents, neighbors, model, curIdx, simTime)
            cmd = struct();
            m = numel(coLocatedAgents);
            n = length(neighbors);
            
            agentIDs = zeros(1, m);
            for k = 1:m
                if iscell(coLocatedAgents)
                    aObj = coLocatedAgents{k};
                else
                    aObj = coLocatedAgents(k);
                end
                agentIDs(k) = obj.getAgentID(aObj);
            end
            
            myID = obj.getAgentID(agent);
            [sortedIDs, ~] = sort(agentIDs);
            myRank = find(sortedIDs == myID, 1);
            if isempty(myRank), myRank = 1; end
            
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
                isCoLocated = length(unique(candidateIndices)) < m;
                
                clusters = cell(1, m);
                for a = 1:m
                    tIdx = candidateIndices(a);
                    edge = model.findEdge(curIdx, tIdx);
                    edgeDist = obj.getEdgeDistance(edge, model, curIdx, tIdx);
                    clusters{a} = obj.buildClusterTargets(curIdx, tIdx, neighbors, model.targets, simTime, edgeDist);
                end
                
                if m == 2
                    [candidateDwells, jointCost] = obj.optimizeJointClusterDwellTime(clusters{1}, clusters{2}, isCoLocated);
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

        function [H_opt, J_min] = optimizeJointClusterDwellTime(obj, cluster1, cluster2, isCoLocated)
            visited1 = []; visited2 = []; avoided1 = {}; avoided2 = {};
            for i = 1:length(cluster1)
                t = cluster1{i};
                if (isprop(t, 'travelTime') && ~isempty(t.travelTime)) || (isfield(t, 'travelTime') && ~isempty(t.travelTime))
                    visited1 = t; else, avoided1{end+1} = t; %#ok<AGROW>
                end
            end
            for i = 1:length(cluster2)
                t = cluster2{i};
                if (isprop(t, 'travelTime') && ~isempty(t.travelTime)) || (isfield(t, 'travelTime') && ~isempty(t.travelTime))
                    visited2 = t; else, avoided2{end+1} = t; %#ok<AGROW>
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
                for k = 1:length(avoided1)
                    avoided1{k}.objectiveAvoided_numeric(H1); costVal = costVal + avoided1{k}.J_i;
                end
                for k = 1:length(avoided2)
                    avoided2{k}.objectiveAvoided_numeric(H2); costVal = costVal + avoided2{k}.J_i;
                end
            end
            
            options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', 'TolFun', 1e-6, 'TolX', 1e-6);
            [H_opt, J_min] = fmincon(@jointCost, [obj.H0; obj.H0], [], [], [], [], [obj.H_lower; obj.H_lower], [obj.H_upper; obj.H_upper], [], options);
            H_opt = H_opt(:)';
        end

        function [H_opt, J_min] = optimizeClusterDwellTime(obj, cluster)
            visited = []; avoided = {};
            for i = 1:length(cluster)
                t = cluster{i};
                if (isprop(t, 'travelTime') && ~isempty(t.travelTime)) || (isfield(t, 'travelTime') && ~isempty(t.travelTime))
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
            
            options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', 'TolFun', 1e-6, 'TolX', 1e-6);
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