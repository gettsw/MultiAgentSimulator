%% 
classdef TargetCentricClusteringModule < handle
    % TARGETCENTRICCLUSTERINGMODULE Standalone Target-Centric Clustering Engine
    %
    % Implements dynamic target auctioning, target service fit scoring, cluster capacity
    % tracking, and hysteresis logic for multi-agent persistent monitoring.
    
    properties
        % Target-Centric Tuning Parameters
        alpha = 1.0;            % Target Urgency Exponent (U_i^alpha)
        beta  = 1.0;            % Cluster Capacity Exponent (P_C^beta)
        gamma = 1.5;            % Distance Decay Exponent (d_i^gamma)
        
        % Cluster & Horizon Rules
        hysteresisDelta = 0.15; % 15% improvement required to swap cluster assignment
        maxHopHorizon   = 3;    % Bidding search radius (in graph hops/distance factor)
        
        % State Memory Maps
        targetAssignments;     % Containers.Map: Target Index -> Assigned Cluster ID
        clusterScores;         % Containers.Map: Target Index -> Current Best Score
    end
    
    methods
        function obj = TargetCentricClusteringModule(alpha, beta, gamma, hysteresisDelta)
            % Constructor: Initialize maps and optional parameter overrides
            obj.targetAssignments = containers.Map('KeyType', 'int32', 'ValueType', 'int32');
            obj.clusterScores      = containers.Map('KeyType', 'int32', 'ValueType', 'double');
            
            if nargin >= 1, obj.alpha = alpha; end
            if nargin >= 2, obj.beta = beta; end
            if nargin >= 3, obj.gamma = gamma; end
            if nargin >= 4, obj.hysteresisDelta = hysteresisDelta; end
        end
        
        function [assignments, scores] = updateClusters(obj, model, adj, simTime)
            % UPDATECLUSTERS Main entry point to run target-centric auction round
            %
            % Returns:
            %   assignments - Array of length numTargets with assigned cluster/agent IDs
            %   scores      - Array of length numTargets with winning service fit scores
            
            numTargets = numel(model.targets);
            numAgents  = numel(model.agents);
            
            % 1. Get current agent positions (representing Cluster Centers)
            agentNodes = obj.extractAgentPositions(model, numAgents);
            
            % 2. Calculate All-Pairs Shortest Path Distances
            distMatrix = obj.computeShortestPaths(adj, model);
            
            % 3. Calculate Target Urgency Vector U_i
            urgency = zeros(1, numTargets);
            for i = 1:numTargets
                urgency(i) = obj.calculateTargetUrgency(model.targets(i), simTime);
            end
            
            % 4. Compute Cluster Capacities P_C_k for each Agent
            clusterCapacity = obj.computeClusterCapacities(numAgents, numTargets, urgency);
            
            % 5. Execute Auction Bidding Phase across Targets
            for i = 1:numTargets
                bestCluster = -1;
                maxScore = -1;
                
                for c = 1:numAgents
                    agNode = agentNodes(c);
                    d_min = distMatrix(agNode, i);
                    
                    % Hop/Distance Horizon Check
                    if d_min > (obj.maxHopHorizon * 5.0) && d_min > 0
                        continue;
                    end
                    
                    % Target-Centric Service Fit Score: S_{i, C_k} = (U_i^alpha * P_{C_k}^beta) / (d_{i, C_k}^gamma)
                    score = ((urgency(i))^obj.alpha * (clusterCapacity(c))^obj.beta) / ((d_min + 0.1)^obj.gamma);
                    
                    if score > maxScore
                        maxScore = score;
                        bestCluster = c;
                    end
                end
                
                % 6. Update Target Assignment with Anti-Oscillation Hysteresis
                keyI = int32(i);
                if bestCluster ~= -1
                    if ~obj.targetAssignments.isKey(keyI)
                        obj.targetAssignments(keyI) = int32(bestCluster);
                        obj.clusterScores(keyI)      = maxScore;
                    else
                        currentCluster = obj.targetAssignments(keyI);
                        currentScore   = obj.clusterScores(keyI);
                        
                        if bestCluster ~= currentCluster
                            if maxScore > (1.0 + obj.hysteresisDelta) * currentScore
                                obj.targetAssignments(keyI) = int32(bestCluster);
                                obj.clusterScores(keyI)      = maxScore;
                            end
                        else
                            obj.clusterScores(keyI) = maxScore;
                        end
                    end
                end
            end
            
            % Format Output Vectors
            assignments = zeros(1, numTargets);
            scores      = zeros(1, numTargets);
            for i = 1:numTargets
                k = int32(i);
                if obj.targetAssignments.isKey(k)
                    assignments(i) = double(obj.targetAssignments(k));
                    scores(i)      = obj.clusterScores(k);
                end
            end
        end
        
        function U_i = calculateTargetUrgency(~, targetObj, simTime)
            % CALCULATETARGETURGENCY Computes target urgency index U_i = (R_i * A_i) / B_i
            R_i = 1.0;
            if isprop(targetObj, 'R') || isfield(targetObj, 'R')
                R_i = targetObj.R;
            end
            
            B_i = 1.0;
            if isprop(targetObj, 'B') || isfield(targetObj, 'B')
                B_i = targetObj.B;
            end
            
            lastVisited = 0;
            if isprop(targetObj, 'lastVisitedTime') && ~isempty(targetObj.lastVisitedTime)
                lastVisited = targetObj.lastVisitedTime;
            end
            
            A_i = simTime - lastVisited; % Time elapsed since last service
            U_i = (R_i * A_i) / max(B_i, 1e-3);
        end
    end
    
    methods (Access = private)
        function agentNodes = extractAgentPositions(obj, model, numAgents)
            agentNodes = zeros(1, numAgents);
            for a = 1:numAgents
                ag = obj.getElement(model.agents, a);
                if isprop(ag, 'current_target_idx') && ~isempty(ag.current_target_idx)
                    agentNodes(a) = ag.current_target_idx;
                else
                    agentNodes(a) = 1;
                end
            end
        end
        
        function clusterCapacity = computeClusterCapacities(obj, numAgents, numTargets, urgency)
            clusterCapacity = zeros(1, numAgents);
            for c = 1:numAgents
                assignedUrgencySum = 0;
                for i = 1:numTargets
                    k = int32(i);
                    if obj.targetAssignments.isKey(k) && obj.targetAssignments(k) == c
                        assignedUrgencySum = assignedUrgencySum + urgency(i);
                    end
                end
                % Capacity P_{C_k} decreases as total assigned urgency increases
                clusterCapacity(c) = 1.0 / (assignedUrgencySum + 1.0);
            end
        end
        
        function distMatrix = computeShortestPaths(obj, adj, model)
            N = size(adj, 1);
            distMatrix = inf(N, N);
            
            for i = 1:N
                distMatrix(i, i) = 0;
                for j = 1:N
                    if adj(i, j) > 0
                        edge = model.findEdge(i, j);
                        distMatrix(i, j) = obj.getEdgeDistance(edge, model, i, j);
                    end
                end
            end
            
            % Floyd-Warshall shortest path distance algorithm
            for k = 1:N
                for i = 1:N
                    for j = 1:N
                        if distMatrix(i, k) + distMatrix(k, j) < distMatrix(i, j)
                            distMatrix(i, j) = distMatrix(i, k) + distMatrix(k, j);
                        end
                    end
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
        
        function elem = getElement(~, collection, idx)
            if iscell(collection)
                elem = collection{idx};
            else
                elem = collection(idx);
            end
        end
    end
end