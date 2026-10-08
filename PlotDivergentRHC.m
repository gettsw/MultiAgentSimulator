%% plot_Divergent_RHC_Optimization.m
% Visualizes the Joint Objective Function Surface / Heatmap for Divergent RHC
% demonstrating how co-location penalties force co-located agents to split.

clear; clc; close all;

%% 1. Mock Environment & Policy Setup
policy = DivergentRHCPolicy();

% Create mock target node positions
targetNodes = struct('index', {}, 'position', {}, 'residingAgents', {});
numNeighbors = 5; % 5 candidate neighbor targets (1 to 5)

for k = 1:numNeighbors
    targetNodes(k).index = k;
    % Distribute mock targets in a circle
    theta = 2*pi*(k-1)/numNeighbors;
    targetNodes(k).position = [10*cos(theta), 10*sin(theta)];
    targetNodes(k).residingAgents = [];
end

% Create 2 co-located mock agents on current node
agent1 = struct('id', 1, 'dwellRemaining', 0);
agent2 = struct('id', 2, 'dwellRemaining', 0);
coLocatedAgents = [agent1, agent2];

neighbors = 1:numNeighbors;
curIdx = 1;
simTime = 0;

% Mock Model Interface
model.targets = targetNodes;
model.findEdge = @(from, to) struct('length', sqrt(sum((targetNodes(to).position - targetNodes(from).position).^2)));

%% 2. Evaluate Joint Cost Surface J(u1, u2)
m = length(coLocatedAgents); % 2 agents
n = length(neighbors);

J_matrix = zeros(n, n);      % Full cost including co-location penalty
J_base_matrix = zeros(n, n); % Raw RHC cost without penalty

for i = 1:n
    for j = 1:n
        u1 = neighbors(i);
        u2 = neighbors(j);
        candidateIndices = [u1, u2];
        
        jointCost = 0;
        
        % Compute RHC uncertainty cost for Agent 1 -> u1
        edge1 = model.findEdge(curIdx, u1);
        edgeDist1 = edge1.length;
        processed1 = buildMockClusterTargets(curIdx, u1, neighbors, targetNodes, simTime, edgeDist1);
        [~, J_val1] = optimizeMockClusterDwellTime(policy, processed1);
        
        % Compute RHC uncertainty cost for Agent 2 -> u2
        edge2 = model.findEdge(curIdx, u2);
        edgeDist2 = edge2.length;
        processed2 = buildMockClusterTargets(curIdx, u2, neighbors, targetNodes, simTime, edgeDist2);
        [~, J_val2] = optimizeMockClusterDwellTime(policy, processed2);
        
        jointCost = J_val1 + J_val2;
        J_base_matrix(i, j) = jointCost;
        
        % Apply Co-location Divergent Penalty on Diagonal (u1 == u2)
        if length(unique(candidateIndices)) < m
            jointCost = jointCost + policy.colocationPenalty;
        end
        
        J_matrix(i, j) = jointCost;
    end
end

%% 3. Locate Optimal Divergent Pair (u1*, u2*)
[minVal, minLinearIdx] = min(J_matrix(:));
[best_u1_idx, best_u2_idx] = ind2sub(size(J_matrix), minLinearIdx);

best_u1 = neighbors(best_u1_idx);
best_u2 = neighbors(best_u2_idx);

fprintf('--- DIVERGENT RHC OPTIMIZATION RESULT ---\n');
fprintf('Optimal Joint Action: Agent 1 -> Target %d, Agent 2 -> Target %d\n', best_u1, best_u2);
fprintf('Minimum Joint Cost: %.4f (Base Cost: %.4f)\n', minVal, J_base_matrix(best_u1_idx, best_u2_idx));

%% 4. Plotting
figure('Color', [1 1 1], 'Position', [100, 100, 1100, 480]);

% --- Plot 1: Raw RHC Joint Cost (Without Penalty) ---
subplot(1, 2, 1);
b1 = bar3(J_base_matrix);
for k = 1:length(b1)
    zdata = b1(k).ZData;
    b1(k).CData = zdata;
    b1(k).FaceColor = 'interp';
end
title('Raw Joint RHC Uncertainty Cost $J(u_1, u_2)$', 'Interpreter', 'latex', 'FontSize', 12);
xlabel('Agent 2 Choice ($u_2$)', 'Interpreter', 'latex');
ylabel('Agent 1 Choice ($u_1$)', 'Interpreter', 'latex');
zlabel('Joint Uncertainty Cost', 'Interpreter', 'latex');
xticks(1:n); xticklabels(arrayfun(@(x) sprintf('Node %d', x), neighbors, 'UniformOutput', false));
yticks(1:n); yticklabels(arrayfun(@(x) sprintf('Node %d', x), neighbors, 'UniformOutput', false));
view(-37.5, 30);
grid on;

% --- Plot 2: Divergent RHC Cost Matrix (With Penalty & Split Marker) ---
subplot(1, 2, 2);
imagesc(J_base_matrix); % Plot base values visually for clear dynamic range
colormap(parula);
colorbar;
hold on;

% Mask diagonal to highlight penalized merged states
diagIdx = 1:n;
plot(diagIdx, diagIdx, 'rx', 'LineWidth', 2.5, 'MarkerSize', 12, 'DisplayName', 'Penalized Merging (u1 = u2)');

% Highlight Optimal Divergent Choice
plot(best_u2_idx, best_u1_idx, 'gp', 'MarkerSize', 16, 'MarkerFaceColor', 'g', ...
    'LineWidth', 1.5, 'DisplayName', sprintf('Optimal Split (u1*=%d, u2*=%d)', best_u1, best_u2));

title('Divergent Decision Space (Diagonal Penalized)', 'Interpreter', 'latex', 'FontSize', 12);
xlabel('Agent 2 Target Choice ($u_2$)', 'Interpreter', 'latex');
ylabel('Agent 1 Target Choice ($u_1$)', 'Interpreter', 'latex');
xticks(1:n); xticklabels(arrayfun(@(x) sprintf('Node %d', x), neighbors, 'UniformOutput', false));
yticks(1:n); yticklabels(arrayfun(@(x) sprintf('Node %d', x), neighbors, 'UniformOutput', false));
legend('Location', 'northeastoutside');
axis square;
hold off;

%% Helper Functions for Mock Standalone Execution
function processedTargets = buildMockClusterTargets(curIdx, neighborIdx, neighbors, targets, simTime, actualEdgeLength)
    processedTargets = {};
    
    % Mock Visited Target
    tVis.index = neighborIdx;
    tVis.travelTime = actualEdgeLength;
    tVis.objectiveVisited_numeric = @(H) 0.5 * (H + actualEdgeLength)^2; 
    processedTargets{end+1} = tVis;
    
    % Mock Avoided Targets
    for j = 1:length(neighbors)
        otherIdx = neighbors(j);
        if otherIdx ~= neighborIdx
            tAvoid.index = otherIdx;
            tAvoid.J_i = 1.2 * (simTime + 1.0); 
            tAvoid.objectiveAvoided_numeric = @(H) 0;
            processedTargets{end+1} = tAvoid;
        end
    end
end

function [H_opt, J_min] = optimizeMockClusterDwellTime(policy, cluster)
    visited = []; avoided = {};
    for i = 1:length(cluster)
        t = cluster{i};
        if isfield(t, 'travelTime') && ~isempty(t.travelTime)
            visited = t;
        else
            avoided{end+1} = t; 
        end
    end
    
    function costVal = cost(H)
        costVal = 0;
        if ~isempty(visited)
            costVal = costVal + visited.objectiveVisited_numeric(H);
        end
        for k = 1:length(avoided)
            costVal = costVal + avoided{k}.J_i; 
        end
    end
    
    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');
    [H_opt, J_min] = fmincon(@cost, policy.H0, [], [], [], [], policy.H_lower, policy.H_upper, [], options);
end