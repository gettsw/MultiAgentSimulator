function collabFigures(outDir)
% Illustrative figures of the collaboration paper (W. Getts and S. Welikala, in preparation),
% written as vector PDFs to outDir (default: results/collab/):
%   fig_class_avoided/visited/abandoned/traded.pdf  uncertainty dynamics of the four target
%       classes of Sec. III, computed exactly with the recursion of Lemma 1
%       (CollabTargetCost.lindley); A_i = 1 and B_i = 1.5 for every agent
%   fig_neighborhoods.pdf  agent neighborhoods and cost sets under ED-RHC and Collab-ED-RHC
%   fig_setups.pdf         the eight benchmark configurations (Configurations/Legacy) with the
%       agents' start targets and the targets with A_i = 12 of the group study
%       (Configurations/Collab/Collab_*_Group.mat)
%   fig_setups_team.pdf    the team-size layouts (Configurations/Collab/Collab_General_N2..N6.mat)
%   fig_maps.pdf           the simulator's maps of the map study (collabMaps), as the GUI draws them
% Layouts are drawn as in the GUI (y up). Each figure is also saved as a 300-dpi PNG.
arguments
    outDir (1,:) char = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'results', 'collab')
end
if ~isfolder(outDir), mkdir(outDir); end
A = 1; B = 1.5;
% target classes: [arrival, departure] of each agent (Inf: stays), start uncertainty, window
classes = {
    'avoided',   zeros(0, 2),                         3, 5, 'none'
    'visited',   [1 Inf; 2 Inf; 3 Inf],               3, 5, 'arrive'
    'abandoned', [-Inf 1; -Inf 2; -Inf 3],            5, 5, 'depart'
    'traded',    [-Inf 2; -Inf 2.5; 3.5 Inf; 4 Inf],  3, 5, 'both'};
for c = 1:size(classes, 1)
    classFigure(classes{c, :}, A, B, outDir);
end
neighborhoodFigure(outDir);
setupFigure(outDir);
teamFigure(outDir);
mapsFigure(outDir);
end

function saveFig(f, base)
exportgraphics(f, base + ".pdf", 'ContentType', 'vector');
exportgraphics(f, base + ".png", 'Resolution', 300);
close(f);
end

% ------------------------------------------------------------------------------------------
function classFigure(name, occ, R0, T, ~, A, B, outDir)
% R_i(t), J_i(t), the monitoring profiles u_ia(t) and the activity v_i(t) of one target
[tb, Rb, zs, rs] = trajectory(R0, A, B, T, occ);
t = linspace(0, T, 4001)';
R = interp1(tb, Rb, t);
J = cumtrapz(t, R);
v = activity(t, R, occ, A, B);
cols = lines(max(size(occ, 1), 1));
ev = unique([occ(isfinite(occ))', 0]); ev = ev(ev > 0 & ev < T);
if strcmp(name, 'avoided')
    f = figure('Visible', 'off', 'Units', 'inches', 'Position', [0 0 3.5 1.45]);
    tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
else
    f = figure('Visible', 'off', 'Units', 'inches', 'Position', [0 0 3.5 2.75]);
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
end
% (a) uncertainty
nexttile; hold on; grid on; box on;
plot(t, R, 'k', 'LineWidth', 1.2);
ylim([0, 1.3 * max(R)]);
markEvents(occ, tb, Rb, cols, T);
markTimes(zs, rs, t, R, 'R');
xlabel('Time: $t$', 'Interpreter', 'latex'); ylabel('$R_i(t)$', 'Interpreter', 'latex');
title('(a) Uncertainty state', 'Interpreter', 'latex', 'FontWeight', 'normal');
style(gca, T);
% (b) local objective
nexttile; hold on; grid on; box on;
plot(t, J, 'k', 'LineWidth', 1.2);
ylim([0, 1.3 * max(J)]);
for e = ev, plot(e, interp1(t, J, e), 'o', 'MarkerSize', 4, 'MarkerFaceColor', [0.4 0.4 0.4], 'MarkerEdgeColor', 'none'); end
markTimes(zs, rs, t, J, 'J');
xlabel('Time: $t$', 'Interpreter', 'latex'); ylabel('$J_i(t)$', 'Interpreter', 'latex');
title('(b) Local objective', 'Interpreter', 'latex', 'FontWeight', 'normal');
style(gca, T);
if ~strcmp(name, 'avoided')
    % (c) monitoring profiles: one bar per agent while u_ia(t) = 1
    nexttile; hold on; grid on; box on;
    n = size(occ, 1);
    for a = 1:n
        s = max(occ(a, 1), 0); e = min(occ(a, 2), T);
        patch([s e e s], a + [-0.3 -0.3 0.3 0.3], cols(a, :), 'EdgeColor', 'none', 'FaceAlpha', 0.85);
        if isfinite(occ(a, 1)) && occ(a, 1) > 0
            text(s, a + 0.42, sprintf('$\\bar d_{%d}$', a), 'Interpreter', 'latex', 'FontSize', 7, 'HorizontalAlignment', 'center');
        end
        if isfinite(occ(a, 2))
            text(e, a + 0.42, sprintf('$d_{%d}$', a), 'Interpreter', 'latex', 'FontSize', 7, 'HorizontalAlignment', 'center');
        end
    end
    ylim([0.4, n + 0.75]); yticks(1:n); yticklabels(compose('$a_%d$', 1:n));
    xlabel('Time: $t$', 'Interpreter', 'latex'); ylabel('Agent', 'Interpreter', 'latex');
    title('(c) Monitoring profiles $u_{ia}(t)$', 'Interpreter', 'latex', 'FontWeight', 'normal');
    style(gca, T);
    % (d) activity
    nexttile; hold on; grid on; box on;
    stairs(t, v, 'k', 'LineWidth', 1.2);
    markTimes(zs, rs, t, v, 'v');
    ylim([-0.15 1.3]); yticks([0 1]);
    xlabel('Time: $t$', 'Interpreter', 'latex'); ylabel('$v_i(t)$', 'Interpreter', 'latex');
    title('(d) Active sensing profile', 'Interpreter', 'latex', 'FontWeight', 'normal');
    style(gca, T);
end
saveFig(f, fullfile(outDir, "fig_class_" + name));
end

function [tb, Rb, zs, rs] = trajectory(R0, A, B, T, occ)
% Breakpoints of R_i(t) (exact: piecewise linear) by the recursion of Lemma 1, with the
% zero-hitting times zs and the restart times rs
ev = unique([0, T, occ(isfinite(occ) & occ > 0 & occ < T)']);
tb = 0; Rb = R0; zs = []; rs = [];
R = R0; held = false;
for k = 1:numel(ev) - 1
    s = ev(k); D = ev(k + 1) - s;
    n = sum(occ(:, 1) <= s + 1e-12 & occ(:, 2) > s + 1e-12);
    sigma = A - B * n;
    if held && sigma > 0, rs(end+1) = s; held = false; end %#ok<AGROW>
    if held, tb(end+1) = ev(k + 1); Rb(end+1) = 0; continue; end %#ok<AGROW>
    Rn = R + sigma * D;
    if Rn < 0
        z = s + R / abs(sigma);
        tb(end+1:end+2) = [z, ev(k + 1)]; Rb(end+1:end+2) = [0, 0]; zs(end+1) = z; held = true; R = 0; %#ok<AGROW>
    else
        tb(end+1) = ev(k + 1); Rb(end+1) = Rn; R = Rn; %#ok<AGROW>
        if Rn == 0 && sigma < 0, zs(end+1) = ev(k + 1); held = true; end %#ok<AGROW>
    end
end
[tb, iu] = unique(tb, 'last'); Rb = Rb(iu);
end

function v = activity(t, R, occ, A, B)
n = arrayfun(@(s) sum(occ(:, 1) <= s & occ(:, 2) > s), t);
v = double(R > 1e-9 | A > B * n);
end

function markEvents(occ, tb, Rb, cols, T)
% colored points at each agent's arrival (filled) and departure (open)
for a = 1:size(occ, 1)
    for k = 1:2
        e = occ(a, k);
        if isfinite(e) && e > 0 && e < T
            y = interp1(tb, Rb, e);
            if k == 1
                plot(e, y, 'o', 'MarkerSize', 5, 'MarkerFaceColor', cols(a, :), 'MarkerEdgeColor', cols(a, :));
            else
                plot(e, y, 's', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', cols(a, :), 'LineWidth', 1.1);
            end
        end
    end
end
end

function markTimes(zs, rs, t, y, panel)
% the zero-hitting times (z_i, z_i') and the restart time (bar z_i)
names = {'$z_i$', '$z_i''$'};
for k = 1:numel(zs)
    yy = interp1(t, y, zs(k));
    plot(zs(k), yy, 'v', 'MarkerSize', 5, 'MarkerFaceColor', [0.85 0.1 0.1], 'MarkerEdgeColor', 'none');
    if panel ~= "v"
        text(zs(k), yy + 0.06 * range(ylim), names{min(k, 2)}, 'Interpreter', 'latex', 'FontSize', 8, ...
            'Color', [0.85 0.1 0.1], 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    end
end
for k = 1:numel(rs)
    yy = interp1(t, y, rs(k));
    plot(rs(k), yy, '^', 'MarkerSize', 5, 'MarkerFaceColor', [0.1 0.3 0.85], 'MarkerEdgeColor', 'none');
    if panel ~= "v"
        text(rs(k), yy + 0.06 * range(ylim), '$\bar z_i$', 'Interpreter', 'latex', 'FontSize', 8, ...
            'Color', [0.1 0.3 0.85], 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    end
end
end

function style(ax, T)
set(ax, 'FontSize', 7.5, 'TickLabelInterpreter', 'latex', 'XLim', [0 T]);
ax.Title.FontSize = 8;
end

% ------------------------------------------------------------------------------------------
function neighborhoodFigure(outDir)
% Agent neighborhoods and cost sets at an event of agent a at target i: (a) ED-RHC, where the
% covered neighbors k_1 (agent c en route) and k_2 (agent b resides) are removed; (b)
% Collab-ED-RHC (mode B), with the co-located group S_i = {a, a'}, the member b at k_2 (its
% announced next target dashed), the agent c in transit, and the cost set K_a
P = [0 0; 1.25 0.9; 1.25 -0.9; -1.3 0.35; -1.0 -0.95; 2.55 1.25; 2.6 -0.45; 2.35 -1.55];
E = [1 2; 1 3; 1 4; 1 5; 2 6; 2 3; 3 7; 3 8; 4 5];
green = [0.78 0.90 0.78]; blue = [0.84 0.89 0.99];
f = figure('Visible', 'off', 'Units', 'inches', 'Position', [0 0 3.5 1.85]);
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
% (a) ED-RHC
ax = nexttile; hold on; axis equal off;
band([1 4 5], green);
drawGraph([2 3]);
cross(P(2, :)); cross(P(3, :));
agent(P(1, :) + [0 0.27], [0.85 0.1 0.1], '$a$');
agent((P(1, :) + P(2, :)) / 2 + [-0.05 0.2], [0.5 0.5 0.5], '$c$');
arrowTo((P(1, :) + P(2, :)) / 2, P(2, :));
agent(P(3, :) + [0 0.27], [0.2 0.4 0.8], '$b$');
text(-1.75, -1.6, '$\bar{\mathcal{I}}_i\setminus\{k_1,k_2\}$', 'Interpreter', 'latex', 'FontSize', 7, 'Color', [0.15 0.45 0.15]);
title('(a) ED-RHC', 'Interpreter', 'latex', 'FontWeight', 'normal', 'FontSize', 8);
lims(ax);
% (b) Collab-ED-RHC
ax = nexttile; hold on; axis equal off;
band([1 2 3 4 5 7 8], blue);
band([1 2 3 4 5], green);
drawGraph([]);
agent(P(1, :) + [-0.14 0.27], [0.85 0.1 0.1], '$a$');
agent(P(1, :) + [0.16 0.27], [0.9 0.5 0.1], '$a''$');
agent((P(1, :) + P(2, :)) / 2 + [-0.05 0.2], [0.5 0.5 0.5], '$c$');
arrowTo((P(1, :) + P(2, :)) / 2, P(2, :));
agent(P(3, :) + [0 0.27], [0.2 0.4 0.8], '$b$');
u = (P(7, :) - P(3, :)) / norm(P(7, :) - P(3, :));   % b's announced next target, node edge to edge
plot([P(3, 1) P(7, 1)] + 0.2 * u(1) * [1 -1], [P(3, 2) P(7, 2)] + 0.2 * u(2) * [1 -1], '--', 'Color', [0.2 0.4 0.8], 'LineWidth', 1.4);
text(-0.25, -0.38, '$S_i$', 'Interpreter', 'latex', 'FontSize', 7);
text(0.3, -1.3, '$b\in\mathcal{M}_a$', 'Interpreter', 'latex', 'FontSize', 7);
text(-1.75, -1.6, '$\bar{\mathcal{I}}_i$', 'Interpreter', 'latex', 'FontSize', 7, 'Color', [0.15 0.45 0.15]);
text(2.25, 0.35, '$\mathcal{K}_a$', 'Interpreter', 'latex', 'FontSize', 7, 'Color', [0.15 0.25 0.65]);
title('(b) Collab-ED-RHC', 'Interpreter', 'latex', 'FontWeight', 'normal', 'FontSize', 8);
lims(ax);
saveFig(f, fullfile(outDir, "fig_neighborhoods"));

    function band(nodes, col)
        % a rounded band along the edges among the given nodes
        for e = 1:size(E, 1)
            if all(ismember(E(e, :), nodes))
                plot(P(E(e, :), 1), P(E(e, :), 2), '-', 'Color', col, 'LineWidth', 16);
            end
        end
        plot(P(nodes, 1), P(nodes, 2), 'o', 'MarkerSize', 22, 'MarkerFaceColor', col, 'MarkerEdgeColor', 'none');
    end
    function drawGraph(outside)
        % the targets in 'outside' (crossed out) get their names next to them
        for e = 1:size(E, 1)
            plot(P(E(e, :), 1), P(E(e, :), 2), 'k-', 'LineWidth', 0.8);
        end
        plot(P(:, 1), P(:, 2), 'o', 'MarkerSize', 9, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k');
        names = {'$i$', '$k_1$', '$k_2$', '$k_3$', '$k_4$', '', '', ''};
        shift = zeros(size(P)); shift(2, :) = [-0.36 0.24]; shift(3, :) = [-0.36 -0.26];
        for q = 1:size(P, 1)
            if ~isempty(names{q})
                d = shift(q, :) * ismember(q, outside);
                text(P(q, 1) + d(1), P(q, 2) + d(2), names{q}, 'Interpreter', 'latex', 'FontSize', 6, ...
                    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
            end
        end
    end
    function cross(p)
        plot(p(1) + [-0.2 0.2], p(2) + [-0.2 0.2], '-', 'Color', [0.85 0.1 0.1], 'LineWidth', 1.3);
        plot(p(1) + [-0.2 0.2], p(2) + [0.2 -0.2], '-', 'Color', [0.85 0.1 0.1], 'LineWidth', 1.3);
    end
    function agent(p, col, name)
        patch(p(1) + 0.12 * [-1 1 0], p(2) + 0.12 * [-0.8 -0.8 1], col, 'EdgeColor', 'k', 'LineWidth', 0.5);
        text(p(1), p(2) + 0.25, name, 'Interpreter', 'latex', 'FontSize', 7, 'HorizontalAlignment', 'center');
    end
    function arrowTo(p, q)
        d = q - p;
        quiver(p(1), p(2), 0.62 * d(1), 0.62 * d(2), 0, 'Color', [0.3 0.3 0.3], 'LineWidth', 0.9, 'MaxHeadSize', 0.8);
    end
    function lims(ax)
        xlim(ax, [-1.85 3.0]); ylim(ax, [-1.8 1.7]);
    end
end

% ------------------------------------------------------------------------------------------
function setupFigure(outDir)
% The eight benchmark configurations: targets, trajectory segments, start targets of the
% agents, and the targets with A_i = 12 in the group study (red)
presets = ["JS_1Agent_3T", "JS_1Agent_4T", "JS_1Agent_5T", "JS_2Agent_5T", ...
    "JS_3Agent_9T", "JS_3Agent_Maze", "JS_3Agent_General", "JS_3Agent_Room"];
names = ["1A-3T", "1A-4T", "1A-5T", "2A-5T", "3A-9T", "3A-Maze", "3A-General", "3A-Room"];
root = fileparts(which('MainUI'));
group = containers.Map('KeyType', 'char', 'ValueType', 'any');   % targets with A > B (group study)
for g = ["JS_2Agent_5T", "Collab_2Agent_5T_Group"; "JS_3Agent_General", "Collab_3Agent_General_Group"]'
    tmp = load(fullfile(root, 'Configurations', 'Collab', g(2) + ".mat"), 's');
    group(char(g(1))) = find(tmp.s.A(:) > tmp.s.B(:))';
end
f = figure('Visible', 'off', 'Units', 'inches', 'Position', [0 0 7.16 3.7]);
tl = tiledlayout(2, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:numel(presets)
    s = load(fullfile(root, 'Configurations', 'Legacy', presets(k) + ".mat"), 's'); s = s.s;
    ax = nexttile; hold on; axis equal; box on;
    for e = 1:size(s.edges, 1)
        plot(s.targets(s.edges(e, :), 1), s.targets(s.edges(e, :), 2), 'k-', 'LineWidth', 0.8);
    end
    hot = [];
    if isKey(group, char(presets(k))), hot = group(char(presets(k))); end
    for q = 1:size(s.targets, 1)
        if ismember(q, hot), fc = [0.95 0.55 0.55]; else, fc = [0.75 0.85 1.0]; end
        plot(s.targets(q, 1), s.targets(q, 2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', fc, 'MarkerEdgeColor', [0.1 0.2 0.5]);
        text(s.targets(q, 1), s.targets(q, 2), sprintf('%d', q), 'FontSize', 6, 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', 'Interpreter', 'latex');
    end
    for a = s.agentStarts(:)'
        p = s.targets(a, :);
        startMarker(p, 3.2, 5.5);
    end
    set(ax, 'YDir', 'normal', 'XTick', [], 'YTick', [], 'XLim', [0 100], 'YLim', [0 100]);
    title(names(k), 'Interpreter', 'latex', 'FontWeight', 'normal', 'FontSize', 8);
end
saveFig(f, fullfile(outDir, "fig_setups"));
end

% ------------------------------------------------------------------------------------------
function teamFigure(outDir)
% The team-size layouts: the General configuration with N = 2..6 agents
root = fileparts(which('MainUI'));
f = figure('Visible', 'off', 'Units', 'inches', 'Position', [0 0 7.16 1.75]);
tiledlayout(1, 5, 'TileSpacing', 'compact', 'Padding', 'compact');
for n = 2:6
    tmp = load(fullfile(root, 'Configurations', 'Collab', "Collab_General_N" + n + ".mat"), 's'); s = tmp.s;
    ax = nexttile; hold on; axis equal; box on;
    for e = 1:size(s.edges, 1)
        plot(s.targets(s.edges(e, :), 1), s.targets(s.edges(e, :), 2), 'k-', 'LineWidth', 0.8);
    end
    for q = 1:size(s.targets, 1)
        plot(s.targets(q, 1), s.targets(q, 2), 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.75 0.85 1.0], 'MarkerEdgeColor', [0.1 0.2 0.5]);
        text(s.targets(q, 1), s.targets(q, 2), sprintf('%d', q), 'FontSize', 5.5, 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', 'Interpreter', 'latex');
    end
    for a = s.agentStarts(:)'
        p = s.targets(a, :);
        startMarker(p, 3.2, 5.5);
    end
    set(ax, 'YDir', 'normal', 'XTick', [], 'YTick', [], 'XLim', [0 100], 'YLim', [0 100]);
    title(sprintf('$N = %d$', n), 'Interpreter', 'latex', 'FontWeight', 'normal', 'FontSize', 8);
end
saveFig(f, fullfile(outDir, "fig_setups_team"));
end

% ------------------------------------------------------------------------------------------
function mapsFigure(outDir)
% The maps of the map study (collabMaps) as the GUI draws them: background image, walls, the
% trajectory segments along the planned paths (Dijkstra Corners), targets and the agents' start
% targets
root = fileparts(which('MainUI'));
M = collabMaps();
f = figure('Visible', 'off', 'Units', 'inches', 'Position', [0 0 7.16 4.9]);
tiledlayout(2, 6, 'TileSpacing', 'compact', 'Padding', 'compact');
tiles = [1 3 5 8 10];   % three maps in the first row, two centered in the second
for k = 1:height(M)
    tmp = load(fullfile(root, 'Configurations', M.Layout(k) + ".mat"), 's'); s = tmp.s;
    model = ScenarioModel(DijkstraCornerPlanner());
    model.importLayout(s);
    ax = nexttile(tiles(k), [1 2]); hold on; axis equal; box on;
    img = flipud(imread(fullfile(root, 'Configurations', 'backgrounds', s.background)));
    image(ax, [0 100], [0 100], img, 'AlphaData', 0.85);
    for w = 1:size(s.walls, 1)
        plot(s.walls(w, [1 3]), s.walls(w, [2 4]), '-', 'Color', [0.2 0.2 0.2], 'LineWidth', 1);
    end
    for e = 1:numel(model.edges)
        c = model.edges(e).curvePoints;
        plot(c(:, 1), c(:, 2), '-', 'Color', [0.1 0.1 0.1], 'LineWidth', 0.6);
    end
    for q = 1:size(s.targets, 1)
        plot(s.targets(q, 1), s.targets(q, 2), 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0.75 0.85 1.0], 'MarkerEdgeColor', [0.1 0.2 0.5]);
        text(s.targets(q, 1), s.targets(q, 2), sprintf('%d', q), 'FontSize', 4.5, 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', 'Interpreter', 'latex');
    end
    for a = s.agentStarts(:)'
        startMarker(s.targets(a, :), 2.4, 4.4);
    end
    set(ax, 'YDir', 'normal', 'XTick', [], 'YTick', [], 'XLim', [0 100], 'YLim', [0 100]);
    title(sprintf('%s ($M = %d$, $N = %d$)', M.Name(k), size(s.targets, 1), numel(s.agentStarts)), ...
        'Interpreter', 'latex', 'FontWeight', 'normal', 'FontSize', 8);
end
saveFig(f, fullfile(outDir, "fig_maps"));
end

function startMarker(p, r, gap)
% an agent's start: a red triangle above its target (y up)
patch(p(1) + r * [-1 1 0], p(2) + gap + r * [-0.8 -0.8 1], [0.85 0.1 0.1], 'EdgeColor', 'k', 'LineWidth', 0.4);
end
