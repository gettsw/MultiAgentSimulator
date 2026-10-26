function extensionTest()
% Headless test of the extension points a new user builds on. Run from the repo root:
%   matlab -batch "addpath(genpath(pwd)); extensionTest"
%
% 1. Target types: setTargetType rebuilds the scenario with targets of the new class and keeps
%    positions, edges, walls and agents; layouts store the type.
% 2. Metrics: the default objective is J(t) (AverageUncertainty), equal to the integral the
%    model keeps; setMetrics changes the objective; runTrial returns metricValues.
% 3. The event-detection example (Examples/EventDetection): SimpleTarget, SimpleAgent,
%    SimplePolicy and SimpleMetric run on the Museum layout; events are detected and
%    missed; SimplePolicy misses fewer events than Random Walk.
% 4. Grid A*: paths start and end at the targets, never cross a wall, and are no longer than
%    the Dijkstra Corners paths by more than 5% on any map.
% 5. Every planner's paths have no sharp corners; an unreachable target gives a straight edge
%    with a warning; stations (addStation) save and reload; Physical Agents on the lab never
%    cross a wall; an agent driven into a wall stops at it and the collision is counted.
% 6. Paper profiles (Configurations/paperProfiles) list only existing components, speed
%    profiles and scenarios; the older names "constant", "FO" and 'ED-ORHC-FO' still work and
%    give the same run as FO-0, FO-3 and 'ED-ORHC-FO-3'.
% Prints one line per check and errors at the end (non-zero exit under -batch) if any failed.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
rmpath(genpath(fullfile(root, 'Old_Design')));
failures = strings(0); nChecks = 0;
warning('off', 'runTrial:AgentPolicyFit');
museum = load(fullfile(root, 'Configurations', 'Museum.mat'), 's');
museum = museum.s;

% ---- 1. Target types ----
m = ScenarioModel();
m.importLayout(museum);
check(all(arrayfun(@(t) isa(t, 'SimpleTarget'), m.targets)), 'Museum layout loads event targets (stored targetType)');
nE = numel(m.edges); nW = size(m.walls, 1); nA = numel(m.agents); P = reshape([m.targets.position], 2, []).';
m.setTargetType("Uncertainty Target");
check(all(arrayfun(@(t) strcmp(class(t), 'Target'), m.targets)), 'setTargetType switches every target to the new class');
check(numel(m.edges) == nE && size(m.walls, 1) == nW && numel(m.agents) == nA ...
    && isequal(reshape([m.targets.position], 2, []).', P), 'setTargetType keeps positions, edges, walls and agents');
s = m.exportLayout();
check(s.targetType == "Uncertainty Target", 'exportLayout stores the target type');

% ---- 2. Metrics ----
out = runTrial(struct('Layout', "ReportLayout.mat", 'Policy', "Random Walk", 'AgentStarts', "layout", ...
    'UncertaintyMode', "Static Uniform", 'EndTime', 20, 'Seed', 1));
check(abs(out.metricValues.AverageUncertainty - out.finalJ) < 1e-12, 'default objective is J(t) (AverageUncertainty = finalJ)');
m = ScenarioModel();
m.setMetrics(["Simple Metric (example)", "Average Uncertainty J(t)"]);
check(isa(m.metrics{1}, 'SimpleMetric') && isa(m.metrics{2}, 'AverageUncertainty'), 'setMetrics sets the objective first');

% ---- 3. Event-detection example ----
missed = zeros(2, 3);
policies = ["Simple Policy (example)", "Random Walk"];
for i = 1:2
    for seed = 1:3
        out = runTrial(struct('Layout', "Museum.mat", 'Policy', policies(i), 'AgentStarts', "layout", ...
            'AgentType', "Simple Agent (example)", 'AgentSpeed', 4, 'EndTime', 300, 'Seed', seed, ...
            'Metrics', ["Simple Metric (example)", "Average Uncertainty J(t)"]));
        missed(i, seed) = out.metricValues.SimpleMetric;
        if i == 1 && seed == 1
            check(isfinite(out.finalJ) && out.finalJ == out.metricValues.SimpleMetric, ...
                'runTrial: finalJ is the first metric (share of events missed)');
        end
    end
end
check(all(missed(:) >= 0 & missed(:) <= 1), 'SimpleMetric is a share in [0, 1]');
check(mean(missed(1, :)) < mean(missed(2, :)), sprintf('SimplePolicy misses fewer events than Random Walk (%.2f vs %.2f)', ...
    mean(missed(1, :)), mean(missed(2, :))));
m = ScenarioModel(); m.importLayout(museum); m.setActivePolicy('Simple Policy (example)');
for a = m.agents, a.state.maxSpeed = 4; end
m.resetSimulationState(); rng(2);
clock = SimulationClock(200, 0.05);
while ~clock.isFinished()
    dt = clock.tick(); m.step(dt, clock.currentTime); m.updateTargetsAndLogObjective(clock.currentTime, dt);
end
check(sum([m.targets.detected]) > 0 && sum([m.targets.missed]) >= 0 && all([m.targets.R] >= 0), ...
    sprintf('event targets detect events (%d detected, %d missed in 200 s)', sum([m.targets.detected]), sum([m.targets.missed])));
moved = arrayfun(@(a) norm(a.state.pos - a.initialPosition) > 0 || a.current_target_idx ~= a.initialTargetIdx, m.agents);
check(any(moved), 'SimplePolicy moves the agents');

% ---- 4. Grid A* ----
cat = scenarioCatalog();
cat = cat(~startsWith(cat.Name, "JS:"), :);
for k = 1:height(cat)
    tmp = load(cat.File(k), 's');
    a = ScenarioModel(GridAStarPlanner()); a.importLayout(tmp.s);
    d = ScenarioModel(DijkstraCornerPlanner()); d.importLayout(tmp.s);
    ends = true; cross = 0; longer = 0;
    for e = 1:numel(a.edges)
        Pa = a.edges(e).curvePoints; Pd = d.edges(e).curvePoints;
        ends = ends && norm(Pa(1, :) - a.edges(e).targets(1).position) < 1e-9 ...
            && norm(Pa(end, :) - a.edges(e).targets(2).position) < 1e-9;
        for i = 1:size(Pa, 1) - 1
            for w = 1:size(a.walls, 1)
                cross = cross + crosses(Pa(i, :), Pa(i + 1, :), a.walls(w, 1:2), a.walls(w, 3:4));
            end
        end
        longer = max(longer, sum(vecnorm(diff(Pa), 2, 2)) / sum(vecnorm(diff(Pd), 2, 2)) - 1);
    end
    check(ends && cross == 0 && longer <= 0.05, sprintf('Grid A* on "%s": ends at the targets, %d wall crossings, at most %.1f%% longer than Dijkstra Corners', ...
        cat.Name(k), cross, 100 * longer));
end

% ---- 5. Smooth planner paths, unreachable targets, stations, collisions ----
for pl = {DijkstraCornerPlanner(), EnergyEfficientPlanner(), GridAStarPlanner()}
    m = ScenarioModel(pl{1});
    m.importLayout(load(fullfile(root, 'Configurations', 'ReportLayout.mat')).s);
    turn = 0; rmin = Inf;
    for e = m.edges
        D = diff(e.curvePoints); th = atan2(D(:, 2), D(:, 1)); len = vecnorm(D, 2, 2);
        dth = abs(angle(exp(1i * diff(th))));
        turn = max([turn; dth]);
        rmin = min([rmin; (len(1:end-1) + len(2:end)) / 2 ./ max(dth, eps)]);
    end
    check(turn <= 0.21 && rmin >= 0.5, sprintf('%s: no sharp corners left (largest heading change between path points %.2f rad, tightest turn radius %.2f)', ...
        class(pl{1}), turn, rmin));
end
m = ScenarioModel();
m.addTarget([20 50]); m.addTarget([80 50]);
m.walls = [50 0 50 100];                                   % a wall across the whole canvas
warning('off', 'PathPlanner:NoRoute');
[~, ok, msg] = m.addEdgeByTargets(1, 2);
warning('on', 'PathPlanner:NoRoute');
check(ok && contains(msg, "No wall-free route") && m.pathPlanner.noRoute, 'unreachable target: straight edge with a warning');
m = ScenarioModel();
m.addTarget([20 50]); st = m.addStation([80 50], 250); m.addEdgeByTargets(1, 2);
s2 = m.exportLayout(); m2 = ScenarioModel(); m2.importLayout(s2);
check(isa(m2.targets(2), 'ChargingStation') && m2.targets(2).chargePower == 250, 'addStation: station saved and reloaded with its power');
out = runTrial(struct('Layout', "ReportLayout.mat", 'Policy', "ED-RHC", 'AgentType', "Physical Agent", ...
    'AgentStarts', "layout", 'UncertaintyMode', "Static Uniform", 'EndTime', 60, 'Seed', 1));
check(out.collisions == 0, 'Physical Agents on the lab: no wall crossings in 60 s');
m = ScenarioModel(); m.addTarget([20 50]); m.addTarget([80 50]); m.addEdgeByTargets(1, 2);
m.addAgentOnTarget([20 50], 15, 1e-6); m.walls = [50 40 50 60];   % a wall added after the edge
m.agents(1).path = m.edges(1).curvePoints; m.resetSimulationState();
m.agents(1).path = m.edges(1).curvePoints; m.agents(1).current_target_idx = 2;
for k = 1:100, m.step(0.05, k * 0.05); end
check(m.collisions(1) > 0 && m.agents(1).state.pos(1) < 50, 'an agent driven into a wall stops at it and the collision is counted');

% ---- 6. Paper profiles and renamed names ----
reg = ComponentRegistry();
cat = scenarioCatalog();
P = paperProfiles();
for p = P'
    bad = strings(1, 0);
    for kind = ["policy", "agenttype", "pathplanner", "targettype", "metric"]
        bad = [bad, p.(kind)(~ismember(p.(kind), string(reg.keysFor(kind))))]; %#ok<AGROW>
    end
    bad = [bad, p.speedprofile(~ismember(p.speedprofile, SpeedProfile.names))]; %#ok<AGROW>
    for q = p.scenario
        if ~any(MainUI.listed(cat.Name, q)), bad(end+1) = q; end %#ok<AGROW>
    end
    check(isempty(bad), sprintf('paper profile "%s" lists only existing names %s', p.Name, strjoin(bad, ", ")));
end
check(P(1).Name == "Full" && all(structfun(@isempty, rmfield(P(1), {'Name', 'Description'}))), ...
    'paper profile "Full" hides nothing');
a1 = AbstractAgent(1, [0 0], 10, Profile="constant");
a2 = PhysicalAgent(1, [0 0], 10, Profile="FO");
check(a1.profile == "FO-0" && a2.profile == "FO-3" && SpeedProfile.energy("FO", 10, 5) == SpeedProfile.energy("FO-3", 10, 5), ...
    'older speed profile names: "constant" is FO-0, "FO" is FO-3');
pol = feval(reg.factoryFor('policy', 'ED-ORHC-FO'));
pol.transit = "FO";
t1 = pol.transit;
pol.transit = "constant speed";
check(isa(pol, 'EDRHCPolicy') && t1 == "FO-3" && pol.transit == "FO-0", ...
    'older policy name ED-ORHC-FO is ED-ORHC-FO-3; transits "FO" and "constant speed" are FO-3 and FO-0');
q = struct('Layout', fullfile("Legacy", "JS_1Agent_3T.mat"), 'AgentStarts', "layout", ...
    'AgentType', "Abstract Agent", 'EndTime', 30, 'Seed', 1);
q.Policy = "ED-ORHC-FO"; out = runTrial(q); Jold = out.finalJ;
q.Policy = "ED-ORHC-FO-3"; out = runTrial(q); Jnew = out.finalJ;
check(Jold == Jnew, sprintf('runTrial with ED-ORHC-FO and ED-ORHC-FO-3: J(30 s) = %.4f and %.4f', Jold, Jnew));

if isempty(failures)
    fprintf('\nextensionTest PASSED: %d checks\n', nChecks);
else
    error('extensionTest:Failed', 'extensionTest FAILED %d of %d checks:\n  %s', ...
        numel(failures), nChecks, strjoin(failures, newline + "  "));
end

    function check(ok, label)
        nChecks = nChecks + 1;
        if ok
            fprintf('PASS  %s\n', label);
        else
            fprintf('FAIL  %s\n', label);
            failures(end+1) = string(label);
        end
    end
end

function h = crosses(a, b, c, d)
% Do segments ab and cd cross (end points excluded)?
den = (d(2)-c(2))*(b(1)-a(1)) - (d(1)-c(1))*(b(2)-a(2));
if abs(den) < 1e-12, h = false; return; end
ua = ((d(1)-c(1))*(a(2)-c(2)) - (d(2)-c(2))*(a(1)-c(1))) / den;
ub = ((b(1)-a(1))*(a(2)-c(2)) - (b(2)-a(2))*(a(1)-c(1))) / den;
h = ua > 1e-9 && ua < 1-1e-9 && ub > 1e-9 && ub < 1-1e-9;
end
