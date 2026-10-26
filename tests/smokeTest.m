function smokeTest()
% Headless smoke test of the simulator. Run from the repo root:
%   matlab -batch "addpath(genpath(pwd)); smokeTest"
%
% 1. Paper defaults: 'Static Uniform' mode, A = 1, B = 10, R(0) = 0.
% 2. Layout files: exportLayout/importLayout round-trip the scenario fields, and old files
%    (targets, edges, walls only) still load.
% 3. Every registered policy x path planner x agent type runs 20 s on the paper's reference
%    scenario (Configurations/ReportLayout.mat, 5 agents) without error, with a finite J,
%    finite agent positions, R >= 0, and at least one agent leaving its start target.
% 4. Edge cases: a team mixing all agent types and an agent on a target with no edges, under
%    every policy; Random Walk dwells (clearing dwell), and so does a policy that asks for its dwell
%    in the move command (cmd.dwellSeconds; tests/FixedDwellPolicy);
%    runTrial with the paper preset (Experiments/paper/paperDefaults); ED-ORHC-SO / -FO with
%    speed and acceleration limits keep every agent within them.
% 5. Agent types: the older type names still resolve (aliases); every type x speed profile runs;
%    only agents at rest service targets; a physical agent arrives within a step or two of its
%    planned time on a wall-routed edge, and keeps the transit times ED-RHC plans; the energy
%    model ED-ORHC plans with matches the energy a physical agent draws on a curved edge; the
%    heading turns smoothly on curved paths; William's Joint RHC harness (runTrial defaults with
%    'Linear Agent') runs.
% 6. Scenarios (scenarioCatalog): each loads (with its background image, if any) and runs 10 s
%    under every policy with Abstract Agents, and under Random Walk with Physical Agents.
% 7. Battery and charging: an agent's predicted transit energy matches the energy it draws;
%    on the office floor with a charging dock (small batteries that stop the robots when empty)
%    Battery-Aware ED-RHC recharges every agent in time, while ED-RHC runs out.
% Prints one line per check and errors at the end (non-zero exit under -batch) if any failed.

root = fileparts(fileparts(mfilename('fullpath')));
tmp = load(fullfile(root, 'Configurations', 'ReportLayout.mat'), 's');
layout = tmp.s;
nChecks = 0;
failures = strings(0, 1);

% ---- 1. Paper defaults ----
model = ScenarioModel();
tgt = Target(2, [0 0]);
check(strcmp(model.uncertaintyMode, 'Static Uniform') && tgt.A_uniformBase == 1 ...
    && tgt.B == 10 && tgt.R == 0, 'defaults: Static Uniform, A = 1, B = 10, R(0) = 0');
check(tgt.A_spike - tgt.B > 0 && tgt.A_spike - 2*tgt.B < 0, ...
    'spike mode: one agent cannot bring a spiking target down, two can');
check(strcmp(layout.uncertaintyMode, 'Static Uniform') && all(layout.A == 1) ...
    && all(layout.B == 10) && all(layout.R0 == 0), 'ReportLayout.mat: Static Uniform, A = 1, B = 10, R(0) = 0');

% ---- 2. Layout files ----
model.importLayout(layout);
check(isequal([model.agents.initialTargetIdx], layout.agentStarts) && numel(model.agents) == 5, ...
    'ReportLayout.mat places 5 agents on its stored start targets');
s = model.exportLayout();
m2 = ScenarioModel();
m2.importLayout(s);
check(isequal(m2.exportLayout(), s), 'export -> import -> export is unchanged');

custom = s;
custom.A(3) = 2.5; custom.B = 7; custom.R0(2) = 4;     % scalar B applies to every target
custom.uncertaintyMode = 'Dynamic Uncertainty'; custom.agentSpeed = 9;
m2.importLayout(custom);
m2.resetSimulationState();
c = m2.exportLayout();
check(c.A(3) == 2.5 && all(c.B == 7) && c.R0(2) == 4 && m2.targets(2).R == 4 ...
    && strcmp(c.uncertaintyMode, 'Dynamic Uncertainty') && all(c.agentSpeed == 9), ...
    'non-default A, B, R(0), mode and speed survive import, reset and export');

old = rmfield(layout, setdiff(fieldnames(layout), {'targets', 'edges', 'walls'}));
m3 = ScenarioModel();
m3.importLayout(old);
check(isempty(m3.agents) && numel(m3.targets) == size(old.targets, 1) ...
    && numel(m3.edges) == size(old.edges, 1) && all([m3.targets.B] == 10), ...
    'old layout file (targets, edges, walls only) loads with defaults');

% ---- 3. Every component combination on the reference scenario ----
reg = ComponentRegistry();
policies = reg.keysFor('policy');
planners = reg.keysFor('pathplanner');
types = reg.keysFor('agenttype');
for i = 1:numel(policies)
    for j = 1:numel(planners)
        for k = 1:numel(types)
            name = sprintf('%-18s %-17s %-14s', policies{i}, planners{j}, types{k});
            [ok, detail] = runScenario(@() reportScenario(planners{j}, types(k)), policies{i}, 20, true);
            check(ok, name + "  " + detail);
        end
    end
end

% ---- 4. Edge cases ----
for i = 1:numel(policies)
    [ok, detail] = runScenario(@() reportScenario('Dijkstra Corners', types), policies{i}, 20, true);
    check(ok, sprintf('%-18s mixed team (%s)  %s', policies{i}, strjoin(types, ', '), detail));
end
for i = 1:numel(policies)
    [ok, detail] = runScenario(@isolatedTarget, policies{i}, 2, false);
    check(ok, sprintf('%-18s agent on a target with no edges  %s', policies{i}, detail));
end
try
    p = paperDefaults();
    p.EndTime = 5;
    out = runTrial(p);
    m = out.metrics;
    check(numel(m.L) == 5 && isequal(out.log.starts, layout.agentStarts) && all(isfinite([m.L m.Theta m.Ea])) ...
        && size(out.log.pos, 1) == numel(out.log.t), ...
        'runTrial: paper preset, starts from the layout, logged metrics');
catch err
    check(false, "runTrial: paper preset  ERROR " + err.message);
end
[~, ~, dwelled] = runScenario(@() reportScenario('Dijkstra Corners', {'Physical Agent'}), "Random Walk", 20, true);
check(dwelled, "Random Walk agents dwell at targets");
try   % a dwell requested in the move command is spent at the target before leaving
    sim = reportScenario('Dijkstra Corners', {'Abstract Agent'});
    sim.policyMap('Fixed Dwell (test)') = FixedDwellPolicy();
    sim.setActivePolicy('Fixed Dwell (test)'); sim.resetSimulationState();
    a = sim.agents(1); start = a.state.pos; waited = 0; clock = SimulationClock(1.9, 0.05);
    while ~clock.isFinished()
        dt = clock.tick(); sim.step(dt, clock.currentTime); sim.updateTargetsAndLogObjective(clock.currentTime, dt);
        waited = waited + (a.dwellRemaining > 0);
    end
    check(norm(a.state.pos - start) < 1e-9 && waited > 30 && ~isempty(a.path), ...
        'cmd.dwellSeconds: the agent waits at its target, with the next path set');
catch err
    check(false, "cmd.dwellSeconds  ERROR " + err.message);
end
for name = ["ED-ORHC-SO", "ED-ORHC-FO-3"]
    try
        sim = reportScenario('Dijkstra Corners', {'Abstract Agent'});
        sim.setActivePolicy(char(name));
        pol = sim.policyMap(char(name));
        pol.speedLimit = 5; pol.accelLimit = 4;
        sim.resetSimulationState();
        clock = SimulationClock(20, 0.05);
        while ~clock.isFinished()
            dt = clock.tick();
            sim.step(dt, clock.currentTime);
            sim.updateTargetsAndLogObjective(clock.currentTime, dt);
        end
        vmax = max([sim.agents.peakSpeed]); umax = max([sim.agents.peakAccel]);
        check(vmax <= 5 + 1e-9 && umax <= 4 + 1e-9 && any([sim.agents.Je] > 0), ...
            sprintf('%s with limits 5 (speed) and 4 (acceleration): peaks %.3f, %.3f', name, vmax, umax));
    catch err
        check(false, name + " with limits  ERROR " + err.message);
    end
end

% ---- 5. Agent types ----
aliasClass = {'Linear Agent', 'AbstractAgent', "FO-0"; 'Second-Order Agent', 'AbstractAgent', "SO"; ...
    'Default Agent', 'PhysicalAgent', "FO-0"; 'Energy Agent', 'PhysicalAgent', "FO-0"};
for k = 1:size(aliasClass, 1)
    a = feval(reg.factoryFor('agenttype', aliasClass{k, 1}), 1, [0 0], 10);
    check(isa(a, aliasClass{k, 2}) && a.profile == aliasClass{k, 3}, ...
        sprintf('older agent type "%s" resolves to %s (%s)', aliasClass{k, :}));
end
for k = 1:numel(types)
    for prof = SpeedProfile.names
        [ok, detail] = runScenario(@() withProfile(reportScenario('Dijkstra Corners', types(k)), prof), ...
            'Random Walk', 20, true);
        check(ok, sprintf('%-23s profile %-8s  %s', types{k}, prof, detail));
    end
end
try   % service only at rest
    sim = reportScenario('Dijkstra Corners', {'Physical Agent'});
    sim.setActivePolicy('Random Walk'); sim.resetSimulationState();
    clock = SimulationClock(30, 0.05); bad = 0;
    while ~clock.isFinished()
        % residency is decided at the start of a step, from the agents' state before it
        inTransit = arrayfun(@(a) ~isempty(a.path) && a.dwellRemaining <= 0, sim.agents);
        dt = clock.tick(); sim.step(dt, clock.currentTime); sim.updateTargetsAndLogObjective(clock.currentTime, dt);
        for tg = sim.targets
            for r = tg.residingAgents(:)'
                if iscell(r), r = r{1}; end
                bad = bad + inTransit(r.index);
            end
        end
    end
    check(bad == 0, sprintf('only agents at rest service targets (%d violations)', bad));
catch err
    check(false, "service at rest  ERROR " + err.message);
end
try   % physical agents arrive when planned on a wall-routed edge
    sim = reportScenario('Dijkstra Corners', {'Physical Agent'});
    e = sim.findEdge(8, 2); P = e.curvePoints; y = sum(vecnorm(diff(P, 1, 1), 2, 2));
    a = PhysicalAgent(1, P(1, :), 15); a.path = P; t = 0;
    while ~isempty(a.path) && t < 60, a.update(0.05); t = t + 0.05; end
    check(t >= a.legTime && t <= a.legTime + 0.1 + 1e-9 && a.legTime == a.shortestTransit(P), ...
        sprintf('physical agent crosses edge 8-2 (%.1f m) in %.2f s (planned %.2f s, its shortest)', y, t, a.legTime));
catch err
    check(false, "physical agent arrival  ERROR " + err.message);
end
try   % ED-RHC plans with the physical agents' own transit times, and they keep them
    sim = reportScenario('Dijkstra Corners', {'Physical Agent'});
    sim.setActivePolicy('ED-RHC'); sim.resetSimulationState();
    step = 0.05; clock = SimulationClock(60, step); nA = numel(sim.agents);
    moving = false(1, nA); t0 = zeros(1, nA); handed = zeros(1, nA); late = [];
    while ~clock.isFinished()
        dt = clock.tick(); sim.step(dt, clock.currentTime); sim.updateTargetsAndLogObjective(clock.currentTime, dt);
        for k = 1:nA
            a = sim.agents(k); now = ~isempty(a.path) && a.dwellRemaining <= 0;
            if now && ~moving(k), t0(k) = clock.currentTime - dt; handed(k) = a.transitTime; end
            if ~now && moving(k) && isempty(a.path), late(end+1) = clock.currentTime - t0(k) - handed(k); end %#ok<AGROW>
            moving(k) = now;
        end
    end
    check(numel(late) > 10 && all(late > -1e-9 & late <= 2 * step + 1e-9), sprintf(['ED-RHC with ' ...
        'physical agents: %d transits, arrival - planned in [%.3f, %.3f] s'], numel(late), min(late), max(late)));
catch err
    check(false, "ED-RHC physical transit times  ERROR " + err.message);
end
try   % the energy ED-ORHC plans with is the energy the agent draws
    sim = reportScenario('Dijkstra Corners', {'Physical Agent'});
    P = sim.findEdge(8, 2).curvePoints; y = sum(vecnorm(diff(P, 1, 1), 2, 2));
    a = PhysicalAgent(1, P(1, :), 15);
    rho = 1.5 * a.shortestTransit(P);
    f = a.transitEnergyModel(P, y, "SO", Inf, Inf, [a.shortestTransit(P) 250]);
    a.transitProfile = "SO"; a.transitTime = rho; a.path = P; t = 0;
    while ~isempty(a.path) && t < 60, a.update(0.05); t = t + 0.05; end
    cf = SpeedProfile.energy("SO", y, rho);
    ab = AbstractAgent(1, P(1, :), 15);
    g = ab.transitEnergyModel(P, y, "SO", Inf, Inf, [1 250]);
    check(abs(f(rho) / a.Je - 1) < 0.02 && isequal(g([rho; 2 * rho]), SpeedProfile.energy("SO", y, [rho; 2 * rho])), ...
        sprintf(['ED-ORHC energy model on edge 8-2 (SO, %.1f s): Physical Agent predicted %.1f, drawn %.1f ' ...
        '(closed form %.1f); Abstract Agent = closed form'], rho, f(rho), a.Je, cf));
catch err
    check(false, "transit energy model  ERROR " + err.message);
end
try   % smooth heading on a curved path
    a = AbstractAgent(1, P(1, :), 15); a.path = P; ori = []; om = [];
    while ~isempty(a.path), a.update(0.05); ori(end+1) = a.state.ori; om(end+1) = a.state.omega; end %#ok<AGROW>
    jump = max(abs(angle(exp(1i * diff(ori)))));
    D = diff(P); th = atan2(D(:, 2), D(:, 1)); len = vecnorm(D, 2, 2);         % tightest turn of the path
    rmin = min((len(1:end-1) + len(2:end)) / 2 ./ max(abs(angle(exp(1i * diff(th)))), eps));
    bound = max(0.6, 1.2 * 15 * 0.05 / rmin);   % the turn the path itself asks for at speed 15
    check(jump < bound && all(isfinite(om)), sprintf(['abstract agent heading on a curved edge: largest step %.2f rad ' ...
        '(path''s tightest radius %.2f allows %.2f), max |omega| %.1f rad/s'], jump, rmin, bound, max(abs(om))));
catch err
    check(false, "curved heading  ERROR " + err.message);
end
try   % William's harness: Joint RHC, 2 'Linear Agent's, Dynamic Uncertainty
    out = runTrial(struct('EndTime', 20));
    check(isfinite(out.finalJ), sprintf('runTrial defaults (Joint RHC, Linear Agent alias): J(20 s) = %.4f', out.finalJ));
catch err
    check(false, "runTrial defaults  ERROR " + err.message);
end

% ---- 6. Scenarios ----
warnState = warning('off', 'runTrial:AgentPolicyFit');
cat = scenarioCatalog();
for k = 1:height(cat)
    tmp = load(cat.File(k), 's'); sc = tmp.s;
    if isfield(sc, 'background') && strlength(string(sc.background)) > 0
        check(ScenarioModel.backgroundFile(sc.background) ~= "", sprintf('scenario "%s": background %s found', cat.Name(k), sc.background));
    end
    mapScenario = ~startsWith(cat.Name(k), "JS:");
    if mapScenario, pols = policies; else, pols = {'ED-RHC'}; end
    combos = [pols(:), repmat({'Abstract Agent'}, numel(pols), 1)];
    if mapScenario, combos(end+1, :) = {'Random Walk', 'Physical Agent'}; end %#ok<AGROW>
    bad = strings(0, 1);
    for c = 1:size(combos, 1)
        [ok, detail] = runScenario(@() scenarioModel(sc, combos{c, 2}), combos{c, 1}, 10, false);
        if ~ok, bad(end+1) = combos{c, 1} + " / " + combos{c, 2} + ": " + detail; end %#ok<AGROW>
    end
    check(isempty(bad), sprintf('scenario "%s": %d policy/agent runs%s', cat.Name(k), size(combos, 1), ...
        join([""; bad], newline + "      ")));
end
warning(warnState);

% ---- 7. Battery and charging ----
try   % predicted transit energy = energy drawn, on a wall-routed edge
    sim = reportScenario('Dijkstra Corners', {'Abstract Agent'});
    P = sim.findEdge(8, 2).curvePoints;
    for typeName = ["Abstract Agent", "Physical Agent"]
        a = feval(reg.factoryFor('agenttype', char(typeName)), 1, P(1, :), 15);
        rho = max(sum(vecnorm(diff(P), 2, 2)) / 15, a.shortestTransit(P));
        Epred = a.transitEnergy(P, rho);
        a.transitTime = rho; a.path = P; t = 0;
        while ~isempty(a.path) && t < 60, a.update(0.05); t = t + 0.05; end
        dev = a.battery.energyUsed / Epred - 1;
        check(abs(dev) < 0.02, sprintf('%s: transit energy on edge 8-2 predicted %.0f J, drawn %.0f J (%+.1f%%)', ...
            typeName, Epred, a.battery.energyUsed, 100 * dev));
    end
catch err
    check(false, "transit energy prediction  ERROR " + err.message);
end
try   % the battery-aware policy recharges in time; ED-RHC runs out
    tmp = load(fullfile(root, 'Configurations', 'OfficeDock.mat'), 's'); dock = tmp.s;
    T = 600;
    for typeName = ["Abstract Agent", "Physical Agent"]
        [low, n, J] = batteryRun(dock, 'Battery-Aware ED-RHC', typeName, T);
        check(low > 0 && n >= 3 && isfinite(J), sprintf(['Battery-Aware ED-RHC, %s, office ' ...
            'with dock (%d s): lowest SOC %.1f%%, %d charges, J = %.1f'], typeName, T, low, n, J));
    end
    [low, ~, J] = batteryRun(dock, 'ED-RHC', "Abstract Agent", T);
    check(low <= 0, sprintf('ED-RHC there runs out of battery (lowest SOC %.1f%%, J = %.1f)', low, J));
catch err
    check(false, "battery-aware policy  ERROR " + err.message);
end

if isempty(failures)
    fprintf('\nsmokeTest PASSED: %d checks\n', nChecks);
else
    error('smokeTest:Failed', 'smokeTest FAILED %d of %d checks:\n  %s', ...
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

    function [ok, detail, dwelled] = runScenario(build, policyName, T, mustMove)
        % Runs the model from build() for T seconds under the policy; dwelled reports whether
        % any agent waited at a target
        rng(0);   % Random Walk dwell draws
        dwelled = false;
        try
            sim = build();
            sim.setActivePolicy(policyName);
            sim.resetSimulationState();
            clock = SimulationClock(T, 0.05);
            J = NaN;
            while ~clock.isFinished()
                dt = clock.tick();
                sim.step(dt, clock.currentTime);
                [~, J] = sim.updateTargetsAndLogObjective(clock.currentTime, dt);
                dwelled = dwelled || any([sim.agents.dwellRemaining] > 0);
            end
            pos = cell2mat(arrayfun(@(a) a.state.pos, sim.agents(:), 'UniformOutput', false));
            starts = cell2mat(arrayfun(@(a) a.initialPosition(:)', sim.agents(:), 'UniformOutput', false));
            moved = any(vecnorm(pos - starts, 2, 2) > 1);
            ok = isfinite(J) && all(isfinite(pos(:))) && all([sim.targets.R] >= 0) && (moved || ~mustMove);
            detail = sprintf('J(%g s) = %.2f', T, J);
            if mustMove && ~moved, detail = detail + "  (no agent left its start)"; end
        catch err
            ok = false;
            detail = sprintf('ERROR %s (%s, line %d)', err.message, err.stack(1).name, err.stack(1).line);
        end
    end

    function [low, charges, J] = batteryRun(sc, policyName, typeName, T)
        % Lowest SOC [%] of any agent, charging sessions (NaN if the policy does not count
        % them) and J(T) of a run of layout sc
        sim = scenarioModel(sc, typeName);
        sim.setMetrics(["Average Uncertainty J(t)", "Minimum SOC"]);
        sim.setActivePolicy(policyName); sim.resetSimulationState();
        clock = SimulationClock(T, 0.05);
        while ~clock.isFinished()
            dt = clock.tick(); sim.step(dt, clock.currentTime); sim.updateTargetsAndLogObjective(clock.currentTime, dt);
        end
        v = sim.metricValues(clock.currentTime);
        low = v.MinimumSOC; J = v.AverageUncertainty;
        charges = NaN; if isprop(sim.policy, 'charges'), charges = sim.policy.charges; end
    end

    function sim = scenarioModel(sc, typeName)
        % A scenario layout with agents of typeName
        sim = ScenarioModel();
        sim.setAgentType(typeName);
        sim.importLayout(sc);
    end

    function sim = reportScenario(plannerName, typeNames)
        % ReportLayout.mat; agent k gets type typeNames{k} (cycling through the list)
        plannerFactory = reg.factoryFor('pathplanner', plannerName);
        sim = ScenarioModel(plannerFactory());
        noAgents = layout;
        noAgents.agentStarts = [];
        sim.importLayout(noAgents);
        for a = 1:numel(layout.agentStarts)
            sim.setAgentFactory(reg.factoryFor('agenttype', typeNames{mod(a - 1, numel(typeNames)) + 1}));
            sim.addAgentOnTarget(sim.targets(layout.agentStarts(a)).position, layout.agentSpeed(a), 1e-6);
        end
    end
end

function sim = isolatedTarget()
% One agent on a target with no edges, plus a second, unconnected target
sim = ScenarioModel();
sim.importLayout(struct('targets', [10 10; 50 50], 'edges', zeros(0, 2), 'agentStarts', 1));
end

function sim = withProfile(sim, prof)
% Every agent of sim uses the speed profile prof
for a = sim.agents, a.profile = prof; end
end
