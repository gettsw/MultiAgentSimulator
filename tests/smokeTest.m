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
            [ok, detail] = runCombo(policies{i}, planners{j}, types{k});
            check(ok, name + "  " + detail);
        end
    end
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

    function [ok, detail] = runCombo(policyName, plannerName, typeName)
        rng(0);   % Random Walk / Battery Efficient dwell draws
        try
            plannerFactory = reg.factoryFor('pathplanner', plannerName);
            sim = ScenarioModel(plannerFactory(), reg.factoryFor('agenttype', typeName));
            sim.importLayout(layout);
            sim.setActivePolicy(policyName);
            sim.resetSimulationState();
            clock = SimulationClock(20, 0.05);
            J = NaN;
            while ~clock.isFinished()
                dt = clock.tick();
                sim.step(dt, clock.currentTime);
                [~, J] = sim.updateTargetsAndLogObjective(clock.currentTime, dt);
            end
            pos = cell2mat(arrayfun(@(a) a.state.pos, sim.agents(:), 'UniformOutput', false));
            starts = cell2mat(arrayfun(@(a) a.initialPosition(:)', sim.agents(:), 'UniformOutput', false));
            moved = any(vecnorm(pos - starts, 2, 2) > 1);
            ok = isfinite(J) && all(isfinite(pos(:))) && all([sim.targets.R] >= 0) && moved;
            detail = sprintf('J(20 s) = %.2f', J);
            if ~moved, detail = detail + "  (no agent left its start)"; end
        catch err
            ok = false;
            detail = sprintf('ERROR %s (%s, line %d)', err.message, err.stack(1).name, err.stack(1).line);
        end
    end
end
