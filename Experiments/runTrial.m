function out = runTrial(params)
% One headless simulation run. Returns metrics plus the policy object (for its stats).
% Called by runSweep (grids of trials) and the diagnostic scripts.

p = withDefaults(params);
starts = parseStarts(p.AgentStarts);
if ~isempty(starts), p.NumAgents = numel(starts); end
% ponytail: Joint RHC's pairwise planner is only designed for two agents; lift this when it scales
assert(p.Policy ~= "Joint RHC" || p.NumAgents == 2, ...
    'Joint RHC is only built for 2 agents (got NumAgents = %d)', p.NumAgents);
root = fileparts(fileparts(mfilename('fullpath')));
if exist('ScenarioModel', 'class') ~= 8
    addpath(genpath(root));  % fallback when the project path is not loaded
end

% ---- Build scenario ----
layoutFile = char(p.Layout);                 % file name in Configurations/, or any full path
if ~isfile(layoutFile), layoutFile = fullfile(root, 'Configurations', layoutFile); end
tmp = load(layoutFile, 's');
model = ScenarioModel();
model.importLayout(tmp.s);
for t = model.targets
    t.configure(p);
end

% Agents start on AgentStarts, or spread evenly over target indices ("auto")
nT = numel(model.targets);
if isempty(starts)
    starts = mod(round((0:p.NumAgents-1) * nT / p.NumAgents), nT) + 1;   % "auto": spread evenly
end
assert(all(starts >= 1 & starts <= nT & starts == round(starts)), ...
    'AgentStarts %s must be target indices 1..%d of %s', mat2str(starts), nT, char(p.Layout));
for k = 1:p.NumAgents
    model.addAgentOnTarget(model.targets(starts(k)).position, p.AgentSpeed, 1e-6, "Linear");
end

model.setActivePolicy(char(p.Policy));

% Parameters named Policy_<property> set that property on the active policy (ablations).
% A switch only some policies have (e.g. Joint-only) is skipped for the others, so one sweep
% can mix policies; a name no policy has is a typo and errors.
pol = model.policyMap(char(p.Policy));
allPols = model.policyMap.values;
f = fieldnames(p);
for i = 1:numel(f)
    if startsWith(f{i}, 'Policy_')
        name = extractAfter(f{i}, 'Policy_');
        assert(any(cellfun(@(q) isprop(q, name), allPols)), 'No policy has property "%s"', name);
        if isprop(pol, name)
            pol.(name) = p.(f{i});
        end
    end
end
model.setUncertaintyMode(char(p.UncertaintyMode));
model.resetSimulationState();  % also clears RHC reservation tables from earlier trials

% ---- Simulate (same stepping as SimulationController.runToEnd) ----
clock = SimulationClock(p.EndTime, p.Dt);
n = ceil(p.EndTime / p.Dt) + 1;
tLog = zeros(n, 1); jLog = zeros(n, 1);
peakR = 0; coLocationTime = 0; k = 0;
while ~clock.isFinished()
    dt = clock.tick();
    model.step(dt, clock.currentTime);
    [~, J] = model.updateTargetsAndLogObjective(clock.currentTime, dt);
    k = k + 1; tLog(k) = clock.currentTime; jLog(k) = J;
    peakR = max([peakR, model.targets.R]);
    if any(arrayfun(@(tg) numel(tg.residingAgents) > 1, model.targets))
        coLocationTime = coLocationTime + dt;
    end
end
tLog = tLog(1:k); jLog = jLog(1:k);

% ---- Metrics ----
finalJ = jLog(end);
tail = tLog >= 0.8 * p.EndTime;
steadyJ = mean(jLog(tail));
c = polyfit(tLog(tail), jLog(tail), 1);
slopeJ = c(1);

out = struct('finalJ', finalJ, 'steadyJ', steadyJ, 'slopeJ', slopeJ, ...
    'peakR', peakR, 'coLocationTime', coLocationTime, 'policy', pol);
end

function starts = parseStarts(v)
% AgentStarts arrives as "auto", a numeric vector, or its text form "[1 5]" (from runSweep)
if isnumeric(v), starts = v(:)'; return; end
v = string(v);
if v == "auto", starts = []; return; end
starts = sscanf(char(erase(v, ["[", "]", ","])), '%f')';
end

function p = withDefaults(params)
p = trialDefaults();
if nargin < 1 || isempty(params), return; end
f = fieldnames(params);
for i = 1:numel(f)
    p.(f{i}) = params.(f{i});
end
end
