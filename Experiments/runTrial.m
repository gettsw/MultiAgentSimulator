function out = runTrial(params)
% One headless simulation run. Returns metrics plus the policy object (for its stats).
% Called by runSweep (grids of trials), the diagnostic scripts, and the paper experiments
% (Experiments/paper). Parameters: see trialDefaults. With params.Log = true the output also
% holds out.log (per-step time, J(t), agent states, SOC and energy) and out.metrics
% (trialMetrics: path length, heading change, acceleration cost, final SOC per agent).

p = withDefaults(params);
root = fileparts(fileparts(mfilename('fullpath')));
if exist('ScenarioModel', 'class') ~= 8
    addpath(genpath(root));  % fallback when the project path is not loaded
end

% ---- Build scenario ----
layoutFile = char(p.Layout);                 % file name in Configurations/, or any full path
if ~isfile(layoutFile), layoutFile = fullfile(root, 'Configurations', layoutFile); end
tmp = load(layoutFile, 's');
reg = ComponentRegistry();
plannerFactory = reg.factoryFor('pathplanner', char(p.Planner));
model = ScenarioModel(plannerFactory(), reg.factoryFor('agenttype', char(p.AgentType)));
model.importLayout(tmp.s);
stored = arrayfun(@(a) a.initialTargetIdx, model.agents);   % start targets saved in the layout
model.agents = [];   % trial parameters place the agents ("layout" reuses the stored starts)
for t = model.targets
    t.configure(p);
end

starts = parseStarts(p.AgentStarts, stored);
if ~isempty(starts), p.NumAgents = numel(starts); end
% ponytail: Joint RHC's pairwise planner is only designed for two agents; lift this when it scales
assert(p.Policy ~= "Joint RHC" || p.NumAgents == 2, ...
    'Joint RHC is only built for 2 agents (got NumAgents = %d)', p.NumAgents);

% Agents start on AgentStarts, or spread evenly over target indices ("auto")
nT = numel(model.targets);
if isempty(starts)
    starts = mod(round((0:p.NumAgents-1) * nT / p.NumAgents), nT) + 1;   % "auto": spread evenly
end
assert(all(starts >= 1 & starts <= nT & starts == round(starts)), ...
    'AgentStarts %s must be target indices 1..%d of %s', mat2str(starts), nT, char(p.Layout));
for k = 1:p.NumAgents
    model.addAgentOnTarget(model.targets(starts(k)).position, p.AgentSpeed, 1e-6);
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
if p.Seed >= 0, rng(p.Seed); end   % -1 (the default) keeps the caller's rng state

% ---- Simulate (same stepping as SimulationController.runToEnd) ----
clock = SimulationClock(p.EndTime, p.Dt);
n = ceil(p.EndTime / p.Dt) + 1;
tLog = zeros(n, 1); jLog = zeros(n, 1);
logOn = isfield(p, 'Log') && p.Log;
if logOn, L = emptyLog(n, numel(model.agents)); end
peakR = 0; coLocationTime = 0; k = 0;
while ~clock.isFinished()
    dt = clock.tick();
    model.step(dt, clock.currentTime);
    [~, J] = model.updateTargetsAndLogObjective(clock.currentTime, dt);
    k = k + 1; tLog(k) = clock.currentTime; jLog(k) = J;
    if logOn, L = logAgents(L, k, model.agents); end
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
if logOn
    for name = ["pos", "vel", "acc", "ori", "soc", "energy"]
        L.(name) = L.(name)(1:k, :, :);
    end
    L.t = tLog; L.J = jLog; L.dt = p.Dt; L.starts = starts;
    out.log = L;
    out.metrics = trialMetrics(L);
end
end

function L = emptyLog(n, nA)
% Per-step agent states: pos/vel/acc are n x nA x 2, the rest n x nA (NaN if the type has none)
L = struct('pos', zeros(n, nA, 2), 'vel', zeros(n, nA, 2), 'acc', zeros(n, nA, 2), ...
    'ori', zeros(n, nA), 'soc', nan(n, nA), 'energy', nan(n, nA));
end

function L = logAgents(L, k, agents)
for a = 1:numel(agents)
    s = agents(a).state;
    L.pos(k, a, :) = s.pos; L.vel(k, a, :) = s.vel; L.acc(k, a, :) = s.acc; L.ori(k, a) = s.ori;
    if isprop(agents(a), 'soc')
        L.soc(k, a) = agents(a).soc;
        L.energy(k, a) = agents(a).e_total;
    end
end
end

function starts = parseStarts(v, stored)
% AgentStarts arrives as "auto", "layout", a numeric vector, or its text form "[1 5]" (from runSweep)
if isnumeric(v), starts = v(:)'; return; end
v = string(v);
if v == "auto", starts = []; return; end
if v == "layout"
    assert(~isempty(stored), 'AgentStarts "layout": the layout file stores no agent starts');
    starts = stored;
    return;
end
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
