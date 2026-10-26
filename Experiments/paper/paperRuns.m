function R = paperRuns(base, conditions, opts)
% Runs every condition of an experiment (paperConditions) for each of its seeds and collects
% the paper's metrics.
%   base        runTrial parameters shared by all runs (paperDefaults)
%   conditions  struct array with fields Label, Params (overrides of base) and Seeds
%   opts.Quick  true: every run lasts 60 s and uses at most two seeds (pipeline check)
% R.Runs has one row per run:
%   J        J(T), the time-averaged total uncertainty
%   Je       energy objective, sum over the agents of int |u|^2 dt (NaN with constant-speed
%            transits, whose accelerations are impulses)
%   JT       energyWeight * Je + J for the energy-aware policies (NaN otherwise)
%   Energy   energy drawn from the agents' batteries, summed over the agents [kJ]
%   SOC      mean final state of charge [%];  MinSOC  lowest SOC of any agent at any time [%]
%   Empty    time the first battery ran empty [s] (NaN if none did)
%   Charges  charging sessions (Battery-Aware ED-RHC; 0 otherwise)
%   Collisions  steps in which an agent's motion crossed a wall
% R.Curves{c, s} holds that run's time t, J(t), mean SOC(t) and the SOC of every agent [%].
arguments
    base (1,1) struct
    conditions (1,:) struct
    opts.Quick (1,1) logical = false
end
nC = numel(conditions);
rows = {};
R.Curves = cell(nC, 1);
for c = 1:nC
    p = base;
    ov = conditions(c).Params;
    for f = string(fieldnames(ov))'
        p.(f) = ov.(f);
    end
    seeds = conditions(c).Seeds;
    if opts.Quick, p.EndTime = min(p.EndTime, 60); seeds = seeds(1:min(2, end)); end
    for s = 1:numel(seeds)
        p.Seed = seeds(s);
        p.Log = true;
        out = runTrial(p);
        soc = max(0, 100 * out.log.soc);   % an empty battery reads 0%
        empty = find(any(soc <= 0, 2), 1);
        if isempty(empty), tEmpty = NaN; else, tEmpty = out.log.t(empty); end
        charges = 0;
        if isprop(out.policy, 'charges'), charges = sum(out.policy.charges); end
        rows{end+1} = table(string(conditions(c).Label), seeds(s), out.finalJ, out.Je, out.JT, ...
            sum(out.metrics.Energy) / 1000, mean(soc(end, :)), min(soc(:)), tEmpty, charges, ...
            out.collisions, 'VariableNames', {'Condition', 'Seed', 'J', 'Je', 'JT', 'Energy', ...
            'SOC', 'MinSOC', 'Empty', 'Charges', 'Collisions'}); %#ok<AGROW>
        R.Curves{c, s} = struct('t', out.log.t, 'J', out.log.J, 'SOC', mean(soc, 2), 'SOCs', soc);
        fprintf('  %-34s seed %2d: J %8.1f  Je %8.0f  energy %6.1f kJ  min SOC %5.1f%%\n', ...
            conditions(c).Label, seeds(s), out.finalJ, out.Je, sum(out.metrics.Energy) / 1000, min(soc(:)));
    end
end
R.Runs = vertcat(rows{:});
R.Labels = string({conditions.Label});
R.Conditions = conditions;
end
