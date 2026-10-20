function R = paperRuns(base, conditions, seeds)
% Runs every condition for every seed and collects the paper's metrics.
%   base        runTrial parameters shared by all runs (paperDefaults)
%   conditions  struct array with fields Label (text) and Params (struct of overrides)
%   seeds       rng seeds, one run per seed and condition
% R.Runs has one row per run: J = J(T); L, Theta, Ea summed over the agents; SOC the mean
% final state of charge [%]; Energy the team's energy drawn from the batteries [J].
% R.Curves{c, s} holds that run's time t, J(t), and mean SOC(t) [%].
nC = numel(conditions);
rows = cell(nC, numel(seeds));
R.Curves = cell(nC, numel(seeds));
for c = 1:nC
    p = base;
    ov = conditions(c).Params;
    for f = string(fieldnames(ov))'
        p.(f) = ov.(f);
    end
    for s = 1:numel(seeds)
        p.Seed = seeds(s);
        p.Log = true;
        out = runTrial(p);
        m = out.metrics;
        rows{c, s} = table(string(conditions(c).Label), seeds(s), out.finalJ, sum(m.L), ...
            sum(m.Theta), sum(m.Ea), mean(m.SOC), sum(m.Energy), ...
            'VariableNames', {'Condition', 'Seed', 'J', 'L', 'Theta', 'Ea', 'SOC', 'Energy'});
        R.Curves{c, s} = struct('t', out.log.t, 'J', out.log.J, 'SOC', 100 * mean(out.log.soc, 2));
    end
end
rows = rows.';               % one condition after another, seeds in order
R.Runs = vertcat(rows{:});
R.Labels = string({conditions.Label});
R.Seeds = seeds;
end
