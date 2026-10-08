% Self-check for analyzeResults on synthetic data with a known effect.
% Mimics an Experiment Manager sweep that mixes policies with a Joint-only switch:
% Classical rows are duplicated across Policy_divergeScoring (the switch does not apply to it).
rs = RandStream('mt19937ar', 'Seed', 1);
[mode, nAg, aSpike] = ndgrid(["Static Uniform", "Dynamic Uncertainty"], [2 3 4 5], [15 20 25 30 35]);
base = table(mode(:), nAg(:), aSpike(:), 'VariableNames', {'UncertaintyMode', 'NumAgents', 'A_spike'});
base.JClassical = 10 + 90 * rand(rs, height(base), 1);   % 20 settings per mode

T = table();
for sc = ["pair", "classical"]
    c = base; c.Policy = repmat("Classical RHC [1]", height(c), 1);
    c.Policy_divergeScoring = repmat(sc, height(c), 1);
    c.finalJ = c.JClassical;                              % identical: switch does not apply
    j = base; j.Policy = repmat("Joint RHC", height(j), 1);
    j.Policy_divergeScoring = repmat(sc, height(j), 1);
    factor = 0.5 * (sc == "pair") + 1.2 * (sc == "classical");
    j.finalJ = factor * j.JClassical .* (1 + 0.01 * randn(rs, height(j), 1));
    T = [T; c; j]; %#ok<AGROW>
end
T.slopeJ = zeros(height(T), 1);
T.JClassical = [];

% Pass as two tables (split by mode) to exercise merging
R = analyzeResults({T(T.UncertaintyMode == "Static Uniform", :), T(T.UncertaintyMode ~= "Static Uniform", :)}, ...
    'Plot', false);

P = R.Paired;
assert(height(P) == 4, 'Expected 2 modes x 2 Joint variants, got %d rows', height(P));
assert(~any(contains(R.Data.Variant(R.Data.Policy == "Classical RHC [1]"), "divergeScoring")), ...
    'Joint-only switch leaked into Classical variant label');
pair = P(P.Variant == "Joint RHC | divergeScoring=pair", :);
cls  = P(P.Variant == "Joint RHC | divergeScoring=classical", :);
assert(all(pair.NPairs == 20) && all(cls.NPairs == 20), 'Pairing lost or duplicated runs');
assert(all(abs(pair.MedianRatio - 0.5) < 0.02) && all(abs(cls.MedianRatio - 1.2) < 0.03), 'Wrong median ratio');
assert(all(pair.CI95Lo <= pair.MedianRatio & pair.MedianRatio <= pair.CI95Hi & ...
    pair.CI95Lo > 0.45 & pair.CI95Hi < 0.55), 'CI malformed or far from the true ratio');
assert(all(pair.WinRate == 1) && all(cls.WinRate == 0), 'Wrong win rate');
assert(all(P.PValue < 0.001), 'Clear effect not significant');
fprintf('analyzeResults OK\n');
