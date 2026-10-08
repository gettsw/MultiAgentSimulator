% Ablation of Joint RHC design choices vs Classical RHC, on every uncertainty mode.
% Each row sets policy properties via Policy_<property> parameters.
J = "Joint RHC";
configs = {
    % ---- baselines ----
    "Classical",                     struct('Policy', "Classical RHC [1]")
    "Classical + clear dwell",       struct('Policy', "Classical RHC [1]", 'Policy_dwellRule', "clear")
    % ---- optimized dwell (fmincon) ----
    "Joint pair / off",              struct('Policy', J, 'Policy_divergeScoring', "pair",      'Policy_mergeGate', "off")
    "Joint classical / off",         struct('Policy', J, 'Policy_divergeScoring', "classical", 'Policy_mergeGate', "off")
    "Joint classical / useful",      struct('Policy', J, 'Policy_divergeScoring', "classical", 'Policy_mergeGate', "useful")
    % ---- clear-to-zero dwell ----
    "Joint clear: pair / off",       struct('Policy', J, 'Policy_dwellRule', "clear", 'Policy_divergeScoring', "pair",      'Policy_mergeGate', "off")
    "Joint clear: pair / capacity",  struct('Policy', J, 'Policy_dwellRule', "clear", 'Policy_divergeScoring', "pair",      'Policy_mergeGate', "capacity")
    "Joint clear: class / off",      struct('Policy', J, 'Policy_dwellRule', "clear", 'Policy_divergeScoring', "classical", 'Policy_mergeGate', "off")
    "Joint clear: class / capacity", struct('Policy', J, 'Policy_dwellRule', "clear", 'Policy_divergeScoring', "classical", 'Policy_mergeGate', "capacity")
    "Joint clear: class / useful",   struct('Policy', J, 'Policy_dwellRule', "clear", 'Policy_divergeScoring', "classical", 'Policy_mergeGate', "useful")
};
modes = ["Static Uniform", "Static Priority Target", "Dynamic Uncertainty"];

results = zeros(size(configs, 1), numel(modes));
for mi = 1:numel(modes)
    fprintf('\n== %s ==\n', modes(mi));
    for i = 1:size(configs, 1)
        p = configs{i, 2};
        p.UncertaintyMode = modes(mi);
        p.EndTime = 300;
        out = runTrial(p);
        results(i, mi) = out.finalJ;
        fprintf('%-31s J=%8.2f  slope=%6.3f  co-loc=%6.1fs\n', configs{i, 1}, out.finalJ, out.slopeJ, out.coLocationTime);
    end
end

% Summary: J relative to Classical per mode, and the geometric mean across modes
rel = results ./ results(1, :);
fprintf('\n%-31s %8s %8s %8s | %8s\n', 'relative to Classical', 'Uniform', 'Priority', 'Dynamic', 'geomean');
for i = 1:size(configs, 1)
    fprintf('%-31s %8.2f %8.2f %8.2f | %8.2f\n', configs{i, 1}, rel(i, :), exp(mean(log(rel(i, :)))));
end
