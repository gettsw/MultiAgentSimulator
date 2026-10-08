% Why do the Joint RHC variants win where they win? For each mode and variant, prints
% J, decision mix, dwell distribution, and (shadowCompare) how often the OTHER diverge
% scoring would have chosen a different target / merge decision / dwell.
variants = {
    "Classical-equivalent",      struct('Policy_useMergeDiverge', false)
    "Pair scoring, gate off",    struct('Policy_divergeScoring', "pair", 'Policy_mergeGate', "off")
    "Classical scoring, off",    struct('Policy_divergeScoring', "classical", 'Policy_mergeGate', "off")
    "Classical scoring, useful", struct('Policy_divergeScoring', "classical", 'Policy_mergeGate', "useful")
};
modes = ["Static Uniform", "Static Priority Target", "Dynamic Uncertainty"];

for m = modes
    fprintf('\n== %s ==\n', m);
    fprintf('%-26s %8s | %5s %5s %5s %5s | %5s %5s %5s | %6s %6s %9s\n', 'variant', 'J', ...
        'merge', 'div', 'singl', 'coLoc', 'dwell', '%lo', '%hi', '%tgtDf', '%mrgDf', 'dwell alt');
    for i = 1:size(variants, 1)
        p = variants{i, 2};
        p.Policy = "Joint RHC";
        p.UncertaintyMode = m;
        p.EndTime = 300;
        p.Policy_shadowCompare = true;
        out = runTrial(p);
        s = out.policy.stats;
        n = max(s.dwellN, 1); k = max(s.shadowN, 1);
        fprintf('%-26s %8.2f | %5d %5d %5d %5d | %5.2f %5.0f %5.0f | %6.0f %6.0f %4.2f/%4.2f\n', ...
            variants{i, 1}, out.finalJ, s.merge, s.diverge, s.single, s.coLocated, ...
            s.dwellSum / n, 100 * s.dwellAtLower / n, 100 * s.dwellAtUpper / n, ...
            100 * s.shadowTargetDiff / k, 100 * s.shadowMergeDiff / k, ...
            s.shadowChosenDwellSum / k, s.shadowDwellSum / k);
    end
end
fprintf(['\n%%tgtDf / %%mrgDf: share of pairwise decisions where the other scoring picks a different\n' ...
         'target / a different merge-vs-diverge.  dwell alt: mean dwell chosen / other scoring.\n']);
