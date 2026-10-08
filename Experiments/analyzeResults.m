function R = analyzeResults(T, opts)
% Statistical comparison of policy variants from Experiment Manager results.
%
%   R = analyzeResults(T)                      % T = exported results table
%   R = analyzeResults({Tstatic, Tdynamic})    % several experiments, merged
%   R = analyzeResults(T, 'Baseline', "Classical RHC [1] | dwellRule=clear", 'By', "NumAgents")
%
% A "variant" is the Policy plus any Policy_<switch> settings that policy actually has.
% Runs are PAIRED: each variant run is compared with the baseline run that has identical
% values for every other parameter (layout, agents, uncertainty mode, B, A_spike, ...).
%
% Returns R.Summary (per mode x variant), R.Paired (per mode x variant vs baseline) and
% R.Data (merged rows with Variant and pairing Key). Optionally plots and saves CSV/PNG.
arguments
    T
    opts.Baseline (1,1) string = "Classical RHC [1]"
    opts.Metric (1,1) string = "finalJ"
    opts.By (1,1) string = ""            % parameter for the J-vs-parameter plot, e.g. "A_spike"
    opts.BoundedSlope (1,1) double = 0.05 % slopeJ below this counts as bounded
    opts.NBoot (1,1) double = 2000
    opts.Plot (1,1) logical = true
    opts.SaveTo (1,1) string = ""        % folder for Summary.csv, Paired.csv and figures
end

defaults = trialDefaults();
T = mergeTables(T, defaults);
assert(ismember(opts.Metric, string(T.Properties.VariableNames)), 'Metric "%s" not in results', opts.Metric);

% ---- Variants and pairing keys ----
policyCols = string(T.Properties.VariableNames(startsWith(T.Properties.VariableNames, 'Policy_')));
T.Variant = variantLabels(T, policyCols);
keyCols = setdiff(string(fieldnames(defaults)), "Policy", 'stable');
T.Key = strings(height(T), 1);
for c = keyCols'
    T.Key = T.Key + "|" + string(T.(c));
end

variants = unique(T.Variant, 'stable');
if ~ismember(opts.Baseline, variants)
    error('Baseline "%s" not found. Variants in the data:\n  %s', opts.Baseline, strjoin(variants, newline + "  "));
end
modes = unique(T.UncertaintyMode, 'stable');
y = T.(opts.Metric);

% ---- Summary per mode x variant ----
[g, gMode, gVar] = findgroups(T.UncertaintyMode, T.Variant);
q = @(v, p) quantileBase(v, p);
Summary = table(gMode, gVar, splitapply(@numel, y, g), splitapply(@median, y, g), ...
    splitapply(@(v) q(v, 0.25), y, g), splitapply(@(v) q(v, 0.75), y, g), splitapply(@mean, y, g), ...
    'VariableNames', {'Mode', 'Variant', 'N', 'Median', 'Q25', 'Q75', 'Mean'});
if ismember("slopeJ", string(T.Properties.VariableNames))
    Summary.FracBounded = splitapply(@(s) mean(s < opts.BoundedSlope), T.slopeJ, g);
end

% ---- Paired comparison vs baseline ----
rs = RandStream('mt19937ar', 'Seed', 0);   % reproducible bootstrap
useSignrank = exist('signrank', 'file') == 2;
rows = {};
ratios = struct('Mode', {}, 'Variant', {}, 'LogRatio', {});
for m = modes'
    inMode = T.UncertaintyMode == m;
    base = T(inMode & T.Variant == opts.Baseline, :);
    [bKeys, ~, bIdx] = unique(base.Key);
    bY = accumarray(bIdx, base.(opts.Metric), [], @mean);   % duplicate baseline runs are identical
    for v = setdiff(variants, opts.Baseline, 'stable')'
        cur = T(inMode & T.Variant == v, :);
        [found, loc] = ismember(cur.Key, bKeys);
        if ~any(found), continue; end
        yv = cur.(opts.Metric)(found);
        yb = bY(loc(found));
        ratio = max(yv, eps) ./ max(yb, eps);
        n = numel(ratio);
        meds = sort(median(ratio(randi(rs, n, n, opts.NBoot)), 1));
        ciLo = meds(max(1, floor(0.025 * opts.NBoot)));
        ciHi = meds(ceil(0.975 * opts.NBoot));
        d = log(ratio);
        if useSignrank && n >= 2
            p = signrank(d); test = "signrank";
        else
            p = signTest(d); test = "sign";
        end
        rows(end+1, :) = {m, v, n, median(ratio), ciLo, ciHi, mean(yv < yb), mean(yv > yb), p, test}; %#ok<AGROW>
        ratios(end+1) = struct('Mode', m, 'Variant', v, 'LogRatio', log2(ratio)); %#ok<AGROW>
    end
end
pairedNames = {'Mode', 'Variant', 'NPairs', 'MedianRatio', 'CI95Lo', 'CI95Hi', 'WinRate', 'LossRate', 'PValue', 'Test'};
if isempty(rows)
    warning('No variant run has a baseline run with identical parameters; nothing to pair.');
    rows = cell(0, numel(pairedNames));
end
Paired = cell2table(rows, 'VariableNames', pairedNames);

fprintf('\n=== %s by mode and variant ===\n', opts.Metric); disp(Summary);
fprintf('=== Paired vs baseline "%s" (ratio < 1 = better than baseline) ===\n', opts.Baseline); disp(Paired);

R = struct('Summary', Summary, 'Paired', Paired, 'Data', T);

% ---- Plots ----
figs = gobjects(0);
if opts.Plot && ~isempty(ratios)
    figs(end+1) = figure('Name', 'Paired ratios vs baseline');
    tl = tiledlayout(1, numel(modes), 'TileSpacing', 'compact');
    for m = modes'
        ax = nexttile(tl);
        sel = ratios([ratios.Mode] == m);
        if isempty(sel), continue; end
        x = categorical(repelem([sel.Variant]', arrayfun(@(s) numel(s.LogRatio), sel)));
        boxchart(ax, x, vertcat(sel.LogRatio));
        yline(ax, 0, '--');
        title(ax, m); ylabel(ax, sprintf('log_2(%s / baseline)', opts.Metric));
    end
    title(tl, "Paired comparison vs " + opts.Baseline, 'Interpreter', 'none');
end
if opts.Plot && opts.By ~= ""
    assert(ismember(opts.By, string(T.Properties.VariableNames)), 'By "%s" not in results', opts.By);
    figs(end+1) = figure('Name', "Median " + opts.Metric + " vs " + opts.By);
    tl = tiledlayout(1, numel(modes), 'TileSpacing', 'compact');
    for m = modes'
        ax = nexttile(tl); hold(ax, 'on');
        for v = variants'
            sel = T.UncertaintyMode == m & T.Variant == v;
            if ~any(sel), continue; end
            [gx, xv] = findgroups(T.(opts.By)(sel));
            plot(ax, xv, splitapply(@median, y(sel), gx), '-o', 'DisplayName', v);
        end
        title(ax, m); xlabel(ax, opts.By); ylabel(ax, "median " + opts.Metric);
        legend(ax, 'Interpreter', 'none', 'Location', 'best');
    end
end

% ---- Save ----
if opts.SaveTo ~= ""
    if ~isfolder(opts.SaveTo), mkdir(opts.SaveTo); end
    writetable(Summary, fullfile(opts.SaveTo, 'Summary.csv'));
    writetable(Paired, fullfile(opts.SaveTo, 'Paired.csv'));
    for k = 1:numel(figs)
        exportgraphics(figs(k), fullfile(opts.SaveTo, sprintf('figure%d.png', k)));
    end
end
end

% =====================================================================
function T = mergeTables(Tin, defaults)
% Normalise one or more results tables to: every trial parameter present with a fixed type,
% Policy_* switches as strings ("default" when absent), known outputs numeric (NaN if absent).
if istable(Tin), Tin = {Tin}; end
outputs = ["finalJ", "steadyJ", "slopeJ", "peakR", "coLocationTime"];
params = string(fieldnames(defaults))';
policyCols = strings(1, 0);
for k = 1:numel(Tin)
    names = string(Tin{k}.Properties.VariableNames);
    policyCols = union(policyCols, names(startsWith(names, "Policy_")), 'stable');
end
for k = 1:numel(Tin)
    t = Tin{k}; h = height(t);
    out = table();
    for c = params
        d = defaults.(c);
        if ~ismember(c, string(t.Properties.VariableNames))
            col = repmat(d, h, 1);
        elseif isstring(d)
            col = string(t.(c));
        else
            col = t.(c);
            if ~isnumeric(col), col = str2double(string(col)); end
            col = double(col);
        end
        out.(c) = col;
    end
    for c = policyCols
        if ismember(c, string(t.Properties.VariableNames))
            out.(c) = string(t.(c));
        else
            out.(c) = repmat("default", h, 1);
        end
    end
    for c = outputs
        if ismember(c, string(t.Properties.VariableNames))
            out.(c) = double(t.(c));
        else
            out.(c) = nan(h, 1);
        end
    end
    Tin{k} = out;
end
T = vertcat(Tin{:});
end

function labels = variantLabels(T, policyCols)
% Policy name plus the Policy_* settings that apply to that policy class
model = ScenarioModel();
labels = T.Policy;
for c = policyCols
    name = extractAfter(c, "Policy_");
    applies = arrayfun(@(pn) isprop(model.policyMap(char(pn)), name), T.Policy);
    use = applies & T.(c) ~= "default";
    labels(use) = labels(use) + " | " + name + "=" + T.(c)(use);
end
end

function v = quantileBase(x, p)
% Linear-interpolated quantile without the Statistics Toolbox
x = sort(x(~isnan(x)));
if isempty(x), v = NaN; return; end
pos = 1 + p * (numel(x) - 1);
v = x(floor(pos)) + (pos - floor(pos)) * (x(ceil(pos)) - x(floor(pos)));
end

function p = signTest(d)
% Exact two-sided sign test on paired differences (no toolbox needed)
d = d(d ~= 0 & ~isnan(d));
n = numel(d);
if n == 0, p = 1; return; end
k = min(sum(d < 0), sum(d > 0));
i = 0:k;
logPmf = gammaln(n + 1) - gammaln(i + 1) - gammaln(n - i + 1) - n * log(2);
p = min(1, 2 * sum(exp(logPmf)));
end
