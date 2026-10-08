function T = runSweep(grid, opts)
% Run every combination of the parameter values in GRID and return one table row per trial.
%
%   T = runSweep(struct('Policy', ["Joint RHC","Classical RHC [1]"], ...
%                       'UncertaintyMode', ["Static Uniform","Dynamic Uncertainty"], ...
%                       'AgentStarts', {{[1 5], [2 6]}}, 'EndTime', 300));
%
% A field holding several vectors (like AgentStarts) is given as a cell array; note the
% double braces inside struct(), which otherwise expands cells into a struct array.
%   R = analyzeResults(T);
%
% Field names are trial parameters (see trialDefaults) or Policy_<switch>; unset parameters
% use the defaults. A switch that does not apply to a policy (e.g. a Joint-only switch on
% Classical) is recorded as "n/a" and the duplicate run is skipped.
arguments
    grid (1,1) struct
    opts.Parallel (1,1) logical = false   % parfor (Parallel Computing Toolbox)
    opts.SaveTo (1,1) string = ""         % .mat file to save T to
end

names = string(fieldnames(grid))';
vals = cellfun(@(n) toCell(grid.(n)), cellstr(names), 'UniformOutput', false);
if isfield(grid, 'AgentStarts') && isnumeric(grid.AgentStarts)
    vals{names == "AgentStarts"} = {grid.AgentStarts};   % one start vector, not one value per agent
end
ranges = cellfun(@(v) 1:numel(v), vals, 'UniformOutput', false);
idx = cell(1, numel(names));
[idx{:}] = ndgrid(ranges{:});

% ---- Expand the grid, marking switches that do not apply, and drop duplicates ----
model = ScenarioModel();
defaultPolicy = trialDefaults().Policy;
params = cell(numel(idx{1}), 1);
keys = strings(numel(idx{1}), 1);
for c = 1:numel(idx{1})
    p = struct();
    for k = 1:numel(names)
        v = vals{k}{idx{k}(c)};
        if isnumeric(v) && numel(v) > 1, v = string(mat2str(v)); end   % e.g. AgentStarts "[1 5]"
        p.(names(k)) = v;
    end
    pol = model.policyMap(char(getfieldOr(p, 'Policy', defaultPolicy)));
    for n = names(startsWith(names, "Policy_"))
        p.(n) = string(p.(n));
        if ~isprop(pol, extractAfter(n, "Policy_")), p.(n) = "n/a"; end
    end
    params{c} = p;
    keys(c) = strjoin(string(struct2cell(p)), "|");
end
[~, keep] = unique(keys, 'stable');
params = params(keep);
nT = numel(params);

% ---- Run ----
fprintf('Running %d trials...\n', nT);
results = cell(nT, 1);
tic;
if opts.Parallel
    parfor i = 1:nT
        results{i} = metricsOnly(runTrial(params{i}));
    end
else
    for i = 1:nT
        results{i} = metricsOnly(runTrial(params{i}));
        fprintf('  [%d/%d] %s  J=%.2f  (%.0fs elapsed)\n', i, nT, keys(keep(i)), results{i}.finalJ, toc);
    end
end
fprintf('Done in %.0f s.\n', toc);

rows = cellfun(@mergeStructs, params, results, 'UniformOutput', false);
T = struct2table(vertcat(rows{:}));
if opts.SaveTo ~= ""
    save(opts.SaveTo, 'T');
    fprintf('Saved to %s\n', opts.SaveTo);
end
end

function c = toCell(x)
if iscell(x), c = x(:)'; else, c = num2cell(x); end
end

function m = metricsOnly(out)
m = rmfield(out, 'policy');   % keep rows small (and parfor-transferable)
end

function s = mergeStructs(a, b)
s = a;
for f = fieldnames(b)'
    s.(f{1}) = b.(f{1});
end
end

function v = getfieldOr(s, name, default)
if isfield(s, name), v = s.(name); else, v = default; end
end
