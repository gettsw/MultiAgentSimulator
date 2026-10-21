function report = exportPaperResults(opts)
% Runs PaperResults.mlx without the Live Editor and saves its report (text, code, outputs and
% figures) as results/paper/PaperResults.pdf (results/paper_quick/ with Quick = true).
%   matlab -batch "addpath(genpath(pwd)); exportPaperResults"
%   matlab -batch "addpath(genpath(pwd)); exportPaperResults(Quick=true)"
% The run stops at the first error, which the report shows in red; this function then errors.
arguments
    opts.Quick (1,1) logical = false
    opts.Format (1,1) string = "pdf"    % or "html", "docx", "latex", "markdown"
end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));

% Toolboxes on the path can shadow MATLAB's live script export (YALMIP ships an export.m)
shadows = strings(0, 1);
while ~isempty(which('export')) && ~startsWith(which('export'), matlabroot)
    shadows(end+1) = fileparts(which('export')); %#ok<AGROW>
    rmpath(shadows(end));
end
restorePath = onCleanup(@() restorePaths(shadows));

setenv('PAPER_RESULTS_QUICK', string(double(opts.Quick)));
clearEnv = onCleanup(@() setenv('PAPER_RESULTS_QUICK', ''));
outDir = fullfile(root, 'results', 'paper');
if opts.Quick, outDir = fullfile(root, 'results', 'paper_quick'); end
if ~isfolder(outDir), mkdir(outDir); end
oldDir = cd(root);
back = onCleanup(@() cd(oldDir));
ext = struct('pdf', ".pdf", 'html', ".html", 'docx', ".docx", 'latex', ".tex", 'markdown', ".md");
% The live script writes finished.txt in its last section. export's own CatchError = false is not
% used: it also fails on errors that code catches itself (it reads lasterror).
marker = fullfile(outDir, "finished.txt");
if isfile(marker), delete(marker); end
report = export(fullfile(root, 'PaperResults.mlx'), fullfile(outDir, "PaperResults" + ext.(opts.Format)), Run=true);
fprintf('Report: %s\n', report);
assert(isfile(marker), 'exportPaperResults:Failed', ...
    'PaperResults.mlx stopped before its last section; the report shows the error in red.');
end

function restorePaths(folders)
if ~isempty(folders), addpath(folders{:}); end
end
