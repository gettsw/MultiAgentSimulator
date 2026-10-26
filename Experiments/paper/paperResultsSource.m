%% Simulator Paper Results
% This live script regenerates every result reported in the simulator paper
% (|Paper_PerMonSimulator/ArXiv.tex|): Tables II-VI, the numbers quoted in the text, and the
% figures made by code. Run all sections (Live Editor: *Run All*). To get the same report
% without the Live Editor, run from the repository root:
%
%   matlab -batch "addpath(genpath(pwd)); exportPaperResults"
%
% The case studies are defined in |paperDefaults| (the reference scenario) and
% |paperConditions| (what each case study varies). The outputs go to |results/paper/| and, when
% the paper's folder sits next to this repository (|../Paper_PerMonSimulator|), its
% |generated/| folder receives the tables, macros and figures the paper includes. GUI windows
% open on a secondary monitor when there is one and close when their figure is saved. This
% file is built from |Experiments/paper/paperResultsSource.m| by |buildPaperResults|.

%% Settings
% |QuickMode| shortens every run to 60 s, uses at most two seeds and one validation problem,
% to check the pipeline in a few minutes. A full run takes about an hour.
QuickMode = false;
quick = getenv("PAPER_RESULTS_QUICK");   % set by exportPaperResults(Quick=true)
if quick ~= "", QuickMode = quick == "1"; end
root = fileparts(which('MainUI'));
assert(root ~= "", 'Make the repository root (the folder with MainUI.m) the current folder.');
addpath(genpath(root));
rmpath(genpath(fullfile(root, 'Old_Design')));
warning('off', 'runTrial:AgentPolicyFit');
base = paperDefaults();
outDir = fullfile(root, 'results', 'paper');
presets = 1:8;
if QuickMode
    outDir = fullfile(root, 'results', 'paper_quick');
    presets = 1;
end
figDir = fullfile(outDir, 'figures');
fprintf('%s mode, output in %s\n', string(QuickMode), outDir);

%% Fixed parameters of the case studies
% Every value is read from the code and the layout file; the paper states them in the text.
parameters = paperTables("parameters", base)

%% Tables II and III: validation against the JS simulator
% ED-RHC (RHC and RHC-alpha) and ED-ORHC (SO and FO) on the eight presets of the legacy JS
% simulator, under its settings (Experiments/validation).
Ted = validateEDRHC(Presets=presets);
Teo = validateEDORHC(Presets=presets);
validation = paperTables("validation", Ted, Teo)

%% Table IV and Fig. 9: policies and agent types
% Random Walk (ten seeds), ED-RHC, ED-ORHC-SO and ED-ORHC-FO-3 with Abstract and Physical Agents
% on the reference scenario (Dijkstra Corners).
policies = paperRuns(base, paperConditions("policies"), Quick=QuickMode);
paperTables("policies", policies)
paperPlots("objectives", policies, fullfile(figDir, "fig_objectives"));

%% Table V and Fig. 10: path planners
% Dijkstra Corners, Energy Efficient and Grid A* with ED-RHC and Physical Agents. Length and
% Transit total the planned edges and the Physical Agent's shortest transit over each.
planners = paperRuns(base, paperConditions("planners"), Quick=QuickMode);
paperTables("planners", planners, base)
showImages(paperGuiFigures("planners", figDir));

%% Fig. 11: energy trade-off of ED-ORHC-SO
% The energy weight follows App. A of the energy-aware paper for each beta; Abstract Agents
% with the speed and acceleration limits.
tradeoff = paperRuns(base, paperConditions("tradeoff"), Quick=QuickMode);
paperTables("tradeoff", tradeoff)
paperPlots("tradeoff", tradeoff, fullfile(figDir, "fig_tradeoff"));

%% Table VI and Fig. 12: battery-aware ED-RHC with a charging station
% Office floor with a 300 W charging dock and 2.5 Wh batteries that stop the agents when empty.
battery = paperRuns(base, paperConditions("battery"), Quick=QuickMode);
paperTables("battery", battery)
paperPlots("soc", battery, fullfile(figDir, "fig_soc"));

%% Figs. 3 and 5: the GUI and the scenario library
guiFiles = paperGuiFigures("overview", figDir);
guiFiles = [guiFiles(:); reshape(paperGuiFigures("scenarios", figDir), [], 1)];
showImages(guiFiles);

%% Figs. 6-8: getting started (Examples/EventDetection)
% The walkthrough script builds the museum scenario through the GUI's click handlers, runs it,
% and compares policies headless; its canvas snapshots are the steps shown in the paper.
if ~QuickMode
    runExample(root);
    close all force
    copyfile(fullfile(root, 'results', 'example', '*.png'), figDir);
end

%% Output files
% |runs.csv| (every run), LaTeX tables and macros for the paper, and the figures (PNG; the
% plots also as PDF) in |figures/|; then copied into the paper's |generated/| folder.
files = writePaperOutputs(outDir, struct('base', base, 'validation', validation, ...
    'policies', policies, 'planners', planners, 'tradeoff', tradeoff, 'battery', battery))
paperDir = fullfile(fileparts(root), 'Paper_PerMonSimulator');
if ~QuickMode && isfolder(paperDir)
    gen = fullfile(paperDir, 'generated');
    if ~isfolder(gen), mkdir(gen); end
    copyfile(fullfile(outDir, '*.tex'), gen);
    copyfile(fullfile(figDir, '*'), gen);
    fprintf('Copied the tables, macros and figures to %s\n', gen);
end
writelines("Finished " + string(datetime("now")), fullfile(outDir, "finished.txt"));   % read by exportPaperResults

function runExample(root)
% The walkthrough in its own workspace (it defines root and outDir too)
run(fullfile(root, 'Examples', 'EventDetection', 'eventDetectionExample.m'));
end
