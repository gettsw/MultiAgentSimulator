%% Simulator Paper Results
% This live script regenerates every result reported in the simulator paper
% (|Paper_PerMonSimulator/ArXiv.tex|): Tables I-IV and Figs. 2, 3, and 5-13. Run all sections
% (Live Editor: *Run All*). To get the same report without the Live Editor, run from the
% repository root:
%
%   matlab -batch "addpath(genpath(pwd)); exportPaperResults"
%
% Fig. 1 (the legacy JavaScript simulator) and Fig. 4 (the UML diagram, drawn in LaTeX) are not
% produced by code. GUI windows open on a secondary monitor when there is one and close when
% their figure is saved.
%
% The experiments are defined in |paperDefaults| (the reference scenario) and |paperConditions|
% (what each experiment varies); change them there to change the paper's experiments. This
% file is built from |Experiments/paper/paperResultsSource.m| by |buildPaperResults|.

%% Settings
% |QuickMode| shortens every run to 60 s and uses 2 seeds, to check the pipeline in about two
% minutes. A full run takes about ten minutes.
QuickMode = false;
quick = getenv("PAPER_RESULTS_QUICK");   % set by exportPaperResults(Quick=true)
if quick ~= "", QuickMode = quick == "1"; end
root = fileparts(which('MainUI'));
assert(root ~= "", 'Make the repository root (the folder with MainUI.m) the current folder.');
addpath(genpath(root));
rmpath(genpath(fullfile(root, 'Old_Design')));
base = paperDefaults();
seeds = 1:4;
outDir = fullfile(root, 'results', 'paper');
modeName = "Full";
if QuickMode
    modeName = "Quick";
    base.EndTime = 60;
    seeds = 1:2;
    outDir = fullfile(root, 'results', 'paper_quick');
end
figDir = fullfile(outDir, 'figures');
fprintf('%s mode: T = %g s, seeds %s, output in %s\n', modeName, base.EndTime, mat2str(seeds), outDir);

%% Table I: fixed parameters
% Every value is read from the code and the layout file.
parameters = paperTables("parameters", base, seeds)

%% Figs. 9 and 2: the reference scenario in the GUI
% |Configurations/ReportLayout.mat| over the Stevens Smart Lab map: 11 targets, 10 edges,
% 3 walls, and 5 agents on targets 1, 6, 2, 3, and 5. Fig. 9 shows it at t = 0 and Fig. 2
% after a short run with the Battery Efficient policy.
showImages([paperGuiFigures("setup", figDir); paperGuiFigures("overview", figDir)]);

%% Table II and Figs. 10-11: movement policies
% Random Walk vs Battery Efficient, with the Dijkstra Corners planner and Default Agents.
policy = paperRuns(base, paperConditions("policy"), seeds);
paperTables("policy", policy)
paperPlots("objective", policy, fullfile(figDir, "fig10_objective_random_walk"), Condition="Random Walk");
paperPlots("objective", policy, fullfile(figDir, "fig11_objective_battery_efficient"), Condition="Battery Efficient");

%% Table III and Fig. 12: path planners
% Dijkstra Corners vs Energy Efficient, with the Battery Efficient policy and Default Agents.
% L, Theta, and E_a are summed over the five agents; all values are means over the runs.
planner = paperRuns(base, paperConditions("planner"), seeds);
paperTables("planner", planner)
showImages(paperGuiFigures("planners", figDir));

%% Table IV and Fig. 13: Energy Agents
% Both planners again, with Energy Agents. SOC is the mean over agents and runs; Energy is the
% team's energy drawn from the batteries, averaged over the runs.
battery = paperRuns(base, paperConditions("battery"), seeds);
paperTables("battery", battery)
paperPlots("soc", battery, fullfile(figDir, "fig13_soc"));

%% Fig. 3: a run in the GUI
% |Configurations/ExampleLayout1.mat| with four Energy Agents and the Battery Efficient policy,
% captured at six times.
showImages(paperGuiFigures("reel", figDir, EndTime=base.EndTime));

%% Figs. 5-8: agent types and policies in the GUI
% |ExampleLayout1.mat| after several runs, with Default Agents and Random Walk (Figs. 5 and 7)
% and with Energy Agents and Battery Efficient (Figs. 6 and 8).
showImages(paperGuiFigures("examples", figDir, EndTime=base.EndTime, Runs=numel(seeds)));

%% Check against the draft
% The draft's numbers were produced by hand in the GUI before Random Walk and Battery
% Efficient dwelled at targets. Running the three experiments again with the dwell set to zero
% recreates those conditions, so the |DraftConditions| column should be close to |Draft|
% (Random Walk varies from run to run). |Current| holds the results above. The check needs the
% draft's 500-s runs, so QuickMode skips it.
comparison = table();
if QuickMode
    disp("Skipped in QuickMode.")
else
    draft.policy = paperRuns(base, paperConditions("policy", NoDwell=true), seeds);
    draft.planner = paperRuns(base, paperConditions("planner", NoDwell=true), seeds);
    draft.battery = paperRuns(base, paperConditions("battery", NoDwell=true), seeds);
    current = struct('policy', policy, 'planner', planner, 'battery', battery);
    comparison = paperComparison(current, draft)
end

%% Output files
% |runs.csv| (every run), |comparison.csv|, LaTeX tables and macros for the paper, and the
% figures (PNG; the plots also as PDF) in |figures/|.
files = writePaperOutputs(outDir, struct('parameters', parameters, 'policy', policy, ...
    'planner', planner, 'battery', battery, 'comparison', comparison))
writelines("Finished " + string(datetime("now")), fullfile(outDir, "finished.txt"));   % read by exportPaperResults
