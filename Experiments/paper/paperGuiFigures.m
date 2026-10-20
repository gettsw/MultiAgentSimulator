function files = paperGuiFigures(kind, outDir, opts)
% Figures of the simulator paper that show the GUI, captured from MainUI. The windows open on
% the secondary monitor when there is one and close when done. Returns the PNG files written:
%   "overview"  Fig. 2: main window, reference scenario after a short run
%   "setup"     Fig. 9: main window, reference scenario at t = 0
%   "planners"  Fig. 12: canvas of the reference scenario under each path planner
%   "reel"      Fig. 3: canvas snapshots of a run on ExampleLayout1 at six times
%   "examples"  Figs. 5-8: ExampleLayout1 after several runs, with Default agents (Random Walk)
%               and with Energy agents (Battery Efficient): canvas and objective window
% Every action goes through the same callbacks as the GUI controls.
arguments
    kind (1,1) string
    outDir (1,1) string
    opts.EndTime (1,1) double = 500   % run length for "reel" and "examples"
    opts.Runs (1,1) double = 4        % runs per example in "examples"
    opts.Monitor (1,1) double = MainUI.secondaryMonitor()
end
if ~isfolder(outDir), mkdir(outDir); end
root = fileparts(which('MainUI'));
switch kind
    case "overview", files = overview(root, outDir, opts);
    case "setup",    files = setup(root, outDir, opts);
    case "planners", files = planners(root, outDir, opts);
    case "reel",     files = reel(root, outDir, opts);
    case "examples", files = examples(root, outDir, opts);
    otherwise, error('paperGuiFigures:UnknownKind', 'Unknown figure "%s"', kind);
end
end

function files = overview(root, outDir, opts)
ui = openReference(root, opts);
cleanup = onCleanup(@() closeUI(ui));
setEndTime(ui, min(60, opts.EndTime));
press(ui, 'Run to End');
files = fullfile(outDir, "fig02_overview.png");
exportapp(ui.fig, files);
end

function files = setup(root, outDir, opts)
ui = openReference(root, opts);
cleanup = onCleanup(@() closeUI(ui));
files = fullfile(outDir, "fig09_setup.png");
exportapp(ui.fig, files);
end

function files = planners(root, outDir, opts)
ui = openReference(root, opts);
cleanup = onCleanup(@() closeUI(ui));
names = ["Dijkstra Corners", "Energy Efficient"];
files = fullfile(outDir, ["fig12_planner_dijkstra.png", "fig12_planner_energy.png"]);
for k = 1:numel(names)
    choose(ui.plannerDropdown, names(k));
    saveCanvas(ui, files(k));
end
end

function files = reel(root, outDir, opts)
ui = openExample(root, opts, 'Energy Agent', 'Battery Efficient');
cleanup = onCleanup(@() closeUI(ui));
times = round(linspace(0, opts.EndTime, 6));
files = fullfile(outDir, compose("fig03_reel_t%03d.png", times'));
rng(1);
for k = 1:numel(times)
    if times(k) > 0
        ui.sim.setEndTime(times(k));
        ui.sim.runToEnd();    % continues from the previous snapshot
    end
    saveCanvas(ui, files(k));
end
end

function files = examples(root, outDir, opts)
% agent type, policy, file tag, figure numbers (layout, objective)
cases = {'Default Agent', 'Random Walk', 'default', [5 7]; 'Energy Agent', 'Battery Efficient', 'energy', [6 8]};
files = strings(0, 1);
for c = 1:size(cases, 1)
    ui = openExample(root, opts, cases{c, 1}, cases{c, 2});
    cleanup = onCleanup(@() closeUI(ui));
    setEndTime(ui, opts.EndTime);
    for r = 1:opts.Runs
        rng(r);
        press(ui, 'Run to End');   % resets a finished run and starts a new line in the objective window
    end
    f = fullfile(outDir, compose("fig%02d_example_%s_", cases{c, 4}', cases{c, 3}) + ["layout.png"; "objective.png"]);
    saveCanvas(ui, f(1));
    exportapp(ui.objectiveWin.fig, f(2));
    clear cleanup   % closes this case's windows
    files = [files; f]; %#ok<AGROW>
end
end

% ---- GUI actions ----
function ui = openReference(root, opts)
% Reference scenario over the Stevens Smart Lab map, components as in Sec. IV
ui = MainUI('Monitor', opts.Monitor);
ui.renderer.loadBackgroundImage(fullfile(root, 'LabLayout.png'));
ui.loadLayout(fullfile(root, 'Configurations', 'ReportLayout.mat'));
choose(ui.policyDropdown, 'Battery Efficient');
end

function ui = openExample(root, opts, agentType, policy)
% ExampleLayout1 (4 targets) with one agent of the given type on every target
ui = MainUI('Monitor', opts.Monitor);
choose(ui.agentTypeDropdown, agentType);
ui.loadLayout(fullfile(root, 'Configurations', 'ExampleLayout1.mat'));
press(ui, 'Add Agent');
for t = 1:numel(ui.model.targets)
    ui.controller.onCanvasClick(ui.model.targets(t).position, ui.speedField.Value);
end
press(ui, 'Idle');
choose(ui.policyDropdown, policy);
end

function choose(dropdown, value)
% Same as picking value in the dropdown
dropdown.Value = value;
dropdown.ValueChangedFcn(dropdown, []);
end

function setEndTime(ui, T)
ui.endTimeField.Value = T;
ui.endTimeField.ValueChangedFcn(ui.endTimeField, []);
end

function press(ui, text)
b = findall(ui.fig, 'Type', 'uibutton', 'Text', text);
b.ButtonPushedFcn(b, []);
end

function saveCanvas(ui, file)
drawnow;
exportgraphics(ui.ax, file, 'Resolution', 200);
end

function closeUI(ui)
if isvalid(ui.fig), close(ui.fig); end
end
