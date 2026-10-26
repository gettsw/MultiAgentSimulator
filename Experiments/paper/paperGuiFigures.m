function files = paperGuiFigures(kind, outDir, opts)
% Figures of the simulator paper that show the GUI, captured from MainUI. The windows open on
% the secondary monitor when there is one and close when done. Returns the PNG files written:
%   "overview"   Fig. 3: main window, reference scenario (Physical Agents, ED-RHC) after 60 s
%   "scenarios"  Fig. 5: canvas of each map scenario of the Scenario dropdown, as loaded
%   "planners"   Fig. 10: canvas of the reference scenario under each path planner
% Every action goes through the same callbacks as the GUI controls.
arguments
    kind (1,1) string
    outDir (1,1) string
    opts.Monitor (1,1) double = MainUI.secondaryMonitor()
end
if ~isfolder(outDir), mkdir(outDir); end
root = fileparts(which('MainUI'));
switch kind
    case "overview",  files = overview(root, outDir, opts);
    case "scenarios", files = scenarios(outDir, opts);
    case "planners",  files = planners(root, outDir, opts);
    otherwise, error('paperGuiFigures:UnknownKind', 'Unknown figure "%s"', kind);
end
end

function files = overview(root, outDir, opts)
ui = openReference(root, opts);
cleanup = onCleanup(@() closeUI(ui));
setEndTime(ui, 60);
rng(1);
press(ui, 'Run to End');
files = fullfile(outDir, "fig_overview.png");
exportapp(ui.fig, files);
end

function files = scenarios(outDir, opts)
ui = MainUI('Monitor', opts.Monitor);
cleanup = onCleanup(@() closeUI(ui));
choose(ui.agentTypeDropdown, 'Physical Agent');
names = ["Lab floor (Stevens)", "Office floor with charging dock", "City patrol", "Harbor patrol", ...
         "Campus", "Museum (event detection example)"];
tags = ["lab", "office", "city", "harbor", "campus", "museum"];
files = fullfile(outDir, "fig_scenario_" + tags + ".png");
for k = 1:numel(names)
    choose(ui.scenarioDropdown, char(names(k)));
    saveCanvas(ui, files(k));
end
end

function files = planners(root, outDir, opts)
ui = openReference(root, opts);
cleanup = onCleanup(@() closeUI(ui));
names = ["Dijkstra Corners", "Energy Efficient", "Grid A*"];
files = fullfile(outDir, ["fig_planner_dijkstra.png", "fig_planner_energy.png", "fig_planner_astar.png"]);
for k = 1:numel(names)
    choose(ui.plannerDropdown, names(k));
    saveCanvas(ui, files(k));
end
end

% ---- GUI actions ----
function ui = openReference(root, opts)
% Reference scenario over the Stevens Smart Lab map with Physical Agents under ED-RHC
ui = MainUI('Monitor', opts.Monitor);
choose(ui.agentTypeDropdown, 'Physical Agent');
ui.loadLayout(fullfile(root, 'Configurations', 'ReportLayout.mat'));
choose(ui.policyDropdown, 'ED-RHC');
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
