%% A new monitoring problem in seven steps: event detection in a museum
% Security robots patrol a small museum. Incidents (events) occur at random at the targets and
% last a short while; a robot at a target detects the incidents in progress there. Four small
% classes in this folder define the problem, each a subclass of a simulator base class:
%   SimpleTarget < Target          events occur at random and expire (the target dynamics)
%   SimpleAgent < Agent           moves at its top speed along its path (the agent model)
%   SimplePolicy < Policy    goes to the neighbor with the most events (the policy)
%   SimpleMetric < Metric         share of the events missed (the performance metric)
% One line each in Components/ComponentRegistry.m lists them in the GUI's dropdowns and in
% runTrial. Run this script section by section (Ctrl+Enter) and watch the GUI: each section
% does what the clicks named in its title do. Every step saves the canvas to
% results/example/, so you can compare with what you see.

%% 1. Open the simulator and import the floor plan (button "Import Map...")
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(genpath(root));
outDir = fullfile(root, 'results', 'example');
if ~isfolder(outDir), mkdir(outDir); end
ui = MainUI();
ui.renderer.loadBackgroundImage(fullfile(root, 'Configurations', 'backgrounds', 'Museum.png'));
ui.model.background = "Museum.png";
snapshot(ui, outDir, "1_map");

%% 2. Draw the walls ("Add Wall", then click both ends of each wall)
% Clicks snap to a 5-unit grid; the museum's walls lie on it. Draw the outer walls too: without
% them, the shortest way around an inner wall that ends at the outer wall passes outside.
walls = [40 10 40 30; 10 40 30 40; 40 90 40 70; 10 60 30 60      % inner walls
         60 10 60 30; 70 40 90 40; 60 90 60 70; 70 60 90 60
         10 10 45 10; 55 10 90 10; 90 10 90 90; 90 90 10 90; 10 90 10 10];   % outer walls
press(ui, 'Add Wall');
for w = walls'
    click(ui, w(1:2)');
    click(ui, w(3:4)');
end

%% 3. Mark the targets ("Add Target", then click each spot)
targets = [20 20; 20 80; 80 20; 80 80; 50 15; 15 50; 50 85; 85 50; 50 50];
press(ui, 'Add Target');
for p = targets'
    click(ui, p');
end
snapshot(ui, outDir, "3_targets");

%% 4. Connect the targets ("Add Edge", then click two targets)
% The path planner (dropdown "Path Planner") routes each edge around the walls
edges = [9 5; 9 6; 9 7; 9 8; 1 9; 2 9; 3 9; 4 9; 1 5; 2 6; 4 7; 3 8];
press(ui, 'Add Edge');
for e = edges'
    click(ui, targets(e(1), :));
    click(ui, targets(e(2), :));
end
snapshot(ui, outDir, "4_edges");

%% 5. Choose the new classes and place three robots (dropdowns; "Add Agent", click targets)
choose(ui.targetTypeDropdown, 'Simple Target (example)');
choose(ui.agentTypeDropdown, 'Simple Agent (example)');
choose(ui.policyDropdown, 'Simple Policy (example)');
choose(ui.objectiveDropdown, 'Simple Metric (example)');
enter(ui.speedField, 4);                          % slow indoor robots
press(ui, 'Add Agent');
for k = [9 1 4]
    click(ui, targets(k, :));
end
press(ui, 'Idle');

%% 6. Run ten minutes ("End time" 600, "Run to End")
% The objective window plots the share of events missed so far
rng(1);
enter(ui.endTimeField, 600);
press(ui, 'Run to End');
snapshot(ui, outDir, "6_run");
exportapp(ui.fig, fullfile(outDir, "6_gui.png"));
exportapp(ui.objectiveWin.fig, fullfile(outDir, "6_objective.png"));
fprintf('Events missed: %.1f %%\n', 100 * ui.model.metrics{1}.value(ui.model, 600));

%% 7. Save the scenario ("Save Layout...") and compare policies without the GUI (runTrial)
file = fullfile(outDir, 'MyMuseum.mat');
ui.saveLayout(file);
policies = ["Simple Policy (example)", "Random Walk"];
missed = zeros(numel(policies), 10);
for i = 1:numel(policies)
    for seed = 1:10
        out = runTrial(struct('Layout', file, 'Policy', policies(i), 'AgentStarts', "layout", ...
            'AgentType', "Simple Agent (example)", 'AgentSpeed', 4, 'EndTime', 600, 'Seed', seed, ...
            'Metrics', ["Simple Metric (example)", "Average Uncertainty J(t)"]));
        missed(i, seed) = out.metricValues.SimpleMetric;
    end
    fprintf('%-24s misses %4.1f %% of the events (mean of 10 runs; std %.1f)\n', ...
        policies(i), 100 * mean(missed(i, :)), 100 * std(missed(i, :)));
end

%% Helpers: the GUI actions above
function press(ui, text)
% Click the button labeled text
b = findall(ui.fig, 'Type', 'uibutton', 'Text', text);
b.ButtonPushedFcn(b, []);
end

function click(ui, pos)
% Click the canvas at pos
ui.controller.onCanvasClick(pos, ui.speedField.Value);
end

function choose(dropdown, value)
% Pick value in a dropdown
dropdown.Value = value;
dropdown.ValueChangedFcn(dropdown, []);
end

function enter(field, value)
% Type value into a numeric field
field.Value = value;
if ~isempty(field.ValueChangedFcn), field.ValueChangedFcn(field, []); end
end

function snapshot(ui, outDir, name)
% Save the canvas as you see it
drawnow;
exportgraphics(ui.ax, fullfile(outDir, name + ".png"), 'Resolution', 150);
end
