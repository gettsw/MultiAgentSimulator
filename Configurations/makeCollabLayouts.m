function files = makeCollabLayouts()
% Builds the layout files of the collaboration paper's experiments in Configurations/Collab/,
% listed in scenarioCatalog ("Collab: ..." rows) so the GUI loads the same setups as runTrial:
%   Collab_2Agent_5T_Group.mat       JS: 2 agents, 5 targets, with A_3 = 12 > B = 10 (target 3
%                                    needs two agents)
%   Collab_3Agent_General_Group.mat  JS: 3 agents, general, with A_3 = A_8 = 12
%   Collab_General_N2.mat ... N6     JS: 3 agents, general, with N = 2..6 agents spread evenly
%                                    over the targets (runTrial's "auto" rule)
% Every target has A = 1, B = 10 and R(0) = 0.5 unless stated, as in the JS presets. Each file is
% the JS preset loaded with ScenarioModel.importLayout, changed, and saved with exportLayout, so
% it has exactly the fields the GUI writes. Run it again after editing:
%   matlab -batch "addpath(genpath(pwd)); makeCollabLayouts"
here = fileparts(mfilename('fullpath'));
outDir = fullfile(here, 'Collab');
if ~isfolder(outDir), mkdir(outDir); end
files = strings(0, 1);
files(end+1) = make1("JS_2Agent_5T", "Collab_2Agent_5T_Group", 3, []);
files(end+1) = make1("JS_3Agent_General", "Collab_3Agent_General_Group", [3 8], []);
for n = 2:6
    files(end+1) = make1("JS_3Agent_General", "Collab_General_N" + n, [], n); %#ok<AGROW>
end
fprintf('%d layouts in %s (changed ones rewritten)\n', numel(files), outDir);

    function file = make1(preset, name, groupTargets, nAgents)
        tmp = load(fullfile(here, 'Legacy', preset + ".mat"), 's');
        src = tmp.s;
        nT = size(src.targets, 1);
        if ~isempty(nAgents)
            src.agentStarts = mod(round((0:nAgents - 1) * nT / nAgents), nT) + 1;
            src.agentSpeed = src.agentSpeed(1) * ones(1, nAgents);
        end
        model = ScenarioModel(DijkstraCornerPlanner());
        model.importLayout(src);
        for k = 1:nT
            model.targets(k).setRates(1, 10, 0.5);
        end
        for k = groupTargets
            model.targets(k).setRates(12, [], []);
        end
        model.setUncertaintyMode('Static Uniform');
        s = model.exportLayout();
        file = fullfile(outDir, name + ".mat");
        if isfile(file)
            old = load(file, 's');
            if isequaln(old.s, s), return; end   % unchanged: keep the file (MAT headers hold a date)
        end
        save(file, 's');
    end
end
