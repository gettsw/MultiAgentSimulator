classdef guiTest < matlab.uitest.TestCase
    % Drives the real MainUI with the App Testing Framework (needs a display; windows open
    % and close on screen while it runs). Run from the repo root:
    %   matlab -batch "addpath(genpath(pwd)); assertSuccess(runtests('guiTest'))"
    % Screenshots go to results/gui/ (git-ignored). File dialogs are bypassed by calling
    % MainUI.loadLayout/saveLayout and ScenarioRenderer.loadBackgroundImage with a path.

    properties
        ui
        root
        outDir
    end

    methods (TestMethodSetup)
        function openUI(tc)
            tc.root = fileparts(fileparts(mfilename('fullpath')));
            tc.outDir = fullfile(tc.root, 'results', 'gui');
            if ~isfolder(tc.outDir), mkdir(tc.outDir); end
            rng(0);
            tc.ui = MainUI();
            tc.addTeardown(@() closeUI(tc.ui));
        end
    end

    methods (Test)
        function showsPaperDefaults(tc)
            tc.verifyEqual(tc.ui.uncertaintyDropdown.Value, 'Static Uniform');
            tc.verifyEqual(tc.ui.model.uncertaintyMode, 'Static Uniform');
            tc.verifyNotEmpty(findall(tc.ui.fig, 'Type', 'uilabel', 'Text', 'Agent Speed:'));
        end

        function loadsReportLayoutAndRunsToEnd(tc)
            tc.ui.renderer.loadBackgroundImage(fullfile(tc.root, 'LabLayout.png'));
            tc.ui.loadLayout(fullfile(tc.root, 'Configurations', 'ReportLayout.mat'));
            m = tc.ui.model;
            tc.verifyEqual([m.agents.initialTargetIdx], [1 6 2 3 5]);
            tc.verifyTrue(all(arrayfun(@(a) isgraphics(a.graphicHandle), m.agents)), 'agents drawn');
            tc.verifyEqual(tc.ui.speedField.Value, 15);
            tc.verifyEqual(tc.ui.uncertaintyDropdown.Value, 'Static Uniform');
            snap(tc, 'report_layout_loaded');

            tc.choose(tc.ui.policyDropdown, 'Classical RHC [1]');
            tc.type(tc.ui.endTimeField, 60);
            tc.press(button(tc, 'Run to End'));
            tc.verifyEqual(tc.ui.clock.currentTime, 60, 'AbsTol', 1e-9);
            J = tc.ui.objectiveWin.currentLine.YData;
            tc.verifyTrue(any(isfinite(J)) && all(J(isfinite(J)) >= 0), 'objective plotted');
            tc.verifyEqual(char(tc.ui.policyDropdown.Enable), 'on', 'dropdowns unlocked after the run');
            snap(tc, 'report_layout_after_60s');
        end

        function playsPausesAndResets(tc)
            tc.ui.loadLayout(fullfile(tc.root, 'Configurations', 'ReportLayout.mat'));
            tc.press(button(tc, 'Play'));
            tc.verifyEqual(char(tc.ui.policyDropdown.Enable), 'off', 'dropdowns locked while running');
            pause(2);
            tc.press(button(tc, 'Pause'));
            t = tc.ui.clock.currentTime;
            tc.verifyGreaterThan(t, 0);
            pause(0.5);
            tc.verifyEqual(tc.ui.clock.currentTime, t, 'clock stopped after Pause');
            tc.press(button(tc, 'Reset Time'));
            tc.verifyEqual(tc.ui.clock.currentTime, 0);
            m = tc.ui.model;
            pos = cell2mat(arrayfun(@(a) a.state.pos, m.agents(:), 'UniformOutput', false));
            starts = reshape([m.targets([1 6 2 3 5]).position], 2, []).';
            tc.verifyEqual(pos, starts, 'agents back on their start targets');
        end

        function savesClearsAndReloads(tc)
            tc.choose(tc.ui.agentTypeDropdown, 'Energy Agent');
            tc.ui.loadLayout(fullfile(tc.root, 'Configurations', 'ReportLayout.mat'));
            tc.verifyClass(tc.ui.model.agents, 'EnergyAgent');   % loaded agents use the selected type
            saved = tc.ui.model.exportLayout();
            f = [tempname '.mat'];
            tc.addTeardown(@() delete(f));
            tc.ui.saveLayout(f);
            tc.press(button(tc, 'Clear All'));
            tc.verifyEmpty(tc.ui.model.targets);
            tc.verifyEmpty(tc.ui.model.agents);
            tc.ui.loadLayout(f);
            tc.verifyEqual(tc.ui.model.exportLayout(), saved);
        end

        function buildsScenarioByClicking(tc)
            % Click positions follow how a user works: on empty canvas, then on the target dots
            tc.press(button(tc, 'Add Target'));
            tc.press(tc.ui.ax, [20 80]);
            tc.press(tc.ui.ax, [70 40]);
            tc.verifyEqual(numel(tc.ui.model.targets), 2, 'targets added on empty canvas');

            tc.press(button(tc, 'Add Edge'));
            tc.press(tc.ui.ax, [20 80]);
            tc.press(tc.ui.ax, [70 40]);
            tc.verifyEqual(numel(tc.ui.model.edges), 1, 'edge added by clicking both target dots');

            tc.press(button(tc, 'Add Agent'));
            tc.press(tc.ui.ax, [20 80]);
            tc.verifyEqual(numel(tc.ui.model.agents), 1, 'agent added by clicking a target dot');
            tc.press(tc.ui.ax, [20 80]);
            tc.verifyEqual(numel(tc.ui.model.agents), 2, 'second agent added by clicking the agent on that target');

            tc.press(button(tc, 'Add Wall'));
            tc.press(tc.ui.ax, [45 45]);
            tc.press(tc.ui.ax, [45 75]);
            tc.verifyEqual(size(tc.ui.model.walls, 1), 1, 'wall added');
            if ~isempty(tc.ui.model.edges)
                tc.verifyGreaterThan(size(tc.ui.model.edges(1).curvePoints, 1), 2, 'edge replanned around the wall');
            end
            tc.press(tc.ui.ax, [45 60]);
            tc.press(tc.ui.ax, [60 60]);
            tc.verifyEqual(size(tc.ui.model.walls, 1), 2, 'wall started on top of an existing wall');
            snap(tc, 'built_by_clicking');
        end

        function addsTargetsOverBackgroundImage(tc)
            tc.ui.renderer.loadBackgroundImage(fullfile(tc.root, 'LabLayout.png'));
            tc.press(button(tc, 'Add Target'));
            tc.press(tc.ui.ax, [30 30]);
            tc.verifyEqual(numel(tc.ui.model.targets), 1, 'target added by clicking on the background image');
        end
    end

    methods (Access = private)
        function b = button(tc, text)
            b = findall(tc.ui.fig, 'Type', 'uibutton', 'Text', text);
            tc.assertNumElements(b, 1, "button " + text);
        end

        function snap(tc, name)
            drawnow;
            exportapp(tc.ui.fig, fullfile(tc.outDir, [name '.png']));
            exportapp(tc.ui.objectiveWin.fig, fullfile(tc.outDir, [name '_objective.png']));
        end
    end
end

function closeUI(ui)
% CloseRequestFcn stops the timer and closes both windows
if isvalid(ui.fig), close(ui.fig); end
end
