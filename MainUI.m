classdef MainUI < handle
    properties
        fig
        ax
        speedField
        accelField          % max acceleration of agents added next
        profileDropdown     % speed profile of agents added next
        modeLabel
        statusLabel
        timeLabel
        endTimeField
        dtField
        timeScaleSlider
        plannerDropdown
        policyDropdown
        agentTypeDropdown
        registry
        model
        renderer
        controller
        clock
        sim
        objectiveWin
        gridStep = 5;
        snapTargets = true;
        frameBuffer = {}       % Accumulated frames for video export
        uncertaintyDropdown % New Dropdown Property
        recordCheckbox      % Frame capture is slow; only record when asked
    end
    
    methods
        function obj = MainUI(opts)
            % MainUI('Monitor', k) opens the windows on monitor k (a row of
            % get(groot, 'MonitorPositions')); by default they open on the primary monitor
            arguments
                opts.Monitor double = []
            end
            obj.registry = ComponentRegistry();
            [mainPos, objectivePos] = MainUI.windowPositions(opts.Monitor);
            obj.buildUI(mainPos);

            plannerFactory = obj.registry.factoryFor('pathplanner', obj.plannerDropdown.Value);
            agentFactory   = obj.registry.factoryFor('agenttype',   obj.agentTypeDropdown.Value);
            obj.model = ScenarioModel(plannerFactory(), agentFactory);
            obj.model.agentAccel = obj.accelField.Value;
            obj.model.agentProfile = string(obj.profileDropdown.Value);
            obj.renderer = ScenarioRenderer(obj.ax);
            obj.controller = GraphEditController(obj.model, obj.renderer, @(m)obj.setStatus(m));
            obj.objectiveWin = ObjectivePlotWindow(objectivePos);
            obj.clock = SimulationClock(obj.endTimeField.Value, obj.dtField.Value);
            
            obj.sim = SimulationController(obj.model, obj.renderer, obj.clock, ...
                @(t,tEnd)obj.setTime(t,tEnd), ...
                @(m)obj.setStatus(m), ...
                @(t,J)obj.objectiveWin.addPoint(t,J), ...
                @(running)obj.onRunStateChanged(running));

            % Bind event listeners and initial UI state
            obj.fig.WindowButtonMotionFcn = @(~,~)obj.safeOnMouseMove();
            obj.controller.setMode("idle");
            
            % Initialize default selected policy and uncertainty states
            obj.onPolicyChanged(obj.policyDropdown.Value);
            obj.onUncertaintyModeChanged(obj.uncertaintyDropdown.Value);
            
            obj.renderer.renderAll(obj.model);
            obj.setTime(0, obj.clock.endTime);
        end

        % File I/O behind the Save/Load Layout buttons, callable without the file dialogs
        function saveLayout(obj, file)
            s = obj.model.exportLayout();
            save(file, 's');
        end

        function loadLayout(obj, file)
            tmp = load(file, 's');
            obj.model.importLayout(tmp.s);
            % The layout may carry its uncertainty mode and agents; show them in the controls
            obj.uncertaintyDropdown.Value = obj.model.uncertaintyMode;
            if ~isempty(obj.model.agents)
                a = obj.model.agents(1);
                obj.speedField.Value = a.state.maxSpeed;
                obj.onAgentAccelChanged(a.state.maxAccel);
                obj.onAgentProfileChanged(a.profile);
            end
            obj.renderer.clearAxes();
            obj.renderer.renderAll(obj.model);
            obj.onResetSimulation();
        end
    end

    methods (Static)
        function k = secondaryMonitor()
            % A monitor other than the primary one (whose origin is [1 1]), for windows opened by
            % scripts and tests; 1 when there is only one monitor
            mons = get(groot, 'MonitorPositions');
            k = find(mons(:, 1) ~= 1 | mons(:, 2) ~= 1, 1);
            if isempty(k), k = 1; end
        end
    end

    methods (Static, Access = private)
        function [mainPos, objPos] = windowPositions(monitor)
            % Default spots on the primary monitor, or fitted onto the given monitor: main window
            % at its top left, objective window to the right or, if there is no room, below
            mainPos = [100 100 1200 780];
            objPos = [1350 200 700 450];
            if isempty(monitor), return; end
            mons = get(groot, 'MonitorPositions');
            m = mons(min(monitor, size(mons, 1)), :);   % [left bottom width height]
            gap = 20;
            top = m(2) + m(4) - 1 - 60;                 % clear of the menu bar and title bar
            w = min(1200, m(3) - 2*gap);
            h = min(780, m(4) - 120);
            mainPos = [m(1) + gap, top - h, w, h];
            ow = min(700, m(3) - 2*gap);
            if mainPos(1) + w + gap + ow <= m(1) + m(3) - gap
                objPos = [mainPos(1) + w + gap, top - 450, ow, 450];
            else
                objPos = [m(1) + gap, mainPos(2) - gap - 30 - 450, ow, 450];   % 30: title bar
            end
        end
    end

    methods (Access = private)
        function buildUI(obj, position)
            obj.fig = uifigure('Name','Multi-Agent Persistent Monitoring Simulator', ...
                'Position', position);
            obj.fig.CloseRequestFcn = @(~,~) obj.forceCloseAll();

            % Grid layouts size everything automatically; the left column scrolls if the window is short
            main = uigridlayout(obj.fig, [1 2], 'ColumnWidth', {280, '1x'});
            left = uigridlayout(main, [2 1], 'RowHeight', {'fit', 'fit'}, ...
                'Padding', [0 0 0 0], 'Scrollable', 'on');
            p  = uigridlayout(uipanel(left, 'Title','Tools'), [15 2], ...
                'RowHeight', [repmat({'fit'}, 1, 14) {36}], 'ColumnWidth', {'1x', '1x'}, ...
                'RowSpacing', 5, 'Padding', [8 6 8 6]);
            tp = uigridlayout(uipanel(left, 'Title','Time Control & Analysis'), [7 4], ...
                'RowHeight', repmat({'fit'}, 1, 7), 'ColumnWidth', {'fit', '1x', 'fit', '1x'}, ...
                'RowSpacing', 5, 'Padding', [8 6 8 6]);
            obj.ax = uiaxes(main);

            try
                disableDefaultInteractivity(obj.ax);
            catch
            end

            grid(obj.ax,'on'); axis(obj.ax,'equal');
            xlim(obj.ax,[0 100]); ylim(obj.ax,[0 100]);

            % ---- Tools panel ----
            place(uibutton(p,'Text','Add Target', ...
                'ButtonPushedFcn', @(~,~)obj.setMode("addTarget")), 1, 1);
            place(uibutton(p,'Text','Add Agent', ...
                'ButtonPushedFcn', @(~,~)obj.setMode("addAgent")), 2, 1);
            obj.agentTypeDropdown = place(uidropdown(p, ...
                'Items', obj.registry.keysFor('agenttype'), 'Value', 'Abstract Agent', ...
                'ValueChangedFcn', @(s,~)obj.onAgentTypeChanged(s.Value)), 2, 2);
            place(uibutton(p,'Text','Add Edge', ...
                'ButtonPushedFcn', @(~,~)obj.setMode("addEdge")), 1, 2);
            place(uibutton(p,'Text','Add Wall', ...
                'ButtonPushedFcn', @(~,~)obj.setMode("addWall")), 3, 1);
            place(uibutton(p,'Text','Idle', ...
                'ButtonPushedFcn', @(~,~)obj.setMode("idle")), 3, 2);
            place(uibutton(p,'Text','Import Map...', ...
                'ButtonPushedFcn', @(~,~)obj.controller.importBackground()), 4, 1);
            place(uibutton(p,'Text','Clear All', ...
                'ButtonPushedFcn', @(~,~)obj.onClearAll()), 4, 2);
            place(uibutton(p,'Text','Save Layout...', ...
                'ButtonPushedFcn', @(~,~)obj.onSaveLayout()), 5, 1);
            place(uibutton(p,'Text','Load Layout...', ...
                'ButtonPushedFcn', @(~,~)obj.onLoadLayout()), 5, 2);

            % Settings of agents added next (agent type above, speed, acceleration, profile)
            place(uilabel(p,'Text','Agent Speed:'), 6, 1);
            obj.speedField = place(uieditfield(p,'numeric','Value',15,'Limits',[0.01 Inf]), 6, 2);
            place(uilabel(p,'Text','Agent Accel:'), 7, 1);
            obj.accelField = place(uieditfield(p,'numeric','Value',15,'Limits',[0.01 Inf], ...
                'ValueChangedFcn', @(s,~)obj.onAgentAccelChanged(s.Value)), 7, 2);
            place(uilabel(p,'Text','Speed Profile:'), 8, 1);
            obj.profileDropdown = place(uidropdown(p, ...
                'Items', {'Constant', 'SO (smooth)', 'FO (trapezoid)'}, 'ItemsData', cellstr(SpeedProfile.names), ...
                'Value', 'constant', ...
                'ValueChangedFcn', @(s,~)obj.onAgentProfileChanged(s.Value)), 8, 2);

            place(uilabel(p,'Text','Path Planner:'), 9, 1);
            obj.plannerDropdown = place(uidropdown(p, ...
                'Items', obj.registry.keysFor('pathplanner'), 'Value', 'Dijkstra Corners', ...
                'ValueChangedFcn', @(s,~)obj.onPlannerChanged(s.Value)), 9, 2);

            % Active Policy Selector (full width so long policy names fit)
            place(uilabel(p,'Text','Active Policy:', 'FontWeight','bold'), 10, [1 2]);
            obj.policyDropdown = place(uidropdown(p, ...
                'Items', obj.registry.keysFor('policy'), ...
                'Value', 'Random Walk', ...
                'ValueChangedFcn', @(s,~)obj.onPolicyChanged(s.Value)), 11, [1 2]);

            % Uncertainty mode: constant A_i by default (as in the paper); spikes are opt-in
            place(uilabel(p,'Text','Uncertainty:', 'FontWeight','bold'), 12, [1 2]);
            obj.uncertaintyDropdown = place(uidropdown(p, ...
                'Items', ScenarioModel.UncertaintyModes, ...
                'Value', 'Static Uniform', ...
                'ValueChangedFcn', @(s,~)obj.onUncertaintyModeChanged(s.Value)), 13, [1 2]);

            obj.modeLabel = place(uilabel(p,'Text','Mode: idle', 'FontWeight','bold'), 14, [1 2]);
            obj.statusLabel = place(uilabel(p,'Text','', 'WordWrap','on', ...
                'VerticalAlignment','top'), 15, [1 2]);

            obj.ax.PickableParts = 'all';
            obj.ax.HitTest = 'on';
            obj.ax.ButtonDownFcn = @(ax,~)obj.onAxesClick(ax);

            % ---- Time Control panel ----
            obj.timeLabel = place(uilabel(tp,'Text','t = 0.00 / 60.00', 'FontWeight','bold'), 1, [1 4]);

            place(uibutton(tp,'Text','Play', ...
                'ButtonPushedFcn', @(~,~)obj.onPlaySimulation()), 2, [1 2]);
            place(uibutton(tp,'Text','Pause', ...
                'ButtonPushedFcn', @(~,~)obj.sim.pause()), 2, [3 4]);
            place(uibutton(tp,'Text','Run to End', ...
                'ButtonPushedFcn', @(~,~)obj.onRunToEndSimulation()), 3, [1 4]);
            place(uibutton(tp,'Text','Reset Time', ...
                'ButtonPushedFcn', @(~,~)obj.onResetSimulation()), 4, [1 2]);
            place(uibutton(tp,'Text','Clear Plots', ...
                'ButtonPushedFcn', @(~,~)obj.onClearPlots()), 4, [3 4]);

            place(uilabel(tp,'Text','End time:'), 5, 1);
            obj.endTimeField = place(uieditfield(tp,'numeric','Value',60,'Limits',[0 Inf], ...
                'ValueChangedFcn', @(s,~)obj.sim.setEndTime(s.Value)), 5, 2);
            place(uilabel(tp,'Text','dt:'), 5, 3);
            obj.dtField = place(uieditfield(tp,'numeric','Value',0.05,'Limits',[1e-4 Inf], ...
                'ValueChangedFcn', @(s,~)obj.sim.setDt(s.Value)), 5, 4);

            place(uilabel(tp,'Text','Speed:'), 6, 1);
            obj.timeScaleSlider = place(uislider(tp, 'Limits',[0 10], 'Value',1, ...
                'ValueChangedFcn', @(s,~)obj.sim.setTimeScale(s.Value)), 6, [2 4]);

            obj.recordCheckbox = place(uicheckbox(tp,'Text','Record', 'Value', false), 7, [1 2]);
            place(uibutton(tp,'Text','Save Video...', ...
                'ButtonPushedFcn', @(~,~)obj.onSaveVideo()), 7, [3 4]);
        end

        function runLabel = getCombinedRunLabel(obj)
            % Formats a clear plot legend string combining Policy & Uncertainty mode
            pol = string(obj.policyDropdown.Value);
            unc = string(obj.uncertaintyDropdown.Value);
            switch unc
                case "Dynamic Uncertainty",    runLabel = pol + " (Dynamic)";
                case "Static Priority Target", runLabel = pol + " (Static Priority)";
                otherwise,                     runLabel = pol + " (Static Uniform)";
            end
        end

        function onPlaySimulation(obj)
            % Tag a new run label with the policy and uncertainty mode if starting from t = 0
            % Restarting a finished run: reset properly so the new plot line gets its label
            if obj.clock.isFinished(), obj.onResetSimulation(); end
            if obj.clock.currentTime == 0 && ~isempty(obj.objectiveWin)
                obj.objectiveWin.startNewRunLine(obj.getCombinedRunLabel());
            end
            if obj.clock.currentTime == 0
                obj.frameBuffer = {};  % Fresh buffer for each new run
            end
            obj.sim.play();
        end
        
        function onRunToEndSimulation(obj)
            % Restarting a finished run: reset properly so the new plot line gets its label
            if obj.clock.isFinished(), obj.onResetSimulation(); end
            if obj.clock.currentTime == 0 && ~isempty(obj.objectiveWin)
                obj.objectiveWin.startNewRunLine(obj.getCombinedRunLabel());
            end
            if obj.clock.currentTime == 0
                obj.frameBuffer = {};  % Fresh buffer for each new run
            end
            obj.sim.runToEnd();
        end
        
        function onPolicyChanged(obj, newPolicy)
            % Sync active policy across model and simulation controller
            if ~isempty(obj.model)
                if isprop(obj.model, 'setActivePolicy') || ismethod(obj.model, 'setActivePolicy')
                    obj.model.setActivePolicy(newPolicy);
                end
            end
            if ~isempty(obj.sim)
                if isprop(obj.sim, 'setActivePolicy') || ismethod(obj.sim, 'setActivePolicy')
                    obj.sim.setActivePolicy(newPolicy);
                end
            end
            obj.setStatus("Switched active policy to: " + string(newPolicy));
        end

        function onUncertaintyModeChanged(obj, newMode)
            % Sync active uncertainty mode across model and simulation controller
            if ~isempty(obj.model)
                if isprop(obj.model, 'setUncertaintyMode') || ismethod(obj.model, 'setUncertaintyMode')
                    obj.model.setUncertaintyMode(newMode);
                end
            end
            if ~isempty(obj.sim)
                if isprop(obj.sim, 'setUncertaintyMode') || ismethod(obj.sim, 'setUncertaintyMode')
                    obj.sim.setUncertaintyMode(newMode);
                end
            end
            obj.setStatus("Switched uncertainty mode to: " + string(newMode));
        end
        
        function onResetSimulation(obj)
            % 1. Pause active timers
            if ~isempty(obj.sim), obj.sim.pause(); end
            
            % 2. Enforce active policy and uncertainty states before resetting
            currentPolicy = obj.policyDropdown.Value;
            currentUncertainty = obj.uncertaintyDropdown.Value;
            
            obj.onPolicyChanged(currentPolicy);
            obj.onUncertaintyModeChanged(currentUncertainty);
            
            % 3. Reset internal buffers while keeping historical trial lines
            if ~isempty(obj.objectiveWin), obj.objectiveWin.reset(); end
            if ~isempty(obj.sim), obj.sim.reset(); end
            
            % 4. Redraw axes and reset time readout
            obj.renderer.renderAll(obj.model);
            obj.setTime(0, obj.clock.endTime);
            obj.setStatus("Reset sim | Policy: " + string(currentPolicy) + " | Mode: " + string(currentUncertainty));
        end
        
        function onClearPlots(obj)
            if ~isempty(obj.objectiveWin)
                obj.objectiveWin.clearAllTrials();
                obj.setStatus("Cleared all trial comparison plots.");
            end
        end
        
        function onPlannerChanged(obj, name)
            factory = obj.registry.factoryFor('pathplanner', name);
            obj.model.setPathPlanner(factory());
            obj.renderer.renderAll(obj.model);
            obj.setStatus(sprintf('Path planner: %s', name));
        end

        function onAgentTypeChanged(obj, name)
            factory = obj.registry.factoryFor('agenttype', name);
            obj.model.setAgentFactory(factory);
            obj.setStatus(sprintf('Agent type: %s (applies to new agents)', name));
        end

        function onAgentAccelChanged(obj, value)
            obj.model.agentAccel = value;
            obj.accelField.Value = value;
        end

        function onAgentProfileChanged(obj, name)
            obj.model.agentProfile = string(name);
            obj.profileDropdown.Value = char(name);
            obj.setStatus(sprintf('Speed profile: %s (applies to new agents)', name));
        end

        function onRunStateChanged(obj, running)
            % Lock component choices while a run is in progress
            state = 'on';
            if running, state = 'off'; end
            dds = {obj.plannerDropdown, obj.policyDropdown, obj.agentTypeDropdown, obj.uncertaintyDropdown, obj.profileDropdown};
            for k = 1:numel(dds)
                if ~isempty(dds{k}) && isvalid(dds{k}), dds{k}.Enable = state; end
            end
        end

        function forceCloseAll(obj)
            if ~isempty(obj.sim), obj.sim.pause(); end
            delete(timerfindall); 
            if ~isempty(obj.objectiveWin) && isprop(obj.objectiveWin, 'fig') && isgraphics(obj.objectiveWin.fig)
                delete(obj.objectiveWin.fig);
            end
            delete(obj.fig);
        end
        
        function onSaveLayout(obj)
            [f,d] = uiputfile('*.mat','Save layout');
            if isequal(f,0), return; end
            obj.saveLayout(fullfile(d,f));
        end

        function onLoadLayout(obj)
            [f,d] = uigetfile('*.mat','Load layout');
            if isequal(f,0), return; end
            obj.loadLayout(fullfile(d,f));
        end

        function onClearAll(obj)
            if ~isempty(obj.sim), obj.sim.pause(); end
            obj.controller.clearAll();
            obj.clock.reset();
            if ~isempty(obj.objectiveWin), obj.objectiveWin.clearAllTrials(); end
            obj.setTime(0, obj.clock.endTime);
            obj.setStatus("Cleared environment layout and trials.");
        end
        
        function setMode(obj, m)
            obj.modeLabel.Text = "Mode: " + string(m);
            obj.controller.setMode(m);
        end
        
        function setStatus(obj, msg)
            if isvalid(obj.statusLabel)
                obj.statusLabel.Text = msg;
            end
        end
        
        function setTime(obj, t, tEnd)
            if isempty(obj.timeLabel) || ~isvalid(obj.timeLabel), return; end
            obj.timeLabel.Text = sprintf('t = %.2f / %.2f', t, tEnd);
            if t > 0 && obj.recordCheckbox.Value
                try
                    drawnow limitrate;
                    obj.frameBuffer{end+1} = getframe(obj.fig);
                catch
                end
            end
        end
        
        function onAxesClick(obj, ax)
            cp = ax.CurrentPoint;
            obj.controller.onCanvasClick([cp(1,1) cp(1,2)], obj.speedField.Value);
        end
        
        function safeOnMouseMove(obj)
            if isempty(obj.ax) || ~isvalid(obj.ax), return; end
            cp = obj.ax.CurrentPoint;
            obj.controller.onCanvasMove([cp(1,1) cp(1,2)]);
        end

        function onSaveVideo(obj)
            if isempty(obj.frameBuffer)
                uialert(obj.fig, ...
                    'No frames recorded yet. Run the simulation first, then save.', ...
                    'No Recording');
                return;
            end
            [f, d] = uiputfile( ...
                {'*.mp4','MPEG-4 Video (*.mp4)'; '*.avi','AVI Video (*.avi)'}, ...
                'Save Simulation Video');
            if isequal(f, 0), return; end
            [~,~,ext] = fileparts(f);
            profile = 'MPEG-4';
            if strcmpi(ext, '.avi'), profile = 'Motion JPEG AVI'; end
            obj.setStatus('Saving video...');
            try
                vw = VideoWriter(fullfile(d, f), profile);
                vw.FrameRate = 20;
                open(vw);
                for i = 1:numel(obj.frameBuffer)
                    writeVideo(vw, obj.frameBuffer{i});
                end
                close(vw);
                obj.setStatus('Video saved: ' + string(f));
            catch err
                obj.setStatus('Video save failed: ' + string(err.message));
            end
        end
    end
end

function c = place(c, row, col)
% Assigns a component to a uigridlayout cell and returns it
c.Layout.Row = row;
c.Layout.Column = col;
end
