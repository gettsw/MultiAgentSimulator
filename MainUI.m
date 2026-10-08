classdef MainUI < handle
    properties
        fig
        ax
        speedField
        modeLabel
        statusLabel
        timeLabel
        endTimeField
        dtField
        timeScaleSlider
        model
        renderer
        controller
        clock
        sim
        objectiveWin
        gridStep = 5;
        snapTargets = true;
        frameBuffer = {}       % Accumulated frames for video export
        agentTypeDropdown
        policyDropdown
        uncertaintyDropdown % New Dropdown Property
        recordCheckbox      % Frame capture is slow; only record when asked
    end
    
    methods
        function obj = MainUI()
            obj.buildUI();
            
            % Instantiate core framework components
            obj.model = ScenarioModel();
            obj.renderer = ScenarioRenderer(obj.ax);
            obj.controller = GraphEditController(obj.model, obj.renderer, @(m)obj.setStatus(m));
            obj.objectiveWin = ObjectivePlotWindow();
            obj.clock = SimulationClock(obj.endTimeField.Value, obj.dtField.Value);
            
            obj.sim = SimulationController(obj.model, obj.renderer, obj.clock, ...
                @(t,tEnd)obj.setTime(t,tEnd), ...
                @(m)obj.setStatus(m), ...
                @(t,J)obj.objectiveWin.addPoint(t,J));
            
            % Bind event listeners and initial UI state
            obj.fig.WindowButtonMotionFcn = @(~,~)obj.safeOnMouseMove();
            obj.controller.setMode("idle");
            
            % Initialize default selected policy and uncertainty states
            obj.onPolicyChanged(obj.policyDropdown.Value);
            obj.onUncertaintyModeChanged(obj.uncertaintyDropdown.Value);
            
            obj.renderer.renderAll(obj.model);
            obj.setTime(0, obj.clock.endTime);
        end
    end
    
    methods (Access = private)
        function buildUI(obj)
            obj.fig = uifigure('Name','Offline Path Planner UI', ...
                'Position', [100 100 1200 780]);
            obj.fig.CloseRequestFcn = @(~,~) obj.forceCloseAll();

            % Grid layouts size everything automatically; the left column scrolls if the window is short
            main = uigridlayout(obj.fig, [1 2], 'ColumnWidth', {280, '1x'});
            left = uigridlayout(main, [2 1], 'RowHeight', {'fit', 'fit'}, ...
                'Padding', [0 0 0 0], 'Scrollable', 'on');
            p  = uigridlayout(uipanel(left, 'Title','Tools'), [12 2], ...
                'RowHeight', [repmat({'fit'}, 1, 11) {36}], 'ColumnWidth', {'1x', '1x'}, ...
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
                'Items', {'Default', 'Linear'}, 'Value', 'Default'), 2, 2);
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

            place(uilabel(p,'Text','Agent Accel:'), 6, 1);
            obj.speedField = place(uieditfield(p,'numeric','Value',15,'Limits',[0.01 Inf]), 6, 2);

            % Active Policy Selector (full width so long policy names fit)
            place(uilabel(p,'Text','Active Policy:', 'FontWeight','bold'), 7, [1 2]);
            obj.policyDropdown = place(uidropdown(p, ...
                'Items', {'Random Walk', 'Classical RHC [1]', 'Divergent RHC', 'Joint RHC', 'Joint RHC N'}, ...
                'Value', 'Random Walk', ...
                'ValueChangedFcn', @(s,~)obj.onPolicyChanged(s.Value)), 8, [1 2]);

            % Dynamic vs. Static Uncertainty Selector
            place(uilabel(p,'Text','Uncertainty:', 'FontWeight','bold'), 9, [1 2]);
            obj.uncertaintyDropdown = place(uidropdown(p, ...
                'Items', {'Dynamic Uncertainty', 'Static Priority Target', 'Static Uniform'}, ...
                'Value', 'Dynamic Uncertainty', ...
                'ValueChangedFcn', @(s,~)obj.onUncertaintyModeChanged(s.Value)), 10, [1 2]);

            obj.modeLabel = place(uilabel(p,'Text','Mode: idle', 'FontWeight','bold'), 11, [1 2]);
            obj.statusLabel = place(uilabel(p,'Text','', 'WordWrap','on', ...
                'VerticalAlignment','top'), 12, [1 2]);

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
        
        function forceCloseAll(obj)
            if ~isempty(obj.sim), obj.sim.pause(); end
            delete(timerfindall); 
            if ~isempty(obj.objectiveWin) && isprop(obj.objectiveWin, 'fig') && isgraphics(obj.objectiveWin.fig)
                delete(obj.objectiveWin.fig);
            end
            delete(obj.fig);
        end
        
        function onSaveLayout(obj)
            s = obj.model.exportLayout();
            [f,d] = uiputfile('*.mat','Save layout');
            if isequal(f,0), return; end
            save(fullfile(d,f), 's');
        end
        
        function onLoadLayout(obj)
            [f,d] = uigetfile('*.mat','Load layout');
            if isequal(f,0), return; end
            tmp = load(fullfile(d,f), 's');
            obj.model.importLayout(tmp.s);
            obj.renderer.clearAxes();
            obj.renderer.renderAll(obj.model);
            obj.onResetSimulation();
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
            selectedType = obj.agentTypeDropdown.Value;
            obj.controller.onCanvasClick([cp(1,1) cp(1,2)], obj.speedField.Value, selectedType);
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
