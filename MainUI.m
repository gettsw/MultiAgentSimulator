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
    end

    methods
        function obj = MainUI()
            obj.buildUI();
            obj.model = ScenarioModel();
            obj.renderer = ScenarioRenderer(obj.ax);
            obj.controller = GraphEditController(obj.model, obj.renderer, @(m)obj.setStatus(m));
            obj.objectiveWin = ObjectivePlotWindow();
            obj.clock = SimulationClock(obj.endTimeField.Value, obj.dtField.Value);
            obj.sim = SimulationController(obj.model, obj.renderer, obj.clock, ...
                @(t,tEnd)obj.setTime(t,tEnd), ...
                @(m)obj.setStatus(m), ...
                @(t,J)obj.objectiveWin.addPoint(t,J));
            obj.fig.WindowButtonMotionFcn = @(~,~)obj.safeOnMouseMove();
            obj.controller.setMode("idle");
            obj.renderer.renderAll(obj.model);
            obj.setTime(0, obj.clock.endTime);
        end
    end

    methods (Access=private)
        function buildUI(obj)
            obj.fig = uifigure('Name','Offline Path Planner UI', ...
                'Position', [100 100 1200 750]);
            obj.fig.CloseRequestFcn = @(~,~) obj.forceCloseAll();
            p = uipanel(obj.fig, 'Title','Tools', 'Position',[10 270 260 470]);
            tp = uipanel(obj.fig, 'Title','Time Control', 'Position',[10 10 260 250]);
            obj.ax = uiaxes(obj.fig, 'Position',[290 10 900 730]);
            try
                disableDefaultInteractivity(obj.ax);
            catch
            end
            grid(obj.ax,'on'); axis(obj.ax,'equal');
            xlim(obj.ax,[0 100]); ylim(obj.ax,[0 100]);
            uibutton(p,'Text','Add Target','Position',[20 400 220 35], ...
                'ButtonPushedFcn', @(~,~)obj.setMode("addTarget"));
            uibutton(p,'Text','Add Agent','Position',[20 360 220 35], ...
                'ButtonPushedFcn', @(~,~)obj.setMode("addAgent"));
            uibutton(p,'Text','Add Edge','Position',[20 320 220 35], ...
                'ButtonPushedFcn', @(~,~)obj.setMode("addEdge"));
            uibutton(p,'Text','Add Wall','Position', [20 280 220 35], ...
                'ButtonPushedFcn', @(~,~)obj.setMode("addWall"));
            uibutton(p,'Text','Idle','Position',[20 240 220 30], ...
                'ButtonPushedFcn', @(~,~)obj.setMode("idle"));
            uibutton(p,'Text','Import Map...','Position',[20 210 220 20], ...
                'ButtonPushedFcn', @(~,~)obj.controller.importBackground());
            uibutton(p,'Text','Clear All','Position',[20 180 220 25], ...
                'ButtonPushedFcn', @(~,~)obj.onClearAll());
            uibutton(p,'Text','Save Layout...','Position',[20 145 105 30], ...
                'ButtonPushedFcn', @(~,~)obj.onSaveLayout());
            uibutton(p,'Text','Load Layout...','Position',[135 145 105 30], ...
                'ButtonPushedFcn', @(~,~)obj.onLoadLayout());
            uilabel(p,'Text','Agent Accel:','Position',[20 110 90 22]);
            obj.speedField = uieditfield(p,'numeric','Value',15,'Limits',[0.01 Inf], ...
                'Position',[115 106 125 30]);
            obj.modeLabel = uilabel(p,'Text','Mode: idle', ...
                'Position',[20 75 220 26], 'FontWeight','bold');
            obj.statusLabel = uilabel(p,'Text','', ...
                'Position',[20 10 220 60], 'WordWrap','on');
            obj.ax.PickableParts = 'all';
            obj.ax.HitTest = 'on';
            obj.ax.ButtonDownFcn = @(ax,~)obj.onAxesClick(ax);
            obj.timeLabel = uilabel(tp,'Text','t = 0.00 / 60.00', ...
                'Position',[20 200 220 22], 'FontWeight','bold');
            uibutton(tp,'Text','Play','Position',[20 155 100 35], ...
                'ButtonPushedFcn', @(~,~)obj.sim.play());
            uibutton(tp,'Text','Pause','Position',[140 155 100 35], ...
                'ButtonPushedFcn', @(~,~)obj.sim.pause());
            uibutton(tp,'Text','Run to End','Position',[20 115 220 35], ...
                'ButtonPushedFcn', @(~,~)obj.sim.runToEnd());
            uibutton(tp,'Text','Reset Time','Position',[20 80 220 30], ...
                'ButtonPushedFcn', @(~,~)obj.onResetSimulation());
            uilabel(tp,'Text','End time:','Position',[20 52 60 22]);
            obj.endTimeField = uieditfield(tp,'numeric','Value',60,'Limits',[0 Inf], ...
                'Position',[85 48 70 28], ...
                'ValueChangedFcn', @(s,~)obj.sim.setEndTime(s.Value));
            uilabel(tp,'Text','dt:','Position',[165 52 20 22]);
            obj.dtField = uieditfield(tp,'numeric','Value',0.05,'Limits',[1e-4 Inf], ...
                'Position',[190 48 50 28], ...
                'ValueChangedFcn', @(s,~)obj.sim.setDt(s.Value));
            uilabel(tp,'Text','Speed:','Position',[20 20 50 22]);
            obj.timeScaleSlider = uislider(tp, ...
                'Limits',[0 10], 'Value',1, ...
                'Position',[75 30 160 3], ...
                'ValueChangedFcn', @(s,~)obj.sim.setTimeScale(s.Value));
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
            obj.sim.reset();
        end

        function onResetSimulation(obj)
            if ~isempty(obj.objectiveWin), obj.objectiveWin.reset(); end
            obj.sim.reset();
        end

        function onClearAll(obj)
            if ~isempty(obj.sim), obj.sim.pause(); end
            obj.controller.clearAll();
            obj.clock.reset();
            obj.setTime(0, obj.clock.endTime);
        end

        function setMode(obj, m)
            obj.modeLabel.Text = "Mode: " + string(m);
            obj.controller.setMode(m);
        end

        function setStatus(obj, msg)
            obj.statusLabel.Text = msg;
        end

        function setTime(obj, t, tEnd)
            if isempty(obj.timeLabel) || ~isvalid(obj.timeLabel), return; end
            obj.timeLabel.Text = sprintf('t = %.2f / %.2f', t, tEnd);
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
    end
end