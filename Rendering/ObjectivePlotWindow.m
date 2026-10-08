classdef ObjectivePlotWindow < handle
    properties
        fig
        ax
        btnReset
        currentLine
        tData = []
        jData = []
        allLines = gobjects(0)
        runLabels = {}
        runCount = 0
        maxPoints = 5000
        lastXMax = 10
        lastYMax = 10
        lastT = -Inf
        newestLineWidth = 2.8
        oldestLineWidth = 1.6
        minVisibility = 0.45
    end
    
    methods
        function obj = ObjectivePlotWindow()
            obj.fig = uifigure('Name','Objective Function J(t)', ...
                'Position',[1350 200 700 450]);
            obj.ax = uiaxes(obj.fig, 'Position',[60 70 610 340]);
            grid(obj.ax,'on');
            title(obj.ax,'Time-Averaged Objective');
            xlabel(obj.ax,'Time (s)');
            ylabel(obj.ax,'J(t) = (1/t) \int_0^t J(\tau) d\tau');
            xlim(obj.ax, [0 obj.lastXMax]);
            ylim(obj.ax, [0 obj.lastYMax]);
            
            obj.btnReset = uibutton(obj.fig, 'push', ...
                'Text','Reset Plot', ...
                'Position',[60 20 120 30], ...
                'ButtonPushedFcn', @(~,~)obj.clearAllTrials());
        end
        
        function reset(obj)
            % Prepares state for a clean reset/rewind without wiping existing run lines
            obj.tData = [];
            obj.jData = [];
            obj.lastT = -Inf;
        end
        
        function clearAllTrials(obj)
            % Completely clears all overlay trials and resets line history
            if ~isempty(obj.allLines)
                delete(obj.allLines(isgraphics(obj.allLines)));
            end
            obj.allLines = gobjects(0);
            obj.runLabels = {};
            obj.runCount = 0;
            obj.tData = [];
            obj.jData = [];
            obj.lastT = -Inf;
            obj.lastXMax = 10;
            obj.lastYMax = 10;
            xlim(obj.ax, [0 obj.lastXMax]);
            ylim(obj.ax, [0 obj.lastYMax]);
            legend(obj.ax, 'off');
        end
        
        function startNewRunLine(obj, policyName)
            obj.runCount = obj.runCount + 1;
            baseColor = 0.2 + 0.6*rand(1,3);
            
            hold(obj.ax, 'on');
            obj.currentLine = plot(obj.ax, NaN, NaN, ...
                'LineWidth', obj.newestLineWidth, ...
                'Color', baseColor);
            hold(obj.ax, 'off');
            
            obj.allLines(end+1) = obj.currentLine;
            
            if nargin >= 2 && ~isempty(policyName)
                obj.runLabels{end+1} = sprintf('Run %d (%s)', obj.runCount, policyName);
            else
                obj.runLabels{end+1} = sprintf('Run %d', obj.runCount);
            end
            
            obj.fadeOlderRuns();
            obj.updateLegend();
        end
        
        function addPoint(obj, t, J)
            if isempty(obj.fig) || ~isvalid(obj.fig) || isempty(obj.ax) || ~isvalid(obj.ax)
                return;
            end
            
            % Lazy initialization: start first line automatically if none exists yet
            if isempty(obj.currentLine) || ~isgraphics(obj.currentLine)
                obj.startNewRunLine();
            end
            
            % Automatic detection of new run/reset based on backwards time jump
            if ~isempty(obj.tData) && isfinite(obj.lastT) && t < obj.lastT
                obj.tData = [];
                obj.jData = [];
                obj.lastT = -Inf;
                obj.startNewRunLine();
            end
            
            obj.tData(end+1) = t;
            obj.jData(end+1) = J;
            obj.lastT = t;
            
            % Trim memory for active line trace
            if numel(obj.tData) > obj.maxPoints
                obj.tData = obj.tData(end-obj.maxPoints+1:end);
                obj.jData = obj.jData(end-obj.maxPoints+1:end);
            end
            
            if isgraphics(obj.currentLine)
                obj.currentLine.XData = obj.tData;
                obj.currentLine.YData = obj.jData;
            end
            
            % Dynamic Axis Auto-scaling
            if t > obj.lastXMax * 0.9
                obj.lastXMax = max(10, t * 1.1);
                xlim(obj.ax, [0 obj.lastXMax]);
            end
            if ~isempty(obj.jData)
                jMax = max(obj.jData);
                if jMax <= 0, jMax = 1; end
                if jMax > obj.lastYMax * 0.9
                    obj.lastYMax = max(10, jMax * 1.1);
                    ylim(obj.ax, [0 obj.lastYMax]);
                end
            end
            drawnow limitrate;
        end
    end
    
    methods (Access=private)
        function updateLegend(obj)
            valid = isgraphics(obj.allLines);
            lines = obj.allLines(valid);
            labels = obj.runLabels(valid);
            if isempty(lines)
                legend(obj.ax, 'off');
                return;
            end
            % Newest runs listed first
            lines = flipud(lines(:));
            labels = fliplr(labels);
            legend(obj.ax, lines, labels, ...
                'Location','northeastoutside', ...
                'Interpreter','none');
        end
        
        function fadeOlderRuns(obj)
            valid = isgraphics(obj.allLines);
            lines = obj.allLines(valid);
            if isempty(lines), return; end
            m = numel(lines);
            if m == 1
                obj.applyVisibility(lines(1), 1.0, obj.newestLineWidth);
                return;
            end
            for i = 1:m
                x = (i-1)/(m-1);
                x2 = sqrt(x);  % Gentler non-linear decay for older trials
                w = obj.minVisibility + (1 - obj.minVisibility) * x2;
                lw = obj.oldestLineWidth + (obj.newestLineWidth - obj.oldestLineWidth) * x2;
                obj.applyVisibility(lines(i), w, lw);
            end
        end
        
        function applyVisibility(~, hLine, w, lineWidth)
            if ~isgraphics(hLine), return; end
            try
                hLine.LineWidth = lineWidth;
            catch
            end
            try
                c = hLine.Color;
                if numel(c) >= 3
                    c3 = c(1:3);
                    cFaded = (1-w)*[1 1 1] + w*c3;
                    hLine.Color = cFaded;
                end
            catch
            end
            try
                c = hLine.Color;
                if numel(c) == 3
                    hLine.Color = [c(1:3) w]; 
                end
            catch
            end
        end
    end
end