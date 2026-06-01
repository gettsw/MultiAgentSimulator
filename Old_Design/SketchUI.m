classdef SketchUI < handle
    properties
        Fig
        Ax
        BtnAddAgent
        BtnAddTarget
        BtnAddEdge
        BtnIdle
        BtnClear
        ModeLabel
        SpeedField
        EdgeInfoLabel

        Mode = "idle"      % "idle" | "addAgent" | "addTarget" | "addEdge"
        EdgePick = []      % selected target indices (1 or 2)

        agents = Agent.empty
        targets = Target.empty
        edges = Edge.empty

        % Target graphics (we draw targets ourselves for consistent appearance)
        targetScatterHandle = gobjects(1)
        targetTextHandles = gobjects(0)

        % Preview line when creating an edge
        EdgePreviewLine
    end

    methods
        function obj = SketchUI()
            obj.buildUI();
            obj.setMode("idle");
            obj.redrawTargets(); % initializes empty
        end
    end

    methods (Access=private)

        function buildUI(obj)
            obj.Fig = uifigure('Name','Sketch UI (Agents/Targets/Edges)', ...
                'Position',[100 100 1200 750]);

            p = uipanel(obj.Fig, 'Title', 'Tools', 'Position', [10 10 260 730]);

            obj.BtnAddTarget = uibutton(p,'push', ...
                'Text','Add Target', 'Position',[20 650 220 42], ...
                'ButtonPushedFcn', @(~,~)obj.setMode("addTarget"));

            obj.BtnAddAgent = uibutton(p,'push', ...
                'Text','Add Agent', 'Position',[20 600 220 42], ...
                'ButtonPushedFcn', @(~,~)obj.setMode("addAgent"));

            obj.BtnAddEdge = uibutton(p,'push', ...
                'Text','Add Edge (pick 2 targets)', 'Position',[20 550 220 42], ...
                'ButtonPushedFcn', @(~,~)obj.setMode("addEdge"));

            obj.BtnIdle = uibutton(p,'push', ...
                'Text','Idle / Select', 'Position',[20 500 220 36], ...
                'ButtonPushedFcn', @(~,~)obj.setMode("idle"));

            uilabel(p,'Text','Agent speed:', 'Position',[20 450 90 22]);
            obj.SpeedField = uieditfield(p,'numeric', ...
                'Value', 5, 'Limits',[0.01 Inf], ...
                'Position',[115 446 125 30]);

            obj.BtnClear = uibutton(p,'push', ...
                'Text','Clear All', 'Position',[20 395 220 40], ...
                'ButtonPushedFcn', @(~,~)obj.clearAll());

            obj.ModeLabel = uilabel(p, 'Text','Mode: idle', ...
                'Position',[20 350 220 28], 'FontWeight','bold');

            obj.EdgeInfoLabel = uilabel(p, 'Text','', ...
                'Position',[20 315 220 45], 'WordWrap','on');

            obj.Ax = uiaxes(obj.Fig, 'Position',[290 10 900 730]);
            title(obj.Ax, 'Click canvas to add items');
            xlabel(obj.Ax,'X'); ylabel(obj.Ax,'Y');
            grid(obj.Ax,'on'); axis(obj.Ax,'equal');
            xlim(obj.Ax,[0 100]); ylim(obj.Ax,[0 100]);

            % Make axes clickable
            obj.Ax.PickableParts = 'all';
            obj.Ax.HitTest = 'on';
            obj.Ax.ButtonDownFcn = @(ax,evt)obj.onCanvasClick(ax,evt);

            % Preview line for edges
            hold(obj.Ax,'on');
            obj.EdgePreviewLine = line(obj.Ax, [NaN NaN], [NaN NaN], ...
                'LineStyle','--','LineWidth',1, 'HitTest','off');
            hold(obj.Ax,'off');
        end

        function setMode(obj, mode)
            obj.Mode = string(mode);
            obj.EdgePick = [];
            obj.ModeLabel.Text = "Mode: " + obj.Mode;
            obj.EdgeInfoLabel.Text = "";

            obj.EdgePreviewLine.XData = [NaN NaN];
            obj.EdgePreviewLine.YData = [NaN NaN];
        end

        function clearAll(obj)
            cla(obj.Ax);

            obj.agents = Agent.empty;
            obj.targets = Target.empty;
            obj.edges = Edge.empty;

            obj.targetScatterHandle = gobjects(1);
            obj.targetTextHandles = gobjects(0);

            % Recreate preview line after cla
            hold(obj.Ax,'on');
            obj.EdgePreviewLine = line(obj.Ax, [NaN NaN], [NaN NaN], ...
                'LineStyle','--','LineWidth',1, 'HitTest','off');
            hold(obj.Ax,'off');

            grid(obj.Ax,'on'); axis(obj.Ax,'equal');
            xlim(obj.Ax,[0 100]); ylim(obj.Ax,[0 100]);
            obj.setMode("idle");
        end

        function onCanvasClick(obj, ax, ~)
            cp = ax.CurrentPoint;
            pos = [cp(1,1) cp(1,2)];

            switch obj.Mode
                case "addTarget"
                    obj.createTargetAt(pos);

                case "addAgent"
                    obj.createAgentAt(pos);

                case "addEdge"
                    obj.pickTargetForEdge(pos);

                otherwise
                    % idle: do nothing
            end
        end

        function createTargetAt(obj, pos)
            idx = numel(obj.targets) + 1;

            % Create your real Target object (must exist in your project)
            t = Target(idx, pos);
            obj.targets(idx) = t; %#ok<AGROW>

            % Draw targets ourselves (consistent marker & label)
            obj.redrawTargets();
        end

        function redrawTargets(obj)
            % Delete old label handles
            if ~isempty(obj.targetTextHandles)
                try
                    delete(obj.targetTextHandles(ishandle(obj.targetTextHandles)));
                catch
                end
            end
            obj.targetTextHandles = gobjects(0);

            % Update / create scatter
            if isempty(obj.targets)
                if ~isempty(obj.targetScatterHandle) && isgraphics(obj.targetScatterHandle)
                    delete(obj.targetScatterHandle);
                end
                obj.targetScatterHandle = gobjects(1);
                return;
            end

            P = reshape([obj.targets.position], 2, []).'; % Nx2

            hold(obj.Ax,'on');

            if isempty(obj.targetScatterHandle) || ~isgraphics(obj.targetScatterHandle)
                obj.targetScatterHandle = scatter(obj.Ax, P(:,1), P(:,2), 110, ...
                    'Marker','o', 'MarkerFaceColor',[0 0.4 1], 'MarkerEdgeColor','k', ...
                    'LineWidth', 1.2, 'HitTest','off');
            else
                obj.targetScatterHandle.XData = P(:,1);
                obj.targetScatterHandle.YData = P(:,2);
            end

            % Labels T1, T2, ...
            for i = 1:size(P,1)
                obj.targetTextHandles(i) = text(obj.Ax, P(i,1)+1.0, P(i,2)+1.0, ...
                    sprintf('T%d', i), ...
                    'FontWeight','bold', 'Color','k', 'FontSize', 10, ...
                    'HitTest','off'); %#ok<AGROW>
            end

            hold(obj.Ax,'off');
        end

        function createAgentAt(obj, pos)
            if isempty(obj.targets)
                uialert(obj.Fig, 'Add at least 1 target before adding agents.', 'No targets');
                return;
            end

            % Find nearest target and require click within tolerance
            [tIdx, dist] = obj.findNearestTarget(pos);

            tol = 3.0; % same tolerance style as edge picking (data-units)
            if isempty(tIdx) || dist > tol
                obj.EdgeInfoLabel.Text = sprintf('Click on/near a target to place an agent (within %.1f units).', tol);
                return;
            end

            % Snap agent to the target position
            snapPos = obj.targets(tIdx).position;

            idx = numel(obj.agents) + 1;
            spd = obj.SpeedField.Value;

            a = Agent(idx, snapPos, spd);

            % OPTIONAL: store which target it starts on (your Agent has current_target_idx)
            a.current_target_idx = tIdx;

            obj.agents(idx) = a; %#ok<AGROW>

            hold(obj.Ax,'on');
            a.draw(obj.Ax);
            hold(obj.Ax,'off');
        end

        function pickTargetForEdge(obj, clickPos)
            if numel(obj.targets) < 2
                uialert(obj.Fig, 'Add at least 2 targets first.', 'Not enough targets');
                return;
            end

            [tIdx, dist] = obj.findNearestTarget(clickPos);
            tol = 3.0; % click tolerance (data units)

            if isempty(tIdx) || dist > tol
                % If already picked first target, show preview to mouse
                if numel(obj.EdgePick) == 1
                    p1 = obj.targets(obj.EdgePick(1)).position;
                    obj.EdgePreviewLine.XData = [p1(1) clickPos(1)];
                    obj.EdgePreviewLine.YData = [p1(2) clickPos(2)];
                end
                return;
            end

            % prevent picking the same target twice
            if any(obj.EdgePick == tIdx)
                return;
            end

            obj.EdgePick(end+1) = tIdx; %#ok<AGROW>

            if numel(obj.EdgePick) == 1
                obj.EdgeInfoLabel.Text = sprintf('Picked T%d. Pick second target...', tIdx);
                p1 = obj.targets(tIdx).position;
                obj.EdgePreviewLine.XData = [p1(1) clickPos(1)];
                obj.EdgePreviewLine.YData = [p1(2) clickPos(2)];
                return;
            end

            i = obj.EdgePick(1);
            j = obj.EdgePick(2);

            if obj.edgeExists(i, j)
                obj.EdgeInfoLabel.Text = sprintf('Edge between T%d and T%d already exists.', i, j);
            else
                eIdx = numel(obj.edges) + 1;

                % Your Edge class matches this exactly
                e = Edge(eIdx, [obj.targets(i), obj.targets(j)]);
                obj.edges(eIdx) = e; %#ok<AGROW>

                hold(obj.Ax,'on');
                e.draw(obj.Ax);   % uses your Edge.draw
                hold(obj.Ax,'off');

                obj.EdgeInfoLabel.Text = sprintf('Created edge T%d—T%d (L=%.2f)', i, j, e.length);
            end

            % Reset
            obj.EdgePick = [];
            obj.EdgePreviewLine.XData = [NaN NaN];
            obj.EdgePreviewLine.YData = [NaN NaN];
        end

        function tf = edgeExists(obj, i, j)
            tf = false;
            for k = 1:numel(obj.edges)
                try
                    t = obj.edges(k).targets;
                    idx1 = t(1).index;
                    idx2 = t(2).index;
                    if (idx1==i && idx2==j) || (idx1==j && idx2==i)
                        tf = true;
                        return;
                    end
                catch
                end
            end
        end

        function [idx, dist] = findNearestTarget(obj, pos)
            idx = [];
            dist = inf;
            if isempty(obj.targets), return; end

            P = reshape([obj.targets.position], 2, []).'; % Nx2
            d = hypot(P(:,1)-pos(1), P(:,2)-pos(2));
            [dist, idx] = min(d);
        end
    end
end
