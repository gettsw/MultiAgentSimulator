classdef ScenarioRenderer < handle
    properties
        ax
        edgePreviewLine
        bgImageHandle
    end

    methods
        function obj = ScenarioRenderer(ax)
            obj.ax = ax;
            hold(obj.ax, 'on');
            obj.edgePreviewLine = line(obj.ax, [NaN NaN], [NaN NaN], ...
                'LineStyle', '--', 'LineWidth', 1, 'HitTest', 'off');
            hold(obj.ax, 'off');
        end

        function loadBackgroundImage(obj)
            [file, path] = uigetfile({'*.jpg;*.png;*.bmp;*.tif', 'Image Files (*.jpg, *.png, *.bmp, *.tif)'});
            if isequal(file, 0), return; end
            img = imread(fullfile(path, file));
            img = flipud(img);
            if ~isempty(obj.bgImageHandle) && isgraphics(obj.bgImageHandle)
                delete(obj.bgImageHandle);
            end
            hold(obj.ax, 'on');
            obj.bgImageHandle = image(obj.ax, [0 100], [0 100], img, 'HandleVisibility', 'off');
            set(obj.bgImageHandle, 'AlphaData', 0.85);
            set(obj.ax, 'YDir', 'normal');
            set(obj.ax, 'Layer', 'top');
            uistack(obj.bgImageHandle, 'bottom');
            hold(obj.ax, 'off');
        end

        function clearAxes(obj)
            if ~isempty(obj.bgImageHandle) && isgraphics(obj.bgImageHandle)
                allObjs = allchild(obj.ax);
                toDelete = allObjs(allObjs ~= obj.bgImageHandle);
                delete(toDelete);
            else
                try
                    delete(allchild(obj.ax));
                catch
                    cla(obj.ax, 'reset');
                end
            end
            grid(obj.ax, 'on');
            axis(obj.ax, 'equal');
            xlim(obj.ax, [0 100]); ylim(obj.ax, [0 100]);
            hold(obj.ax, 'on');
            obj.edgePreviewLine = line(obj.ax, [NaN NaN], [NaN NaN], ...
                'LineStyle', '--', 'LineWidth', 1, 'HitTest', 'off');
            hold(obj.ax, 'off');
        end

        function resetEdgePreview(obj)
            if ~isempty(obj.edgePreviewLine) && isgraphics(obj.edgePreviewLine)
                obj.edgePreviewLine.XData = [NaN NaN];
                obj.edgePreviewLine.YData = [NaN NaN];
            end
        end

        function updateEdgePreview(obj, p1, p2)
            if ~isempty(obj.edgePreviewLine) && isgraphics(obj.edgePreviewLine)
                obj.edgePreviewLine.XData = [p1(1) p2(1)];
                obj.edgePreviewLine.YData = [p1(2) p2(2)];
            end
        end

        function renderAll(obj, model)
            obj.renderWalls(model);
            obj.renderEdges(model);
            obj.renderTargets(model);
            obj.renderAgents(model);
        end

        function renderWalls(obj, model)
            if isempty(model.walls), return; end
            hold(obj.ax, 'on');
            for i = 1:size(model.walls, 1)
                w = model.walls(i, :);
                line(obj.ax, [w(1) w(3)], [w(2) w(4)], ...
                    'Color', [0.2 0.2 0.2], 'LineWidth', 4, 'Tag', 'Wall');
            end
            hold(obj.ax, 'off');
        end

        function renderTargets(obj, model)
            for k = 1:numel(model.targets)
                t = model.targets(k);
                if isempty(t.graphicHandle) || ~isgraphics(t.graphicHandle)
                    hold(obj.ax, 'on');
                    t.draw(obj.ax);
                    hold(obj.ax, 'off');
                end
            end
        end

        function renderAgents(obj, model)
            for k = 1:numel(model.agents)
                a = model.agents(k);
                if isempty(a.graphicHandle) || ~isgraphics(a.graphicHandle)
                    hold(obj.ax, 'on');
                    a.draw(obj.ax);
                    hold(obj.ax, 'off');
                else
                    a.updateVisuals();
                end
            end
        end

        function renderEdges(obj, model)
            for k = 1:numel(model.edges)
                e = model.edges(k);
                if isempty(e.lineHandle) || ~isgraphics(e.lineHandle)
                    hold(obj.ax, 'on');
                    e.lineHandle = plot(obj.ax, e.curvePoints(:,1), e.curvePoints(:,2), ...
                        'Color', [0.6 0.6 0.6], 'LineWidth', 1.5, 'LineStyle', '--', ...
                        'PickableParts', 'none');
                    uistack(e.lineHandle, 'bottom');
                    hold(obj.ax, 'off');
                end
            end
        end
    end
end