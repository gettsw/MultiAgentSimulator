classdef (Abstract) Agent < handle & matlab.mixin.Heterogeneous
    % Base class of every agent type. A subclass implements update(dt) (its dynamics) and may
    % extend resetToInitial and label. Sharing this base (matlab.mixin.Heterogeneous) lets
    % agents of different types live in the same ScenarioModel.agents array.
    properties
        index
        state                       % KinematicState
        controller                  % PathFollowerController
        path = []                   % N x 2 waypoints to the destination target
        pathIndex = 1
        current_target_idx = []     % destination target, or the target the agent is at
        dwellRemaining double = 0   % seconds left to wait at the current target
        initialPosition
        initialTargetIdx
        color = [0.8 0.2 0.2]
        size = 8
        labelFontSize = 8
        graphicHandle
        textHandle
        ax
    end

    methods (Abstract)
        update(obj, dt)   % advance the dynamics by dt along obj.path; clear obj.path on arrival
    end

    methods
        function obj = Agent(index, position, maxSpeed)
            obj.index = index;
            obj.initialPosition = position(:)';
            obj.state = KinematicState(obj.initialPosition, maxSpeed, 15);
            obj.controller = PathFollowerController();
        end

        function resetToInitial(obj)
            obj.state.pos = obj.initialPosition;
            obj.state.vel = [0 0];
            obj.state.acc = [0 0];
            obj.state.ori = 0;
            obj.path = [];
            obj.pathIndex = 1;
            obj.dwellRemaining = 0;
            obj.current_target_idx = obj.initialTargetIdx;
        end

        function s = label(obj)
            % Text drawn on the agent
            s = num2str(obj.index);
        end

        function draw(obj, ax)
            obj.ax = ax;
            if ~isempty(obj.graphicHandle) && isvalid(obj.graphicHandle), delete(obj.graphicHandle); end
            [x, y] = obj.calculateVertices();
            % Graphics let clicks through to the canvas so a target under an agent stays clickable
            obj.graphicHandle = patch(ax, x, y, obj.color, 'EdgeColor', 'k', 'FaceAlpha', 0.8, 'PickableParts', 'none');
            obj.textHandle = text(ax, obj.state.pos(1), obj.state.pos(2), obj.label(), ...
                'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'w', ...
                'FontSize', obj.labelFontSize, 'PickableParts', 'none');
        end

        function updateVisuals(obj)
            if isempty(obj.graphicHandle) || ~isgraphics(obj.graphicHandle), return; end
            [x, y] = obj.calculateVertices();
            set(obj.graphicHandle, 'XData', x, 'YData', y);
            set(obj.textHandle, 'Position', [obj.state.pos(1), obj.state.pos(2), 0], 'String', obj.label());
        end
    end

    methods (Access = protected)
        function [x, y] = calculateVertices(obj)
            h = obj.size * sqrt(3)/2;
            xb = [obj.size/2, -obj.size/2, -obj.size/2, obj.size/2];
            yb = [0, h/2, -h/2, 0];
            x = xb*cos(obj.state.ori) - yb*sin(obj.state.ori) + obj.state.pos(1);
            y = xb*sin(obj.state.ori) + yb*cos(obj.state.ori) + obj.state.pos(2);
        end
    end
end
