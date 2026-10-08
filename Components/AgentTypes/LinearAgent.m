classdef LinearAgent < handle
    properties
        index
        state              
        controller         
        
        path = []                        
        pathIndex = 1                    
        current_target_idx = []
        dwellRemaining double = 0
        type string = "Linear"         % Identifies it to your policyMap
        color = [0.8 0.2 0.2];      
        size = 8;
        graphicHandle; 
        textHandle; 
        ax;
        initialPosition; 
        initialTargetIdx;               
    end
    methods
        function obj = LinearAgent(index, position, maxSpeed)
            obj.index = index;
            % Ensure pos is saved as a clean row vector [1x2]
            rowPos = position(:)';
            obj.state = KinematicState(rowPos, maxSpeed, 15);
            obj.controller = PathFollowerController();
            obj.initialPosition = rowPos;
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
        function update(obj, dt)
            if isempty(obj.path)
                obj.state.acc = [0 0];
                obj.state.vel = [0 0];
            else
                % Get the coordinate position of the next waypoint in the path matrix
                % (Handles path as either a matrix of row coordinates or an array of objects)
                if isobject(obj.path)
                    nextPoint = obj.path(obj.pathIndex).position(:)';
                else
                    nextPoint = obj.path(obj.pathIndex, :);
                end
                
                % Calculate direction vector and distance
                toTarget = nextPoint - obj.state.pos;
                distance = norm(toTarget);
                
                % Maximum distance the agent can travel this frame at full speed
                maxStep = obj.state.maxSpeed * dt;
                
                if distance <= maxStep
                    % --- ARRIVED AT WAYPOINT ---
                    obj.state.pos = nextPoint; % Snap precisely to the point
                    obj.state.vel = [0 0];
                    
                    obj.pathIndex = obj.pathIndex + 1;
                    if obj.pathIndex > size(obj.path, 1) || (isobject(obj.path) && obj.pathIndex > numel(obj.path))
                        obj.path = [];
                        obj.pathIndex = 1;
                    end
                else
                    % --- CONSTANT SPEED MOVE ---
                    dir = toTarget / distance;
                    obj.state.vel = dir * obj.state.maxSpeed;
                    obj.state.pos = obj.state.pos + obj.state.vel * dt;
                    obj.state.acc = [0 0];
                    
                    % Calculate orientation angle (heading) based on movement direction
                    obj.state.ori = atan2(dir(2), dir(1));
                end
            end
            
           
            obj.updateVisuals();
        end
        function draw(obj, ax)
            obj.ax = ax;
            if ~isempty(obj.graphicHandle) && isvalid(obj.graphicHandle), delete(obj.graphicHandle); end
            [x, y] = obj.calculateVertices();
            obj.graphicHandle = patch(ax, x, y, obj.color, 'EdgeColor', 'k', 'FaceAlpha', 0.8);
            obj.textHandle = text(ax, obj.state.pos(1), obj.state.pos(2), num2str(obj.index), ...
                'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'w', 'FontSize', 8);
        end
        function updateVisuals(obj)
            if isempty(obj.graphicHandle) || ~isgraphics(obj.graphicHandle), return; end
            [x, y] = obj.calculateVertices();
            set(obj.graphicHandle, 'XData', x, 'YData', y);
            set(obj.textHandle, 'Position', [obj.state.pos(1), obj.state.pos(2), 0]);
        end
    end
    methods (Access=private)
        function [x, y] = calculateVertices(obj)
            h = obj.size * sqrt(3)/2;
            xb = [obj.size/2, -obj.size/2, -obj.size/2, obj.size/2];
            yb = [0, h/2, -h/2, 0];
            x = xb*cos(obj.state.ori) - yb*sin(obj.state.ori) + obj.state.pos(1);
            y = xb*sin(obj.state.ori) + yb*cos(obj.state.ori) + obj.state.pos(2);
        end
    end
end