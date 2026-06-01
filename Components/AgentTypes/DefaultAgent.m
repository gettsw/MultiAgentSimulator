classdef DefaultAgent < handle
    properties
        index
        state              
        controller         
        
        path = []                        
        pathIndex = 1                    
        current_target_idx = []
        dwellRemaining double = 0
        type string = "Default"
        color = [0.8 0.2 0.2]; 
        size = 8;
        graphicHandle; 
        textHandle; 
        ax;
        initialPosition; 
        initialTargetIdx;               
    end

    methods
        function obj = DefaultAgent(index, position, maxSpeed)
            obj.index = index;
            obj.state = KinematicState(position, maxSpeed, 15);
            obj.controller = PathFollowerController();
            obj.initialPosition = position;
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
                obj.state.vel = obj.state.vel * 0.8;
            else
                [arrived, aCmd, nextIdx] = obj.controller.computeControl(obj.state, obj.path, obj.pathIndex, dt);
                
                obj.state.acc = aCmd;
                obj.pathIndex = nextIdx; 

                if arrived
                    obj.path = [];
                    obj.pathIndex = 1;
                end
            end
            
            PhysicalIntegrator.update(obj.state, dt);
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