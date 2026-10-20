classdef LinearAgent < Agent
    % First-order agent: moves along its path at constant maximum speed (no acceleration)
    properties
        type string = "Linear"
    end

    methods
        function obj = LinearAgent(index, position, maxSpeed)
            obj@Agent(index, position, maxSpeed);
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
    end
end
