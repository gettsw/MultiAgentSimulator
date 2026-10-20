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
            obj.state.acc = [0 0];
            if isempty(obj.path)
                obj.state.vel = [0 0];
            else
                % Travel maxSpeed*dt along the path, passing every waypoint that distance
                % reaches, so the speed is constant along curved paths too (transit time =
                % path length / maxSpeed, as event-driven RHC assumes)
                remaining = obj.state.maxSpeed * dt;
                dir = [0 0];
                while ~isempty(obj.path)
                    toNext = obj.waypoint(obj.pathIndex) - obj.state.pos;
                    distance = norm(toNext);
                    if distance > 0, dir = toNext / distance; end
                    if distance > remaining
                        obj.state.pos = obj.state.pos + dir * remaining;
                        break;
                    end
                    obj.state.pos = obj.waypoint(obj.pathIndex);   % reach it, carry on with the rest
                    remaining = remaining - distance;
                    obj.pathIndex = obj.pathIndex + 1;
                    if obj.pathIndex > obj.numWaypoints()
                        obj.path = [];
                        obj.pathIndex = 1;
                    end
                end
                if isempty(obj.path)
                    obj.state.vel = [0 0];               % arrived
                else
                    obj.state.vel = dir * obj.state.maxSpeed;
                end
                if any(dir), obj.state.ori = atan2(dir(2), dir(1)); end
            end

            obj.updateVisuals();
        end
    end

    methods (Access = private)
        % The path is a matrix of [x y] rows, or an array of objects with a position
        function p = waypoint(obj, k)
            if isobject(obj.path)
                p = obj.path(k).position(:)';
            else
                p = obj.path(k, :);
            end
        end

        function n = numWaypoints(obj)
            if isobject(obj.path), n = numel(obj.path); else, n = size(obj.path, 1); end
        end
    end
end
