classdef DefaultAgent < Agent
    % Second-order agent: PD path follower (PathFollowerController) with speed and
    % acceleration limits (KinematicState)
    properties
        type string = "Default"
    end

    methods
        function obj = DefaultAgent(index, position, maxSpeed)
            obj@Agent(index, position, maxSpeed);
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
    end
end
