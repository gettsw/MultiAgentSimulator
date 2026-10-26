classdef SimpleAgent < Agent
    % SIMPLEAGENT  A minimal new agent type: moves at its top speed along its path.
    %
    % Part of the beginner's example in Examples/EventDetection (a new monitoring problem built
    % from four small subclasses: SimpleTarget, SimpleAgent, SimplePolicy and SimpleMetric).
    %
    % The agent: a point robot that moves at constant speed state.maxSpeed straight from one
    % point of its path to the next (no acceleration limit, no feedback control), and stops at
    % the last point.
    %
    % How it plugs in:
    %   - It subclasses Agent, the base of every agent type. Agent already holds the state
    %     (KinematicState: pos, vel, ori, maxSpeed, ...), the path, the dwell, the battery
    %     (Battery) and the drawing.
    %   - It implements update(dt), the one method an agent type must provide. The model
    %     calls it at every step. When the agent has a path (N x 2 points from the path planner,
    %     set by the model when the policy sends the agent to a neighbor), it moves dt seconds
    %     along it and removes the points it reaches; an empty path tells the model the agent
    %     has arrived. While the agent waits at a target the model calls update with no path,
    %     and the agent stays put.
    %   - It reports its velocity, heading and mode ("traveling" or "idle"), which the battery
    %     model (calculate_energy) and the drawing (updateVisuals) use.
    %   - Registered in Components/ComponentRegistry.m as
    %       obj.agentTypes('Simple Agent (example)') = @(idx, pos, sp) SimpleAgent(idx, pos, sp);
    %     so the GUI's Agent Type dropdown and runTrial's AgentType parameter can select it.
    %
    % To make it your own: limit the acceleration, slow down in turns, or add a sensing range.
    % AbstractAgent and PhysicalAgent (Components/AgentTypes) are complete examples.
    methods
        function obj = SimpleAgent(index, position, maxSpeed)
            obj@Agent(index, position, maxSpeed);
            obj.color = [0.15 0.45 0.80];
        end

        function update(obj, dt)
            step = obj.state.maxSpeed * dt;                  % distance to cover in this step
            start = obj.state.pos;
            while step > 0 && ~isempty(obj.path)
                d = obj.path(1, :) - obj.state.pos;
                if norm(d) <= step                           % reach the next path point
                    obj.state.pos = obj.path(1, :);
                    obj.path(1, :) = [];
                    step = step - norm(d);
                else
                    obj.state.pos = obj.state.pos + step * d / norm(d);
                    step = 0;
                end
            end
            obj.state.vel = (obj.state.pos - start) / dt;
            if norm(obj.state.vel) > 0
                obj.state.ori = atan2(obj.state.vel(2), obj.state.vel(1));
                obj.mode = "traveling";
            else
                obj.mode = "idle";
            end
            obj.calculate_energy(dt);                        % battery drain
            obj.updateVisuals();
        end
    end
end
