classdef PathFollowerController < handle
    % Feedback law of PhysicalAgent: PD tracking of a reference point that moves along
    % the path (position pRef, velocity vRef, acceleration aRef), with acceleration feedforward,
    %   a = aRef + kp (pRef - p) + kd (vRef - v),   |a| <= maxAccel,
    % so the agent keeps up with the planned speed profile instead of chasing a nearby waypoint.
    % The defaults (kp = 16, kd = 8) give a critically damped tracking error (time constant
    % 0.25 s). An agent counts as arrived within arrivePosTol of the end of the path once the
    % reference has stopped and its speed is below arriveSpeedTol.
    properties
        kp = 16.0
        kd = 8.0
        arrivePosTol = 0.5
        arriveSpeedTol = 0.1
    end

    methods
        function aCmd = track(obj, state, pRef, vRef, aRef)
            aCmd = aRef + obj.kp * (pRef - state.pos) + obj.kd * (vRef - state.vel);
            if norm(aCmd) > state.maxAccel
                aCmd = aCmd / norm(aCmd) * state.maxAccel;
            end
        end

        function tf = hasArrived(obj, state, pEnd)
            tf = norm(pEnd - state.pos) < obj.arrivePosTol && norm(state.vel) < obj.arriveSpeedTol;
        end
    end
end
