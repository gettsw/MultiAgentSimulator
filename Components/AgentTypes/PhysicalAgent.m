classdef PhysicalAgent < Agent
    % Physical agent (the counterpart of the idealized AbstractAgent): a second-order
    % (double-integrator) agent that tracks its planned
    % motion with a PD law (PathFollowerController) under speed and acceleration saturation
    % (maxSpeed, maxAccel; PhysicalIntegrator). The reference point moves along the path with the
    % same speed profile an AbstractAgent would follow ("constant", "SO" or "FO", or the
    % one an energy-aware policy hands over), so the agent keeps the planned pace, but lags and
    % overshoots wherever the plan asks more than its limits allow. It arrives once the
    % reference has stopped and it is within arrivePosTol of the target and almost still; while
    % dwelling it holds its position at the target.
    % The profile, maxSpeed, maxAccel and the gains (controller.kp, controller.kd) are set per
    % agent (GUI: Agent Speed / Agent Accel / Speed Profile; runTrial: AgentSpeed,
    % Agent_profile, Agent_maxAccel, Agent_kp, Agent_kd). Je counts the full commanded
    % acceleration (|a|^2, steering included).
    properties (Access = private)
        leg = []          % the edge in progress (Agent.beginLeg)
        holdPoint = []    % where the agent holds while it has no path
    end

    methods
        function obj = PhysicalAgent(index, position, maxSpeed, opts)
            arguments
                index
                position
                maxSpeed
                opts.Profile string = "constant"
            end
            obj@Agent(index, position, maxSpeed);
            obj.profile = opts.Profile;
        end

        function resetToInitial(obj)
            resetToInitial@Agent(obj);
            obj.leg = [];
            obj.holdPoint = [];
        end

        function update(obj, dt)
            c = obj.controller;
            if obj.batteryEmpty()
                aCmd = -obj.state.vel / dt;                       % brake to a stop
                if norm(aCmd) > obj.state.maxAccel, aCmd = aCmd / norm(aCmd) * obj.state.maxAccel; end
                obj.mode = "power_outage";
            elseif isempty(obj.path)
                if isempty(obj.holdPoint), obj.holdPoint = obj.state.pos; end
                aCmd = c.track(obj.state, obj.holdPoint, [0 0], [0 0]);   % hold at the target
                obj.mode = "idle";
            else
                if isempty(obj.leg), obj.startLeg(); end
                L = obj.leg;
                obj.leg.tau = L.tau + dt;
                [l, v, u] = SpeedProfile.at(L.P, obj.leg.tau);
                if obj.leg.tau >= L.P.rho, v = 0; u = 0; end              % the reference has stopped
                [pRef, dir] = obj.pointAt(L, l);
                aCmd = c.track(obj.state, pRef, v * dir, u * dir);
                obj.mode = "traveling";
            end
            obj.state.acc = aCmd;
            ori0 = obj.state.ori;
            PhysicalIntegrator.update(obj.state, dt);                  % heading from the velocity
            obj.state.omega = angle(exp(1i * (obj.state.ori - ori0))) / dt;
            obj.Je = obj.Je + (aCmd * aCmd') * dt;
            obj.peakSpeed = max(obj.peakSpeed, norm(obj.state.vel));
            obj.peakAccel = max(obj.peakAccel, norm(aCmd));
            if ~isempty(obj.leg) && obj.leg.tau >= obj.leg.P.rho - 1e-9 * max(1, obj.leg.P.rho) ...
                    && c.hasArrived(obj.state, obj.leg.pts(end, :))
                obj.holdPoint = obj.leg.pts(end, :);                % arrived
                obj.path = [];
                obj.pathIndex = 1;
                obj.leg = [];
                obj.clearTransitPlan();
            end
            obj.calculate_energy(dt);
            obj.updateVisuals();
        end
    end

    methods (Access = private)
        function startLeg(obj)
            obj.leg = obj.beginLeg();
            obj.holdPoint = [];
        end
    end
end
