classdef AbstractAgent < Agent
    % Abstract agent (an idealized model, the counterpart of PhysicalAgent; not a MATLAB
    % abstract class): follows its path by arc length exactly as its speed profile prescribes,
    % so every transit takes exactly the planned time and the agent is at rest on the targets.
    % This is the agent model of the event-driven RHC papers:
    %   profile "constant"  a first-order agent moving at maxSpeed (Welikala and Cassandras,
    %                       Automatica 2021; transit time = path length / maxSpeed)
    %   profile "SO", "FO"  a second-order agent with the energy-optimal profiles of Welikala and
    %                       Cassandras, arXiv 2102.12963 (heading from the path, Theorem 2)
    % The profile, maxSpeed and maxAccel are set per agent (GUI: Agent Speed / Agent Accel /
    % Speed Profile; runTrial: AgentSpeed, Agent_profile, Agent_maxAccel). Energy-aware RHC
    % policies hand over the profile, transit time and limits of each transit. On curved paths
    % the speed profile depends on the path length only; the heading follows the path and turns
    % at omega = kappa * v (Theorem 2 of the energy-aware paper; Agent.headingAt). Steering
    % costs nothing: Je counts the tangential acceleration only.
    properties (Access = private)
        leg = []   % the edge in progress (Agent.beginLeg)
    end

    methods
        function obj = AbstractAgent(index, position, maxSpeed, opts)
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
        end

        function update(obj, dt)
            if isempty(obj.path) || obj.batteryEmpty()
                obj.state.vel = [0 0];
                obj.state.acc = [0 0];
                obj.state.omega = 0;
                if isempty(obj.path), obj.mode = "idle"; else, obj.mode = "power_outage"; end
            else
                if isempty(obj.leg), obj.startLeg(); end
                L = obj.leg;
                t1 = L.tau + dt;
                if t1 >= L.P.rho - 1e-9 * max(1, L.P.rho), t1 = L.P.rho; end   % no round-off step
                [l, v, u, e] = SpeedProfile.at(L.P, t1);
                obj.Je = obj.Je + e - L.e;
                obj.leg.tau = t1;
                obj.leg.e = e;
                [p, dir] = obj.pointAt(L, l);
                [obj.state.ori, kappa] = obj.headingAt(L, l);
                obj.state.omega = kappa * v;
                obj.mode = "traveling";
                if t1 >= L.P.rho   % arrived: at rest on the target
                    obj.state.pos = L.pts(end, :);
                    obj.state.vel = [0 0];
                    obj.state.acc = [0 0];
                    obj.state.omega = 0;
                    obj.path = [];
                    obj.pathIndex = 1;
                    obj.leg = [];
                    obj.clearTransitPlan();
                else
                    obj.state.pos = p;
                    obj.state.vel = v * dir;
                    obj.state.acc = u * dir;
                end
            end
            obj.calculate_energy(dt);
            obj.updateVisuals();
        end
    end

    methods (Access = private)
        function startLeg(obj)
            obj.leg = obj.beginLeg();
            P = obj.leg.P;
            if P.name == "constant", obj.Je = NaN; end   % impulsive accelerations: undefined
            [vPeak, uPeak] = SpeedProfile.peaks(P);
            obj.peakSpeed = max(obj.peakSpeed, vPeak);
            obj.peakAccel = max(obj.peakAccel, uPeak);
        end
    end
end
