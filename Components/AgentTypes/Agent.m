classdef (Abstract) Agent < handle & matlab.mixin.Heterogeneous
    % Base class of every agent type. A subclass implements update(dt) (its motion along
    % obj.path) and may extend resetToInitial and label. Sharing this base
    % (matlab.mixin.Heterogeneous) lets agents of different types live in the same
    % ScenarioModel.agents array.
    %
    % What every agent has:
    %   - kinematic state and limits (KinematicState: pos, vel, acc, maxSpeed, maxAccel)
    %   - a speed profile for its transits (SpeedProfile: "constant", "SO" or "FO"). A policy
    %     that plans a transit may hand the agent its transit time, profile and speed /
    %     acceleration limits before it leaves a target (as RHC policies set dwellRemaining);
    %     otherwise the agent uses its own profile with peak speed maxSpeed
    %   - energy metrics: Je = int |u|^2 dt (NaN once a "constant" transit is used: its
    %     impulsive accelerations make it undefined), peak speed and acceleration
    %   - a battery (power model below), reported as SOC (about an hour of operation on the
    %     default 26 Wh); with stopWhenEmpty = true the agent stops when it is empty
    properties
        index
        state                       % KinematicState
        controller                  % PathFollowerController (feedback law of PhysicalAgent)
        path = []                   % N x 2 waypoints to the destination target
        pathIndex = 1
        current_target_idx = []     % destination target, or the target the agent is at
        dwellRemaining double = 0   % seconds left to wait at the current target
        initialPosition
        initialTargetIdx

        profile string = "constant" % speed profile of its transits: "constant", "SO" or "FO"
        transitTime = NaN           % planned transit time of the next edge (NaN: from maxSpeed)
        transitProfile string = ""  % profile of the next edge ("" = profile)
        transitLimits = [Inf Inf]   % [speed, acceleration] limits of the next edge's profile

        Je = 0                      % energy objective int |u(t)|^2 dt so far
        peakSpeed = 0               % largest speed so far
        peakAccel = 0               % largest |acceleration| so far

        mode string = "idle"        % "idle", "traveling" or "power_outage" (battery)
        e_total = 0                 % energy drawn from the battery so far [J]
        soc = 1.0                   % battery state of charge
        battery_percentage = 100
        stopWhenEmpty = false       % true: stop moving when the battery is empty (default:
                                    % the battery is only reported, as in the RHC papers)

        color = [0.8 0.2 0.2]
        size = 8
        labelFontSize = 7
        graphicHandle
        textHandle
        ax
    end

    properties (Constant)
        % Battery power model: P = (P_cpu + P_comp + P0 + ALPHA_VEL |v| + GAMMA_ACC |a|^2) / ETA_CONV
        MASS = 10.0;
        ROLLING_FRICTION = 0.02;
        GRAVITY = 9.81;
        P0_BASE_MOTION = 5.82;
        ALPHA_VEL = 0.02 * 10 * 9.81;
        GAMMA_ACC = 0.5;
        CPU_MOVE = 5.5;
        CPU_IDLE = 4.0;
        COMP_ACTIVE = 11.7;
        COMP_STANDBY = 11.0;
        BATTERY_E_Wh = 26.0;
        ETA_CONV = 0.90;
    end

    methods (Abstract)
        update(obj, dt)   % advance the motion by dt along obj.path; clear obj.path on arrival
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
            obj.state.omega = 0;
            obj.path = [];
            obj.pathIndex = 1;
            obj.dwellRemaining = 0;
            obj.current_target_idx = obj.initialTargetIdx;
            obj.clearTransitPlan();
            obj.Je = 0;
            obj.peakSpeed = 0;
            obj.peakAccel = 0;
            obj.mode = "idle";
            obj.e_total = 0;
            obj.soc = 1.0;
            obj.battery_percentage = 100;
        end

        function set.profile(obj, p)
            assert(any(p == SpeedProfile.names), 'Agent:Profile', ...
                'Unknown speed profile "%s" (use %s)', p, strjoin(SpeedProfile.names, ", "));
            obj.profile = p;
        end

        function s = label(obj)
            % Text drawn on the agent: its index and battery level
            s = sprintf('%d\n%.0f%%', obj.index, obj.battery_percentage);
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

        function calculate_energy(obj, dt)
            % Battery drain over dt from the current speed, acceleration and mode
            v_norm = norm(obj.state.vel);
            a_norm = norm(obj.state.acc);
            p_mot = obj.P0_BASE_MOTION + (obj.ALPHA_VEL * v_norm) + (obj.GAMMA_ACC * a_norm^2);
            if obj.mode == "traveling"
                p_cpu = obj.CPU_MOVE; p_comp = obj.COMP_ACTIVE;
            else
                p_cpu = obj.CPU_IDLE; p_comp = obj.COMP_STANDBY;
            end
            energy_step = (p_cpu + p_comp + p_mot) / obj.ETA_CONV * dt;
            obj.e_total = obj.e_total + energy_step;
            obj.soc = obj.soc - energy_step / (obj.BATTERY_E_Wh * 3600);
            obj.battery_percentage = max(0, obj.soc * 100);
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

        function L = beginLeg(obj)
            % The transit that starts now along obj.path: arc lengths from the current position
            % (L.pts, L.s), the speed profile L.P (planned by the policy, or the agent's own
            % with peak speed maxSpeed; it depends on the path length only), and the heading
            % along the path (L.th per segment, turned over a window of half-width L.h around
            % each vertex; see headingAt)
            if isobject(obj.path)
                pts = cell2mat(arrayfun(@(w) w.position(:)', obj.path(:), 'UniformOutput', false));
            else
                pts = obj.path;
            end
            pts = [obj.state.pos; pts];
            s = [0; cumsum(vecnorm(diff(pts, 1, 1), 2, 2))];
            keep = [true; diff(s) > 0];                 % drop repeated points
            pts = pts(keep, :); s = s(keep);
            if numel(s) < 2                             % already there
                pts = [pts; pts]; s = [0; eps];
            end
            y = s(end);
            d = diff(pts, 1, 1);
            th = unwrap(atan2(d(:, 2), d(:, 1)));
            seg = diff(s);
            h = min([0.5 * ones(numel(seg) - 1, 1), seg(1:end-1) / 2, seg(2:end) / 2], [], 2);
            name = obj.transitProfile;
            if name == "", name = obj.profile; end
            lim = obj.transitLimits;
            rho = obj.transitTime;
            if ~(rho > 0), rho = SpeedProfile.defaultTransit(name, y, obj.state.maxSpeed); end
            rho = max(rho, SpeedProfile.minTransit(y, lim(1), lim(2)));
            L = struct('pts', pts, 's', s, 'P', SpeedProfile.shape(name, y, rho, lim(1), lim(2)), ...
                'tau', 0, 'e', 0, 'th', th, 'h', h);
        end

        function [p, dir] = pointAt(~, L, l)
            % Position at arc length l along the leg, and the unit tangent there
            k = min(find(L.s < l, 1, 'last'), numel(L.s) - 1);
            if isempty(k), k = 1; end
            dir = (L.pts(k + 1, :) - L.pts(k, :)) / (L.s(k + 1) - L.s(k));
            p = L.pts(k, :) + (min(l, L.s(end)) - L.s(k)) * dir;
        end

        function [theta, kappa] = headingAt(~, L, l)
            % Heading theta and its rate per unit length kappa (the path curvature) at arc
            % length l. The heading follows the segment directions and turns linearly across a
            % window around each vertex (half-width up to 0.5), so the angular velocity
            % omega = kappa * v stays finite (Welikala and Cassandras, arXiv 2102.12963,
            % Theorem 2: omega = F(l) v). Curvature never enters the speed profile.
            n = numel(L.s);
            k = min(find(L.s < l, 1, 'last'), n - 1);
            if isempty(k), k = 1; end
            theta = L.th(k); kappa = 0;
            if k > 1 && l < L.s(k) + L.h(k - 1)                 % turning at the vertex where k starts
                w = 2 * L.h(k - 1); dth = L.th(k) - L.th(k - 1);
                theta = L.th(k - 1) + dth * (l - L.s(k) + L.h(k - 1)) / w; kappa = dth / w;
            elseif k < n - 1 && l > L.s(k + 1) - L.h(k)         % turning at the vertex where k ends
                w = 2 * L.h(k); dth = L.th(k + 1) - L.th(k);
                theta = L.th(k) + dth * (l - L.s(k + 1) + L.h(k)) / w; kappa = dth / w;
            end
        end

        function clearTransitPlan(obj)
            obj.transitTime = NaN;
            obj.transitProfile = "";
            obj.transitLimits = [Inf Inf];
        end

        function tf = batteryEmpty(obj)
            tf = obj.stopWhenEmpty && obj.battery_percentage <= 0;
        end
    end
end
