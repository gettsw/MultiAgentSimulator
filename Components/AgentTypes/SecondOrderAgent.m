classdef SecondOrderAgent < EnergyAgent
    % The energy-aware agent of S. Welikala and C. G. Cassandras, "Event-driven receding horizon
    % control of energy-aware dynamic agents for distributed persistent monitoring", arXiv
    % 2102.12963 (IEEE CDC 2021): a second-order unicycle whose tangential acceleration u is the
    % control. It follows its path by arc length (the heading follows the path, Theorem 2),
    % starts and stops at rest on the targets and stays still while dwelling. On an edge of
    % length y crossed in time rho it uses one of two speed profiles:
    %   "SO"         v(t) = 6y t (rho - t) / rho^3, the energy-optimal profile (Theorem 3);
    %                peak speed 3y/(2 rho), peak acceleration 6y/rho^2, energy 12y^2/rho^3
    %   "trapezoid"  accelerate for rho/3, cruise at 3y/(2 rho), brake for rho/3 (FO-2 and FO-3,
    %                Theorem 4); peak acceleration 9y/(2 rho^2), energy 27y^2/(2 rho^3)
    % A policy sets transitTime and transitProfile before the agent leaves a target (as the RHC
    % policies set dwellRemaining). Without them the agent uses the SO profile with peak speed
    % maxSpeed. Je accumulates the paper's energy objective int u(t)^2 dt exactly (steering is
    % not counted). The battery (e_total, soc) uses the EnergyAgent power model with the
    % tangential acceleration; it is reported only and never stops the agent.
    properties
        transitTime = NaN               % transit time of the next edge [s] (NaN: peak speed maxSpeed)
        transitProfile string = "SO"    % speed profile of the next edge: "SO" or "trapezoid"
        Je = 0                          % energy objective int u(t)^2 dt so far
        peakSpeed = 0                   % largest tangential speed so far
        peakAccel = 0                   % largest |tangential acceleration| so far
    end

    properties (Access = private)
        leg = []   % the edge in progress: struct(pts, s, y, rho, shape, tau)
    end

    methods
        function obj = SecondOrderAgent(index, position, maxSpeed)
            obj@EnergyAgent(index, position, maxSpeed);
        end

        function resetToInitial(obj)
            resetToInitial@EnergyAgent(obj);
            obj.transitTime = NaN;
            obj.transitProfile = "SO";
            obj.Je = 0;
            obj.peakSpeed = 0;
            obj.peakAccel = 0;
            obj.leg = [];
        end

        function update(obj, dt)
            if isempty(obj.path)
                obj.state.vel = [0 0];
                obj.state.acc = [0 0];
                obj.mode = "idle";
            else
                if isempty(obj.leg), obj.startLeg(); end
                L = obj.leg;
                t1 = min(L.tau + dt, L.rho);
                obj.Je = obj.Je + SecondOrderAgent.energy(L.shape, L.y, L.rho, L.tau, t1);
                obj.leg.tau = t1;
                [l, v, u] = SecondOrderAgent.profile(L.shape, L.y, L.rho, t1);
                k = min(find(L.s < l, 1, 'last'), numel(L.s) - 1);   % segment that holds l
                if isempty(k), k = 1; end
                dir = (L.pts(k + 1, :) - L.pts(k, :)) / (L.s(k + 1) - L.s(k));
                obj.state.ori = atan2(dir(2), dir(1));
                obj.mode = "traveling";
                if t1 >= L.rho   % arrived: at rest on the target
                    obj.state.pos = L.pts(end, :);
                    obj.state.vel = [0 0];
                    obj.state.acc = [0 0];
                    obj.path = [];
                    obj.pathIndex = 1;
                    obj.leg = [];
                    obj.transitTime = NaN;
                else
                    obj.state.pos = L.pts(k, :) + (l - L.s(k)) * dir;
                    obj.state.vel = v * dir;
                    obj.state.acc = u * dir;
                end
            end
            obj.calculate_energy(dt);
            obj.updateVisuals();
        end
    end

    methods (Static)
        function [l, v, u] = profile(shape, y, rho, t)
            % Distance l, speed v and tangential acceleration u at time t in [0, rho] on an edge
            % of length y crossed in time rho
            switch shape
                case "SO"
                    s = t / rho;
                    l = y * (3 * s^2 - 2 * s^3);
                    v = 6 * y * t * (rho - t) / rho^3;
                    u = 6 * y * (rho - 2 * t) / rho^3;
                case "trapezoid"
                    uF = 9 * y / (2 * rho^2);
                    if t <= rho / 3
                        l = uF * t^2 / 2; v = uF * t; u = uF;
                    elseif t <= 2 * rho / 3
                        l = y / 4 + 3 * y / (2 * rho) * (t - rho / 3); v = 3 * y / (2 * rho); u = 0;
                    else
                        l = y - uF * (rho - t)^2 / 2; v = uF * (rho - t); u = -uF;
                    end
                otherwise
                    error('SecondOrderAgent:Profile', 'Unknown speed profile "%s"', shape);
            end
        end

        function E = energy(shape, y, rho, t0, t1)
            % int_{t0}^{t1} u(t)^2 dt on the edge (the whole edge, [0, rho], gives E(rho))
            switch shape
                case "SO"
                    E = 6 * y^2 / rho^6 * ((rho - 2 * t0)^3 - (rho - 2 * t1)^3);
                case "trapezoid"
                    ramps = max(0, min(t1, rho / 3) - t0) + max(0, t1 - max(t0, 2 * rho / 3));
                    E = (9 * y / (2 * rho^2))^2 * ramps;
            end
        end
    end

    methods (Access = private)
        function startLeg(obj)
            % Arc lengths of the path from the current position; transit time and profile
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
            shape = obj.transitProfile;
            rho = obj.transitTime;
            if ~(rho > 0), rho = 1.5 * y / obj.state.maxSpeed; end   % peak speed = maxSpeed
            obj.leg = struct('pts', pts, 's', s, 'y', y, 'rho', rho, 'shape', shape, 'tau', 0);
            switch shape
                case "SO",        peaks = [3 * y / (2 * rho), 6 * y / rho^2];
                case "trapezoid", peaks = [3 * y / (2 * rho), 9 * y / (2 * rho^2)];
                otherwise, error('SecondOrderAgent:Profile', 'Unknown speed profile "%s"', shape);
            end
            obj.peakSpeed = max(obj.peakSpeed, peaks(1));
            obj.peakAccel = max(obj.peakAccel, peaks(2));
        end
    end
end
