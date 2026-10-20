classdef SecondOrderAgent < EnergyAgent
    % The energy-aware agent of S. Welikala and C. G. Cassandras, "Event-driven receding horizon
    % control of energy-aware dynamic agents for distributed persistent monitoring", arXiv
    % 2102.12963 (IEEE CDC 2021): a second-order unicycle whose tangential acceleration u is the
    % control. It follows its path by arc length (the heading follows the path, Theorem 2),
    % starts and stops at rest on the targets and stays still while dwelling.
    %
    % On an edge of length y crossed in time rho it uses the minimum-energy (min int u^2 dt)
    % speed profile of its family, under a speed limit vb and an acceleration limit ub (Inf =
    % no limit):
    %   "SO"  any profile. Without limits v(t) = 6y t (rho - t)/rho^3, E = 12 y^2/rho^3 (Theorem 3).
    %         A reached limit changes the shape (App. D, generalized to both limits):
    %           speed:  u falls linearly to 0 as v reaches vb, cruise, mirror image
    %                   (u stays continuous, unlike the paper's SO-V, which uses more energy)
    %           accel:  u = ub, then linear from ub to -ub, then -ub (the paper's SO-A)
    %           both:   u = ub, then linear down to 0 as v reaches vb, cruise, mirror image
    %   "FO"  trapezoids (accelerate, cruise, brake; the paper's FO-3). Without limits the
    %         cruise speed is 3y/(2 rho), E = 27 y^2/(2 rho^3) (Theorem 5); with limits it is
    %         clamped to what they allow (App. E's FO-V and FO-A)
    % Limits make transits shorter than minTransit infeasible.
    %
    % A policy sets transitTime, transitProfile and transitLimits before the agent leaves a
    % target (as the RHC policies set dwellRemaining). Without them the agent uses the SO profile
    % with peak speed maxSpeed. Je accumulates the paper's energy objective int u(t)^2 dt
    % exactly (steering is not counted). The battery (e_total, soc) uses the EnergyAgent power
    % model with the tangential acceleration; it is reported only and never stops the agent.
    properties
        transitTime = NaN               % transit time of the next edge [s] (NaN: SO with peak speed maxSpeed)
        transitProfile string = "SO"    % profile family of the next edge: "SO" or "FO"
        transitLimits = [Inf Inf]       % [speed limit, acceleration limit] of the next edge
        Je = 0                          % energy objective int u(t)^2 dt so far
        peakSpeed = 0                   % largest tangential speed so far
        peakAccel = 0                   % largest |tangential acceleration| so far
    end

    properties (Access = private)
        leg = []   % the edge in progress: struct(pts, s, P (profile), tau, e)
    end

    methods
        function obj = SecondOrderAgent(index, position, maxSpeed)
            obj@EnergyAgent(index, position, maxSpeed);
        end

        function resetToInitial(obj)
            resetToInitial@EnergyAgent(obj);
            obj.transitTime = NaN;
            obj.transitProfile = "SO";
            obj.transitLimits = [Inf Inf];
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
                t1 = min(L.tau + dt, L.P.rho);
                [l, v, u, e] = SecondOrderAgent.profileAt(L.P, t1);
                obj.Je = obj.Je + e - L.e;
                obj.leg.tau = t1;
                obj.leg.e = e;
                k = min(find(L.s < l, 1, 'last'), numel(L.s) - 1);   % segment that holds l
                if isempty(k), k = 1; end
                dir = (L.pts(k + 1, :) - L.pts(k, :)) / (L.s(k + 1) - L.s(k));
                obj.state.ori = atan2(dir(2), dir(1));
                obj.mode = "traveling";
                if t1 >= L.P.rho   % arrived: at rest on the target
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
        function P = shape(family, y, rho, vb, ub)
            % The minimum-energy profile of <family> ("SO" or "FO") on an edge of length y crossed
            % in time rho (at least minTransit) under speed limit vb and acceleration limit ub.
            % P.kind: "cubic" (unconstrained SO), "sat" (SO with u at its limit, no cruise), or
            % "arc": accelerate at a for s, ramp u from a to 0 over d, cruise at v (SO with the
            % speed limit reached; d = 0 for FO trapezoids)
            P = struct('family', family, 'y', y, 'rho', rho, 'kind', "arc", 'a', 0, 's', 0, 'd', 0, 'v', 0);
            switch family
                case "SO"
                    [~, regime, p] = SecondOrderAgent.soEnergy(y, rho, vb, ub);
                    switch regime
                        case "none", P.kind = "cubic";
                        case "A",    P.kind = "sat"; P.a = ub; P.s = p;
                        case "V",    P.a = 2 * vb / p; P.d = p; P.v = vb;
                        case "VA",   P.a = ub; P.d = p; P.s = vb / ub - p / 2; P.v = vb;
                    end
                case "FO"
                    [~, vc] = SecondOrderAgent.foEnergy(y, rho, vb, ub);
                    P.v = vc; P.a = vc^2 / (vc * rho - y); P.s = vc / P.a;
                otherwise
                    error('SecondOrderAgent:Profile', 'Unknown profile family "%s"', family);
            end
        end

        function [l, v, u, e] = profileAt(P, t)
            % Distance l, speed v, tangential acceleration u and energy int_0^t u^2 at time t in
            % [0, rho]; every profile is symmetric about rho/2
            if t <= P.rho / 2
                [l, v, u, e] = SecondOrderAgent.firstHalf(P, t);
            else
                [l, v, u, e] = SecondOrderAgent.firstHalf(P, P.rho - t);
                [~, ~, ~, half] = SecondOrderAgent.firstHalf(P, P.rho / 2);
                l = P.y - l; u = -u; e = 2 * half - e;
            end
        end

        function E = transitEnergy(family, y, rho, vb, ub)
            % Energy int u^2 dt of the minimum-energy profile over the whole edge, for each
            % transit time in rho (vectorized)
            if family == "SO"
                E = SecondOrderAgent.soEnergy(y, rho, vb, ub);
            else
                E = SecondOrderAgent.foEnergy(y, rho, vb, ub);
            end
        end

        function r = minTransit(y, vb, ub)
            % Shortest transit time the limits allow (same for both families: accelerate at ub,
            % cruise at vb, brake). Under a speed limit alone the energy grows without bound as
            % rho approaches y/vb, so 0.1% is added there.
            if isfinite(vb) && isfinite(ub)
                if y >= vb^2 / ub, r = y / vb + vb / ub; else, r = 2 * sqrt(y / ub); end
            elseif isfinite(vb)
                r = 1.001 * y / vb;
            elseif isfinite(ub)
                r = 2 * sqrt(y / ub);
            else
                r = 0;
            end
        end
    end

    methods (Static, Access = private)
        function [E, regime, p] = soEnergy(y, rho, vb, ub)
            % Minimum energy of any rest-to-rest profile (a convex problem with a unique optimum)
            % and, for scalar rho, the regime ("none", "V", "A", "VA") and its parameter p:
            % t1 for "V", the saturation time s for "A", the ramp time d for "VA"
            E = 12 * y^2 ./ rho.^3;
            todo = 3 * y ./ (2 * rho) > vb | 6 * y ./ rho.^2 > ub;   % a limit would be broken
            regime = "none"; p = 0;
            if isfinite(vb)   % speed limit only: needs peak acceleration 2vb/t1 <= ub
                t1 = 3 * (vb * rho - y) / (2 * vb);
                k = todo & 3 * y ./ (2 * rho) > vb & t1 > 0 & 2 * vb ./ t1 <= ub * (1 + 1e-12);
                E(k) = 16 * vb^3 ./ (9 * (vb * rho(k) - y));
                todo = todo & ~k;
                if isscalar(rho) && k, regime = "V"; p = t1; end
            end
            if isfinite(ub)   % acceleration limit only: needs peak speed ub (s + m/4) <= vb
                s = (rho - sqrt(max(3 * rho.^2 - 12 * y / ub, 0))) / 2;
                k = todo & 6 * y ./ rho.^2 > ub & ub * (s + (rho - 2 * s) / 4) <= vb * (1 + 1e-12);
                E(k) = ub^2 * (rho(k) + 4 * s(k)) / 3;
                todo = todo & ~k;
                if isscalar(rho) && k, regime = "A"; p = s; end
            end
            if any(todo) && isfinite(vb) && isfinite(ub)   % both limits reached
                d = sqrt(max(12 * (vb * rho - y - vb^2 / ub) / ub, 0));
                E(todo) = 2 * ub * vb - ub^2 * d(todo) / 3;
                if isscalar(rho), regime = "VA"; p = d; end
            end
        end

        function [E, vc] = foEnergy(y, rho, vb, ub)
            % Minimum energy of a trapezoid: the energy 2 vc^3/(vc rho - y) falls with the cruise
            % speed vc up to 3y/(2 rho), so vc is that value clamped to the limits: at most vb,
            % and at least the vc whose acceleration vc^2/(vc rho - y) is ub
            if isfinite(ub)
                vlo = rho * ub / 2 .* (1 - sqrt(max(1 - 4 * y ./ (ub * rho.^2), 0)));
            else
                vlo = 0;
            end
            vc = min(vb, max(vlo, 3 * y ./ (2 * rho)));
            E = 2 * vc.^3 ./ (vc .* rho - y);
        end

        function [l, v, u, e] = firstHalf(P, t)
            y = P.y; rho = P.rho;
            switch P.kind
                case "cubic"
                    q = t / rho;
                    l = y * (3 * q^2 - 2 * q^3);
                    v = 6 * y * t * (rho - t) / rho^3;
                    u = 6 * y * (rho - 2 * t) / rho^3;
                    e = 6 * y^2 / rho^6 * (rho^3 - (rho - 2 * t)^3);
                case "sat"      % u = a until s, then linear to -a at rho - s
                    a = P.a; s = P.s; m = rho - 2 * s;
                    if t <= s
                        u = a; v = a * t; l = a * t^2 / 2; e = a^2 * t;
                    else
                        tau = t - s;
                        u = a * (1 - 2 * tau / m);
                        v = a * (s + tau - tau^2 / m);
                        l = a * s^2 / 2 + a * s * tau + a * (tau^2 / 2 - tau^3 / (3 * m));
                        e = a^2 * s + a^2 * m / 6 * (1 - (1 - 2 * tau / m)^3);
                    end
                case "arc"      % u = a until s, then linear to 0 over d, then cruise at v
                    a = P.a; s = P.s; d = P.d;
                    if t <= s
                        u = a; v = a * t; l = a * t^2 / 2; e = a^2 * t;
                    elseif t <= s + d
                        tau = t - s;
                        u = a * (1 - tau / d);
                        v = a * s + a * (tau - tau^2 / (2 * d));
                        l = a * s^2 / 2 + a * s * tau + a * (tau^2 / 2 - tau^3 / (6 * d));
                        e = a^2 * s + a^2 * d / 3 * (1 - (1 - tau / d)^3);
                    else
                        u = 0; v = P.v;
                        l = a * s^2 / 2 + a * s * d + a * d^2 / 3 + P.v * (t - s - d);
                        e = a^2 * s + a^2 * d / 3;
                    end
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
            family = obj.transitProfile;
            lim = obj.transitLimits;
            rho = obj.transitTime;
            if ~(rho > 0)                               % default: SO, peak speed = maxSpeed
                family = "SO"; lim = [Inf Inf]; rho = 1.5 * y / obj.state.maxSpeed;
            end
            rho = max(rho, SecondOrderAgent.minTransit(y, lim(1), lim(2)));
            P = SecondOrderAgent.shape(family, y, rho, lim(1), lim(2));
            obj.leg = struct('pts', pts, 's', s, 'P', P, 'tau', 0, 'e', 0);
            [~, vPeak] = SecondOrderAgent.profileAt(P, rho / 2);
            [~, ~, uPeak] = SecondOrderAgent.profileAt(P, 0);
            obj.peakSpeed = max(obj.peakSpeed, vPeak);
            obj.peakAccel = max(obj.peakAccel, abs(uPeak));
        end
    end
end
