classdef SpeedProfile
    % Rest-to-rest speed profiles along an edge of length y crossed in time rho, shared by the
    % agent types (AbstractAgent follows them exactly, PhysicalAgent tracks them)
    % and by the energy-aware RHC policies. A profile family may be bounded by a speed limit vb
    % and an acceleration limit ub (Inf = no limit):
    %   "constant"  constant speed y/rho (a first-order agent; the jumps at both ends are
    %               impulses, so the energy int u^2 dt is undefined)
    %   "SO"        the minimum-energy profile (min int u^2 dt). Without limits v(t) =
    %               6y t (rho - t)/rho^3, E = 12 y^2/rho^3 (Welikala and Cassandras, arXiv
    %               2102.12963, Theorem 3). A reached limit changes the shape (App. D,
    %               generalized to both limits):
    %                 speed:  u falls linearly to 0 as v reaches vb, cruise, mirror image (u stays
    %                         continuous, unlike the paper's SO-V, which uses more energy)
    %                 accel:  u = ub, then linear from ub to -ub, then -ub (the paper's SO-A)
    %                 both:   u = ub, then linear down to 0 as v reaches vb, cruise, mirror image
    %   "FO"        the minimum-energy trapezoid (accelerate, cruise, brake; the paper's FO-3).
    %               Without limits the cruise speed is 3y/(2 rho), E = 27 y^2/(2 rho^3)
    %               (Theorem 5); with limits it is clamped to what they allow (App. E)
    % Limits make transits shorter than minTransit infeasible.
    properties (Constant)
        names = ["constant", "SO", "FO"]
    end

    methods (Static)
        function P = shape(name, y, rho, vb, ub)
            % The profile <name> on an edge of length y crossed in time rho (at least
            % minTransit). P.kind: "cubic" (unconstrained SO), "sat" (SO with u at its limit, no
            % cruise), or "arc": accelerate at a for s, ramp u from a to 0 over d, cruise at v
            % (constant: s = d = 0; SO with the speed limit reached; FO trapezoids: d = 0)
            if nargin < 4, vb = Inf; end
            if nargin < 5, ub = Inf; end
            P = struct('name', name, 'y', y, 'rho', rho, 'kind', "arc", 'a', 0, 's', 0, 'd', 0, 'v', 0);
            switch name
                case "constant"
                    P.v = y / rho;
                case "SO"
                    [~, regime, p] = SpeedProfile.soEnergy(y, rho, vb, ub);
                    switch regime
                        case "none", P.kind = "cubic";
                        case "A",    P.kind = "sat"; P.a = ub; P.s = p;
                        case "V",    P.a = 2 * vb / p; P.d = p; P.v = vb;
                        case "VA",   P.a = ub; P.d = p; P.s = vb / ub - p / 2; P.v = vb;
                    end
                case "FO"
                    [~, vc] = SpeedProfile.foEnergy(y, rho, vb, ub);
                    P.v = vc; P.a = vc^2 / (vc * rho - y); P.s = vc / P.a;
                otherwise
                    error('SpeedProfile:Name', 'Unknown speed profile "%s" (use %s)', name, ...
                        strjoin(SpeedProfile.names, ", "));
            end
        end

        function [l, v, u, e] = at(P, t)
            % Distance l, speed v, tangential acceleration u and energy int_0^t u^2 at time t in
            % [0, rho]; every profile is symmetric about rho/2
            t = min(max(t, 0), P.rho);
            if t <= P.rho / 2
                [l, v, u, e] = SpeedProfile.firstHalf(P, t);
            else
                [l, v, u, e] = SpeedProfile.firstHalf(P, P.rho - t);
                [~, ~, ~, half] = SpeedProfile.firstHalf(P, P.rho / 2);
                l = P.y - l; u = -u; e = 2 * half - e;
            end
        end

        function [vPeak, uPeak] = peaks(P)
            % Largest speed and |acceleration| of the profile (impulses not counted)
            [~, vPeak] = SpeedProfile.at(P, P.rho / 2);
            [~, ~, uPeak] = SpeedProfile.at(P, 0);
            uPeak = abs(uPeak);
        end

        function E = energy(name, y, rho, vb, ub)
            % Energy int u^2 dt over the whole edge, for each transit time in rho (vectorized);
            % NaN for "constant" (undefined)
            if nargin < 4, vb = Inf; end
            if nargin < 5, ub = Inf; end
            switch name
                case "SO",       E = SpeedProfile.soEnergy(y, rho, vb, ub);
                case "FO",       E = SpeedProfile.foEnergy(y, rho, vb, ub);
                case "constant", E = nan(size(rho));
                otherwise,       error('SpeedProfile:Name', 'Unknown speed profile "%s"', name);
            end
        end

        function r = minTransit(y, vb, ub)
            % Shortest transit time the limits allow (accelerate at ub, cruise at vb, brake).
            % Under a speed limit alone the energy grows without bound as rho approaches y/vb,
            % so 0.1% is added there.
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

        function rho = defaultTransit(name, y, vmax)
            % Transit time when no policy plans one: the shortest whose peak speed is vmax
            % (constant: y/vmax; SO and FO peak at 3y/(2 rho): 1.5 y/vmax)
            if name == "constant", rho = y / vmax; else, rho = 1.5 * y / vmax; end
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
end
