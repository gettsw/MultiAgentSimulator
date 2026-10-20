classdef SecondOrderAgent < EnergyAgent
    % The energy-aware agent of S. Welikala and C. G. Cassandras, "Event-driven receding horizon
    % control of energy-aware dynamic agents for distributed persistent monitoring", arXiv
    % 2102.12963 (IEEE CDC 2021): a second-order unicycle whose tangential acceleration u is the
    % control. It follows its path by arc length (the heading follows the path, Theorem 2),
    % starts and stops at rest on the targets and stays still while dwelling. On an edge of
    % length y crossed in time rho it uses the minimum-energy speed profile of its class:
    %   "SO"    v(t) = 6y t (rho - t)/rho^3 (Theorem 3), energy 12 y^2/rho^3
    %   "FO"    trapezoid: accelerate for rho/3, cruise at 3y/(2 rho), brake for rho/3 (the
    %           paper's FO-3, Theorem 5), energy 27 y^2/(2 rho^3)
    %   "SO-V"  SO under the speed bound maxSpeed (App. D): when the SO peak 3y/(2 rho) exceeds
    %           it, u falls linearly to 0 as v reaches the bound, cruise, mirror image; energy
    %           16 vb^3/(9 (vb rho - y)). (The paper's SO-V cuts the SO parabola at the bound,
    %           so u jumps there; this profile keeps u continuous and uses less energy.)
    %   "SO-A"  SO under the acceleration bound maxAccel (App. D): u = ub, then linear from ub
    %           to -ub, then -ub; energy ub^2 (3 rho - 2 sqrt(3 rho^2 - 12 y/ub))/3
    %   "FO-V"  trapezoid cruising at maxSpeed (App. E), energy 2 vb^3/(vb rho - y)
    %   "FO-A"  trapezoid accelerating at maxAccel (App. E), energy ub^2 (rho - sqrt(rho^2 - 4y/ub))
    % A bounded profile equals its unbounded one while the bound is not reached.
    % A policy sets transitTime and transitProfile before the agent leaves a target (as the RHC
    % policies set dwellRemaining). Without them the agent uses the SO profile with peak speed
    % maxSpeed. Je accumulates the paper's energy objective int u(t)^2 dt exactly (steering is
    % not counted). The battery (e_total, soc) uses the EnergyAgent power model with the
    % tangential acceleration; it is reported only and never stops the agent.
    properties
        transitTime = NaN               % transit time of the next edge [s] (NaN: SO with peak speed maxSpeed)
        transitProfile string = "SO"    % speed profile of the next edge (see above)
        Je = 0                          % energy objective int u(t)^2 dt so far
        peakSpeed = 0                   % largest tangential speed so far
        peakAccel = 0                   % largest |tangential acceleration| so far
    end

    properties (Access = private)
        leg = []   % the edge in progress: struct(pts, s, P (profile), tau, e)
    end

    properties (Constant)
        profiles = ["SO", "SO-V", "SO-A", "FO", "FO-V", "FO-A"]
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
        function P = shape(name, y, rho, vb, ub)
            % Parameters of the minimum-energy profile <name> on an edge of length y crossed in
            % time rho, with speed bound vb and acceleration bound ub (used by the -V and -A
            % profiles). P.kind: "cubic" (SO), "ramp" (SO-V, bound reached), "sat" (SO-A, bound
            % reached), "trap" (FO family)
            P = struct('name', name, 'y', y, 'rho', rho, 'kind', "cubic", 'a', 0, 't1', 0, 'v', 0);
            switch name
                case "SO"
                case "SO-V"
                    if 3 * y / (2 * rho) > vb
                        t1 = 3 * (vb * rho - y) / (2 * vb);
                        P.kind = "ramp"; P.t1 = t1; P.a = 2 * vb / t1; P.v = vb;
                    end
                case "SO-A"
                    if 6 * y / rho^2 > ub
                        s = (rho - sqrt(max(3 * rho^2 - 12 * y / ub, 0))) / 2;
                        P.kind = "sat"; P.t1 = s; P.a = ub;
                    end
                case {"FO", "FO-V", "FO-A"}
                    P.kind = "trap";
                    ta = rho / 3; uA = 9 * y / (2 * rho^2); vc = 3 * y / (2 * rho);
                    if name == "FO-V" && vc > vb
                        vc = vb; ta = rho - y / vb; uA = vb / ta;
                    elseif name == "FO-A" && uA > ub
                        uA = ub; vc = rho * ub / 2 * (1 - sqrt(max(1 - 4 * y / (ub * rho^2), 0))); ta = vc / ub;
                    end
                    P.a = uA; P.t1 = ta; P.v = vc;
                otherwise
                    error('SecondOrderAgent:Profile', 'Unknown speed profile "%s"', name);
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

        function E = transitEnergy(name, y, rho, vb, ub)
            % Energy int u^2 dt of profile <name> over the whole edge, for each transit time in rho
            if startsWith(name, "SO"), E = 12 * y^2 ./ rho.^3; else, E = 27 * y^2 ./ (2 * rho.^3); end
            switch name
                case "SO-V"
                    k = 3 * y ./ (2 * rho) > vb;
                    E(k) = 16 * vb^3 ./ (9 * (vb * rho(k) - y));
                case "SO-A"
                    k = 6 * y ./ rho.^2 > ub;
                    E(k) = ub^2 * (3 * rho(k) - 2 * sqrt(max(3 * rho(k).^2 - 12 * y / ub, 0))) / 3;
                case "FO-V"
                    k = 3 * y ./ (2 * rho) > vb;
                    E(k) = 2 * vb^3 ./ (vb * rho(k) - y);
                case "FO-A"
                    k = 9 * y ./ (2 * rho.^2) > ub;
                    E(k) = ub^2 * (rho(k) - sqrt(max(rho(k).^2 - 4 * y / ub, 0)));
            end
        end

        function r = minTransit(name, y, vb, ub)
            % Shortest feasible transit time of profile <name> (0: none). The speed-bounded
            % profiles need rho > y/vb (their energy grows without bound there).
            switch name
                case {"SO-V", "FO-V"}, r = 1.001 * y / vb;
                case {"SO-A", "FO-A"}, r = 2 * sqrt(y / ub);
                otherwise,             r = 0;
            end
        end
    end

    methods (Static, Access = private)
        function [l, v, u, e] = firstHalf(P, t)
            y = P.y; rho = P.rho;
            switch P.kind
                case "cubic"
                    s = t / rho;
                    l = y * (3 * s^2 - 2 * s^3);
                    v = 6 * y * t * (rho - t) / rho^3;
                    u = 6 * y * (rho - 2 * t) / rho^3;
                    e = 6 * y^2 / rho^6 * (rho^3 - (rho - 2 * t)^3);
                case "ramp"     % u = a (1 - t/t1) until v = vb at t1, then cruise
                    a = P.a; t1 = P.t1;
                    if t <= t1
                        u = a * (1 - t / t1);
                        v = a * (t - t^2 / (2 * t1));
                        l = a * (t^2 / 2 - t^3 / (6 * t1));
                        e = a^2 * t1 / 3 * (1 - (1 - t / t1)^3);
                    else
                        u = 0; v = P.v;
                        l = 2 * P.v * t1 / 3 + P.v * (t - t1);
                        e = a^2 * t1 / 3;
                    end
                case "sat"      % u = ub until t1, then linear to -ub at rho - t1
                    ub = P.a; s = P.t1; m = rho - 2 * s;
                    if t <= s
                        u = ub; v = ub * t; l = ub * t^2 / 2; e = ub^2 * t;
                    else
                        tau = t - s;
                        u = ub * (1 - 2 * tau / m);
                        v = ub * (s + tau - tau^2 / m);
                        l = ub * s^2 / 2 + ub * s * tau + ub * (tau^2 / 2 - tau^3 / (3 * m));
                        e = ub^2 * s + ub^2 * m / 6 * (1 - (1 - 2 * tau / m)^3);
                    end
                case "trap"     % u = a until t1, then cruise at v
                    a = P.a; ta = P.t1;
                    if t <= ta
                        u = a; v = a * t; l = a * t^2 / 2; e = a^2 * t;
                    else
                        u = 0; v = P.v; l = a * ta^2 / 2 + P.v * (t - ta); e = a^2 * ta;
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
            name = obj.transitProfile;
            vb = obj.state.maxSpeed; ub = obj.state.maxAccel;
            rho = obj.transitTime;
            if ~(rho > 0), name = "SO"; rho = 1.5 * y / vb; end      % default: SO, peak speed = maxSpeed
            rho = max(rho, SecondOrderAgent.minTransit(name, y, vb, ub));
            P = SecondOrderAgent.shape(name, y, rho, vb, ub);
            obj.leg = struct('pts', pts, 's', s, 'P', P, 'tau', 0, 'e', 0);
            [~, vPeak] = SecondOrderAgent.profileAt(P, rho / 2);
            [~, ~, uPeak] = SecondOrderAgent.profileAt(P, 0);
            obj.peakSpeed = max(obj.peakSpeed, vPeak);
            obj.peakAccel = max(obj.peakAccel, abs(uPeak));
        end
    end
end
