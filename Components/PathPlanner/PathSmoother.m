classdef PathSmoother
    % Smooths a planner's polyline by replacing each corner with a circular arc (a fillet)
    % tangent to both segments, so the heading is continuous and the curvature is bounded by
    % 1/r on each arc. Each arc is as large as the segments allow (its tangent points stay on
    % the segments, sharing an inner segment equally with the next corner), at most rMax, and
    % shrunk until every point of it keeps the clearance from every wall (or 90% of the
    % corner's own distance to the walls, if the corner is closer than that). A corner whose arc
    % cannot keep it even when small stays a corner. The result is sampled at most
    % spacing apart; if it crossed a wall anyway, the polyline is returned unchanged.
    % Used by the planners (DijkstraCornerPlanner, EnergyEfficientPlanner, GridAStarPlanner).
    methods (Static)
        function P = fillet(path, walls, clearance, rMax, spacing)
            if nargin < 5, spacing = 0.5; end
            n = size(path, 1);
            if n < 3
                P = PathSmoother.sample(path, spacing);
                return;
            end
            seg = vecnorm(diff(path, 1, 1), 2, 2);
            share = 0.5 * ones(n - 1, 1); share([1 end]) = 1;     % end segments: no other corner
            P = path(1, :);
            for k = 2:n-1
                a = (path(k, :) - path(k - 1, :)) / seg(k - 1);
                b = (path(k + 1, :) - path(k, :)) / seg(k);
                th = acos(max(-1, min(1, a * b')));                 % turn angle
                if th < 1e-6 || th > pi - 1e-6
                    P = [P; PathSmoother.sample([P(end, :); path(k, :)], spacing, true)]; %#ok<AGROW>
                    continue;
                end
                tmax = min(seg(k - 1) * share(k - 1), seg(k) * share(k));
                r = min(rMax, tmax / tan(th / 2));
                c = min(clearance, 0.9 * PathSmoother.wallDistance(path(k, :), walls));
                arc = [];
                while r > 0.02
                    arc = PathSmoother.arc(path(k, :), a, b, th, r, spacing);
                    if PathSmoother.clear(arc, walls, c), break; end
                    r = 0.7 * r; arc = [];
                end
                if isempty(arc)
                    corner = path(k, :);                            % keep the corner
                    P = [P; PathSmoother.sample([P(end, :); corner], spacing, true)]; %#ok<AGROW>
                else
                    P = [P; PathSmoother.sample([P(end, :); arc(1, :)], spacing, true); arc(2:end, :)]; %#ok<AGROW>
                end
            end
            P = [P; PathSmoother.sample([P(end, :); path(end, :)], spacing, true)];
            P = P([true; any(abs(diff(P, 1, 1)) > 1e-12, 2)], :);
            if PathSmoother.crosses(P, walls)                       % never worse than the input
                P = PathSmoother.sample(path, spacing);
            end
        end

        function P = sample(path, spacing, dropFirst)
            % Points along the polyline at most spacing apart (the input points included)
            if nargin < 3, dropFirst = false; end
            P = path(1, :);
            for i = 1:size(path, 1) - 1
                L = norm(path(i + 1, :) - path(i, :));
                m = max(1, ceil(L / spacing));
                t = (1:m)' / m;
                P = [P; path(i, :) + t .* (path(i + 1, :) - path(i, :))]; %#ok<AGROW>
            end
            if dropFirst, P = P(2:end, :); end
        end
    end

    methods (Static, Access = private)
        function A = arc(V, a, b, th, r, spacing)
            % The fillet of radius r at corner V between directions a (in) and b (out)
            t = r * tan(th / 2);
            T1 = V - t * a; T2 = V + t * b;
            bis = (b - a) / norm(b - a);                            % toward the inside of the turn
            C = V + bis * r / cos(th / 2);
            phi1 = atan2(T1(2) - C(2), T1(1) - C(1));
            phi2 = atan2(T2(2) - C(2), T2(1) - C(1));
            dphi = angle(exp(1i * (phi2 - phi1)));                  % the short way round
            m = max([2, ceil(abs(dphi) * r / spacing) + 1, ceil(abs(dphi) / 0.2) + 1]);   % <= 0.2 rad apart
            phi = phi1 + linspace(0, dphi, m)';
            A = C + r * [cos(phi), sin(phi)];
        end

        function d = wallDistance(p, walls)
            % Distance from point p to the nearest wall (Inf without walls)
            d = Inf;
            for w = 1:size(walls, 1)
                p1 = walls(w, 1:2); v = walls(w, 3:4) - p1;
                s = min(max(((p(1) - p1(1)) * v(1) + (p(2) - p1(2)) * v(2)) / max(v * v', eps), 0), 1);
                d = min(d, hypot(p(1) - p1(1) - s * v(1), p(2) - p1(2) - s * v(2)));
            end
        end

        function ok = clear(A, walls, clearance)
            ok = true;
            for w = 1:size(walls, 1)
                p1 = walls(w, 1:2); v = walls(w, 3:4) - p1;
                s = ((A(:, 1) - p1(1)) * v(1) + (A(:, 2) - p1(2)) * v(2)) / max(v * v', eps);
                s = min(max(s, 0), 1);
                d = hypot(A(:, 1) - (p1(1) + s * v(1)), A(:, 2) - (p1(2) + s * v(2)));
                if any(d < clearance), ok = false; return; end
            end
        end

        function hit = crosses(P, walls)
            % Does any segment of P cross a wall?
            hit = false;
            if isempty(walls), return; end
            A = P(1:end-1, :); B = P(2:end, :);
            for w = 1:size(walls, 1)
                c = walls(w, 1:2); d = walls(w, 3:4);
                den = (d(2) - c(2)) * (B(:, 1) - A(:, 1)) - (d(1) - c(1)) * (B(:, 2) - A(:, 2));
                ua = ((d(1) - c(1)) * (A(:, 2) - c(2)) - (d(2) - c(2)) * (A(:, 1) - c(1))) ./ den;
                ub = ((B(:, 1) - A(:, 1)) .* (A(:, 2) - c(2)) - (B(:, 2) - A(:, 2)) .* (A(:, 1) - c(1))) ./ den;
                if any(abs(den) > 1e-12 & ua > 1e-9 & ua < 1 - 1e-9 & ub > 1e-9 & ub < 1 - 1e-9)
                    hit = true; return;
                end
            end
        end
    end
end
