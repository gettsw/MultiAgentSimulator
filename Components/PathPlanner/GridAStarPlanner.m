classdef GridAStarPlanner < PathPlanner
    % A* search on an 8-connected occupancy grid, the standard grid-based planner (Hart,
    % Nilsson and Raphael, 1968). The bounds are covered by square cells of side resolution;
    % a cell is blocked when its center is closer than clearance to a wall (the walls
    % inflated by the robot's clearance). A* with the octile-distance heuristic (admissible
    % and consistent on this grid) finds the shortest grid path; it is then shortened by
    % line-of-sight shortcuts (each point is joined to the farthest later point it can see
    % with the clearance kept), which removes the grid's zig-zag, and its corners are replaced
    % by circular arcs (PathSmoother; radius at most filletRadius, at least filletClearance
    % from the walls). With smooth = false the raw grid path is returned.
    % The grid is built once per set of walls and bounds and reused for every edge.
    properties
        resolution = 0.5    % cell side
        clearance = 1.2     % smallest distance kept from the walls (as DijkstraCornerPlanner)
        smooth = true       % shorten the grid path with line-of-sight shortcuts and round its corners
        filletRadius = 10   % largest corner arc radius
        filletClearance = 0.6   % smallest distance from a corner arc to the walls
        pathSpacing = 0.5   % largest distance between path points
    end

    properties (Access = private)
        cacheKey = []       % walls and bounds the grid was built for
        free                % ny x nx logical, true for free cells
        xs                  % cell center coordinates
        ys
    end

    methods
        function path = plan(obj, pStart, pEnd, walls, bounds)
            if nargin < 5 || isempty(bounds), bounds = [0 100 0 100]; end
            obj.buildGrid(walls, bounds);
            s = obj.nearestFree(pStart);
            g = obj.nearestFree(pEnd);
            cells = obj.astar(s, g);
            obj.noRoute = isempty(cells);
            if obj.noRoute                                   % no route: straight line
                warning('PathPlanner:NoRoute', 'No wall-free route from [%g %g] to [%g %g]; using the straight line.', ...
                    pStart(1), pStart(2), pEnd(1), pEnd(2));
                path = PathSmoother.sample([pStart; pEnd], obj.pathSpacing);
                return;
            end
            [r, c] = ind2sub(size(obj.free), cells);
            path = [pStart; obj.xs(c)', obj.ys(r)'; pEnd];
            path = path([true; any(diff(path, 1, 1) ~= 0, 2)], :);   % drop repeated points
            if obj.smooth
                path = obj.shortcut(path, walls);
                path = PathSmoother.fillet(path, walls, obj.filletClearance, obj.filletRadius, obj.pathSpacing);
            end
        end
    end

    methods (Access = private)
        function buildGrid(obj, walls, bounds)
            key = [bounds(:); walls(:); obj.resolution; obj.clearance];
            if isequal(key, obj.cacheKey), return; end
            h = obj.resolution;
            obj.xs = bounds(1):h:bounds(2);
            obj.ys = bounds(3):h:bounds(4);
            [X, Y] = meshgrid(obj.xs, obj.ys);
            obj.free = true(size(X));
            for w = 1:size(walls, 1)                         % distance of every cell to wall w
                d = GridAStarPlanner.distToSegment(X, Y, walls(w, 1:2), walls(w, 3:4));
                obj.free(d < obj.clearance) = false;
            end
            obj.cacheKey = key;
        end

        function k = nearestFree(obj, p)
            % Index of the free cell nearest p
            [X, Y] = meshgrid(obj.xs, obj.ys);
            d = (X - p(1)).^2 + (Y - p(2)).^2;
            d(~obj.free) = Inf;
            [~, k] = min(d(:));
        end

        function cells = astar(obj, s, g)
            % Linear indices of the cells of a shortest 8-connected path from s to g ([] if none)
            [ny, nx] = size(obj.free);
            h = obj.resolution;
            [gr, gc] = ind2sub([ny nx], g);
            heur = @(r, c) h * (max(abs(r - gr), abs(c - gc)) + (sqrt(2) - 1) * min(abs(r - gr), abs(c - gc)));
            cost = inf(ny * nx, 1); cost(s) = 0;
            prev = zeros(ny * nx, 1);
            closed = false(ny * nx, 1);
            dr = [-1 -1 -1 0 0 1 1 1]; dc = [-1 0 1 -1 1 -1 0 1];
            step = h * sqrt(dr.^2 + dc.^2);
            [sr, sc] = ind2sub([ny nx], s);
            heap = zeros(ny * nx, 2); n = 1; heap(1, :) = [heur(sr, sc), s];   % [f, cell]
            while n > 0
                [f, u] = deal(heap(1, 1), heap(1, 2));        % pop the smallest f
                heap(1, :) = heap(n, :); n = n - 1;
                i = 1;
                while true                                    % sift down
                    l = 2 * i; m = i;
                    if l <= n && heap(l, 1) < heap(m, 1), m = l; end
                    if l + 1 <= n && heap(l + 1, 1) < heap(m, 1), m = l + 1; end
                    if m == i, break; end
                    heap([i m], :) = heap([m i], :); i = m;
                end
                if closed(u), continue; end
                closed(u) = true;
                if u == g, break; end
                [r, c] = ind2sub([ny nx], u);
                for k = 1:8
                    rr = r + dr(k); cc = c + dc(k);
                    if rr < 1 || rr > ny || cc < 1 || cc > nx, continue; end
                    v = rr + (cc - 1) * ny;
                    if ~obj.free(v) || closed(v), continue; end
                    if dr(k) ~= 0 && dc(k) ~= 0 && ~(obj.free(r + (cc - 1) * ny) && obj.free(rr + (c - 1) * ny))
                        continue;                             % no cutting past a blocked corner
                    end
                    alt = cost(u) + step(k);
                    if alt < cost(v)
                        cost(v) = alt; prev(v) = u;
                        n = n + 1; heap(n, :) = [alt + heur(rr, cc), v];
                        i = n;
                        while i > 1                           % sift up
                            p = floor(i / 2);
                            if heap(p, 1) <= heap(i, 1), break; end
                            heap([i p], :) = heap([p i], :); i = p;
                        end
                    end
                end
            end
            cells = [];
            if ~closed(g), return; end
            u = g;
            while u ~= 0
                cells = [u; cells]; %#ok<AGROW>
                u = prev(u);
            end
        end

        function out = shortcut(obj, path, walls)
            % Greedy line-of-sight shortening: from each kept point, jump to the farthest later
            % point whose segment keeps the clearance from every wall
            out = path(1, :);
            i = 1;
            n = size(path, 1);
            while i < n
                j = n;
                while j > i + 1 && ~obj.visible(path(i, :), path(j, :), walls)
                    j = j - 1;
                end
                out(end + 1, :) = path(j, :); %#ok<AGROW>
                i = j;
            end
        end

        function ok = visible(obj, a, b, walls)
            % Does segment ab keep the clearance from every wall? (checked at points spaced
            % resolution/2 apart; the end points are exempt, as targets may sit near walls)
            ok = true;
            if isempty(walls), return; end
            n = max(2, ceil(norm(b - a) / (obj.resolution / 2)) + 1);
            t = linspace(0, 1, n)'; t = t(2:end-1);
            if isempty(t), return; end
            P = a + t .* (b - a);
            for w = 1:size(walls, 1)
                d = GridAStarPlanner.distToSegment(P(:, 1), P(:, 2), walls(w, 1:2), walls(w, 3:4));
                if any(d < obj.clearance * 0.999), ok = false; return; end
            end
        end
    end

    methods (Static, Access = private)
        function d = distToSegment(X, Y, p1, p2)
            % Distance of points (X, Y) to the segment p1-p2
            v = p2 - p1;
            t = ((X - p1(1)) * v(1) + (Y - p1(2)) * v(2)) / max(v * v', eps);
            t = min(max(t, 0), 1);
            d = hypot(X - (p1(1) + t * v(1)), Y - (p1(2) + t * v(2)));
        end
    end
end
