classdef DijkstraCornerPlanner < PathPlanner
    % Shortest path over a visibility graph: the nodes are the start, the end, and the wall end
    % points pushed out by cornerBuffer (dropped when closer than wallProximity to a wall); two
    % nodes are joined when the segment between them crosses no wall. Dijkstra's algorithm finds
    % the shortest route, and PathSmoother replaces each corner with the largest circular arc
    % that keeps filletClearance from the walls (radius at most filletRadius), so the heading
    % is continuous and the curvature bounded.
    properties
        cornerBuffer double = 1.2
        wallProximity double = 0.6
        filletRadius double = 10      % largest corner arc radius
        filletClearance double = 0.6  % smallest distance from a corner arc to the walls
        pathSpacing double = 0.5      % largest distance between path points
        endpointTol double = 1e-6
    end

    methods
        function path = plan(obj, pStart, pEnd, walls, bounds)
            if nargin < 5 || isempty(bounds), bounds = []; end
            nodes = [pStart; pEnd];
            for i = 1:size(walls, 1)
                w = walls(i, :);
                p1 = [w(1), w(2)]; p2 = [w(3), w(4)];
                nodes = [nodes; obj.getBufferedCorners(p1, p2, walls, bounds)];
            end
            nNodes = size(nodes, 1);
            adj = inf(nNodes, nNodes);
            for i = 1:nNodes
                for j = i+1:nNodes
                    if ~obj.isLineBlocked(nodes(i,:), nodes(j,:), walls)
                        d = norm(nodes(i,:) - nodes(j,:));
                        adj(i,j) = d; adj(j,i) = d;
                    end
                end
            end
            pathIndices = obj.dijkstra(adj, 1, 2);
            path = obj.finishPath(nodes, pathIndices, walls, bounds);
        end
    end

    methods (Access=protected)
        function path = finishPath(obj, nodes, idxs, walls, bounds)
            % The route through nodes(idxs), smoothed and kept in bounds; the straight line
            % (and noRoute) when the end is unreachable
            obj.noRoute = numel(idxs) < 2 || idxs(1) ~= 1;
            if obj.noRoute
                warning('PathPlanner:NoRoute', 'No wall-free route from [%g %g] to [%g %g]; using the straight line.', ...
                    nodes(1, 1), nodes(1, 2), nodes(2, 1), nodes(2, 2));
                path = PathSmoother.sample(nodes([1 2], :), obj.pathSpacing);
                return;
            end
            path = obj.smoothPath(nodes(idxs, :), walls);
            path = obj.clampToBounds(path, bounds);
        end

        function smoothed = smoothPath(obj, path, walls)
            smoothed = PathSmoother.fillet(path, walls, obj.filletClearance, obj.filletRadius, obj.pathSpacing);
        end

        function clipped = clampToBounds(~, path, bounds)
            if isempty(bounds) || isempty(path)
                clipped = path;
                return;
            end
            xMin = bounds(1); xMax = bounds(2); yMin = bounds(3); yMax = bounds(4);
            clipped = path;
            clipped(:,1) = min(max(clipped(:,1), xMin), xMax);
            clipped(:,2) = min(max(clipped(:,2), yMin), yMax);
        end

        function pts = getBufferedCorners(obj, p1, p2, walls, bounds)
            dist = obj.cornerBuffer;
            dir = (p2 - p1) / (norm(p2 - p1) + eps);
            perp = [-dir(2), dir(1)];
            offsets = {dir*dist+perp*dist, dir*dist-perp*dist, -dir*dist+perp*dist, -dir*dist-perp*dist};
            pts = [p1 + offsets{3}; p1 + offsets{4}; p2 + offsets{1}; p2 + offsets{2}];
            valid = false(size(pts,1),1);
            for i = 1:size(pts,1)
                if obj.isPointNearWall(pts(i,:), walls), continue; end
                if ~obj.isPointInBounds(pts(i,:), bounds), continue; end
                valid(i) = true;
            end
            pts = pts(valid, :);
        end

        function inside = isPointInBounds(~, pt, bounds)
            if isempty(bounds)
                inside = true; return;
            end
            inside = pt(1) >= bounds(1) && pt(1) <= bounds(2) && ...
                     pt(2) >= bounds(3) && pt(2) <= bounds(4);
        end

        function near = isPointNearWall(obj, pt, walls)
            % Is pt closer than wallProximity to any wall? (all walls at once)
            if isempty(walls), near = false; return; end
            P1 = walls(:, 1:2); V = walls(:, 3:4) - P1;
            c1 = sum((pt - P1) .* V, 2);
            c2 = sum(V .* V, 2);
            b = min(max(c1 ./ c2, 0), 1);          % closest point: p1 (b = 0), p2 (b = 1) or between
            d = vecnorm(pt - (P1 + b .* V), 2, 2);
            near = any(d < obj.wallProximity);
        end

        function blocked = isLineBlocked(obj, p1, p2, walls)
            blocked = false;
            if isempty(walls), return; end
            mid = (p1 + p2) / 2;
            if obj.isPointNearWall(mid, walls), blocked = true; return; end
            blocked = any(obj.intersectSegments(p1, p2, walls(:, 1:2), walls(:, 3:4)));
        end

        function hit = intersectSegments(obj, a, b, c, d)
            % Does segment ab cross segment cd? c and d may hold one segment per row
            den = (d(:,2)-c(:,2))*(b(1)-a(1)) - (d(:,1)-c(:,1))*(b(2)-a(2));
            ua = ((d(:,1)-c(:,1)).*(a(2)-c(:,2)) - (d(:,2)-c(:,2)).*(a(1)-c(:,1))) ./ den;
            ub = ((b(1)-a(1))*(a(2)-c(:,2)) - (b(2)-a(2))*(a(1)-c(:,1))) ./ den;
            tol = obj.endpointTol;
            hit = abs(den) >= 1e-10 & ua > tol & ua < 1-tol & ub >= -tol & ub <= 1+tol;
        end

        function idxs = dijkstra(~, adj, startNode, endNode)
            n = size(adj, 1);
            dist = inf(1, n); prev = zeros(1, n);
            dist(startNode) = 0;
            Q = 1:n;
            while ~isempty(Q)
                [~, q_idx] = min(dist(Q));
                u = Q(q_idx);
                if u == endNode || isinf(dist(u)), break; end
                Q(q_idx) = [];
                for v = 1:n
                    if isinf(adj(u,v)), continue; end
                    alt = dist(u) + adj(u,v);
                    if alt < dist(v), dist(v) = alt; prev(v) = u; end
                end
            end
            idxs = []; curr = endNode;
            while curr ~= 0
                idxs = [curr, idxs]; curr = prev(curr);
            end
        end
    end
end
