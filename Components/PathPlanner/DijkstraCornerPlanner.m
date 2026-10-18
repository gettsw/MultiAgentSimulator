classdef DijkstraCornerPlanner < PathPlanner
    properties
        cornerBuffer double = 1.2
        wallProximity double = 0.6
        smoothingFactor double = 10
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
            rawPath = nodes(pathIndices, :);
            path = obj.smoothPath(rawPath);
            path = obj.clampToBounds(path, bounds);
        end
    end

    methods (Access=protected)
        function smoothed = smoothPath(obj, path)
            if size(path, 1) < 3
                smoothed = path;
                return;
            end
            t = 1:size(path, 1);
            ts = linspace(1, size(path, 1), size(path, 1) * obj.smoothingFactor);
            smoothed = [interp1(t, path(:,1), ts, 'pchip')', ...
                        interp1(t, path(:,2), ts, 'pchip')'];
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
            near = false;
            for i = 1:size(walls, 1)
                w = walls(i, :);
                p1 = [w(1), w(2)]; p2 = [w(3), w(4)];
                v = p2 - p1; w_vec = pt - p1;
                c1 = dot(w_vec, v);
                if c1 <= 0, d = norm(pt - p1);
                else
                    c2 = dot(v, v);
                    if c2 <= c1, d = norm(pt - p2);
                    else, b = c1 / c2; pb = p1 + b * v; d = norm(pt - pb); end
                end
                if d < obj.wallProximity, near = true; return; end
            end
        end

        function blocked = isLineBlocked(obj, p1, p2, walls)
            blocked = false;
            mid = (p1 + p2) / 2;
            if obj.isPointNearWall(mid, walls), blocked = true; return; end
            for i = 1:size(walls, 1)
                w = walls(i, :);
                if obj.intersectSegments(p1, p2, [w(1),w(2)], [w(3),w(4)])
                    blocked = true; return;
                end
            end
        end

        function hit = intersectSegments(obj, a, b, c, d)
            den = (d(2)-c(2))*(b(1)-a(1)) - (d(1)-c(1))*(b(2)-a(2));
            if abs(den) < 1e-10, hit = false; return; end
            ua = ((d(1)-c(1))*(a(2)-c(2)) - (d(2)-c(2))*(a(1)-c(1))) / den;
            ub = ((b(1)-a(1))*(a(2)-c(2)) - (b(2)-a(2))*(a(1)-c(1))) / den;
            tol = obj.endpointTol;
            hit = (ua > tol && ua < 1-tol && ub >= -tol && ub <= 1+tol);
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
