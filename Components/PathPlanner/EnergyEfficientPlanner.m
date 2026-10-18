classdef EnergyEfficientPlanner < DijkstraCornerPlanner
    % Minimizes distance + turning penalty so paths favor gentler bends,
    % reducing the acceleration-squared term in EnergyAgent power draw.
    properties
        turnPenalty double = 8.0   % cost per radian of heading change (same units as distance)
    end

    methods
        function obj = EnergyEfficientPlanner()
            obj.cornerBuffer = 2.5;
            obj.wallProximity = 1.0;
            obj.smoothingFactor = 20;
        end

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
            pathIndices = obj.energyDijkstra(nodes, adj, 1, 2);
            rawPath = nodes(pathIndices, :);
            path = obj.smoothPath(rawPath);
            path = obj.clampToBounds(path, bounds);
        end
    end

    methods (Access=protected)
        function idxs = energyDijkstra(obj, nodes, adj, startNode, endNode)
            % State: (prev+1, curr). prev=0 is the virtual start.
            n = size(adj, 1);
            dist = inf(n+1, n);
            parentPrev = zeros(n+1, n);
            dist(1, startNode) = 0;
            visited = false(n+1, n);
            best_r = 0;

            while true
                tmp = dist; tmp(visited) = inf;
                [mn, linIdx] = min(tmp(:));
                if isinf(mn), break; end
                [r, c] = ind2sub([n+1, n], linIdx);
                visited(r, c) = true;

                if c == endNode
                    best_r = r;
                    break;
                end

                p = r - 1;
                for next = 1:n
                    if isinf(adj(c, next)), continue; end
                    if p == 0
                        turnCost = 0;
                    else
                        inDir = nodes(c, :) - nodes(p, :);
                        inDir = inDir / (norm(inDir) + eps);
                        outDir = nodes(next, :) - nodes(c, :);
                        outDir = outDir / (norm(outDir) + eps);
                        cosA = max(-1, min(1, dot(inDir, outDir)));
                        turnCost = obj.turnPenalty * acos(cosA);
                    end
                    alt = dist(r, c) + adj(c, next) + turnCost;
                    new_r = c + 1;
                    if alt < dist(new_r, next)
                        dist(new_r, next) = alt;
                        parentPrev(new_r, next) = p;
                    end
                end
            end

            if best_r == 0
                idxs = [startNode, endNode];
                return;
            end

            idxs = endNode;
            r = best_r; c = endNode;
            while true
                p = r - 1;
                if p == 0, break; end
                idxs = [p, idxs];
                pp = parentPrev(r, c);
                r = pp + 1; c = p;
            end
        end
    end
end
