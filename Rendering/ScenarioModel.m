classdef ScenarioModel < handle
    properties
        agents = []
        targets = Target.empty
        edges = Edge.empty
        walls = [] 
        cumUncertaintyIntegral = 0
        lastLogTime = 0
        lastUncertainty = 0
        hasLastSample = false
        policyMap
    end

    methods
        function obj = ScenarioModel()
            obj.policyMap = containers.Map('KeyType','char','ValueType','any');
            obj.policyMap('Default') = RandomWalkPolicy();
            obj.policyMap('Energy') = BatteryEfficientPolicy();
        end

        function addWall(obj, p1, p2)
            obj.walls = [obj.walls; p1(1), p1(2), p2(1), p2(2)];
        end

        function replanAllEdges(obj)
            for k = 1:numel(obj.edges)
                e = obj.edges(k);
                pStart = e.targets(1).position;
                pEnd = e.targets(2).position;
                e.curvePoints = obj.planShortestPath(pStart, pEnd);
                if ~isempty(e.lineHandle) && isgraphics(e.lineHandle)
                    delete(e.lineHandle);
                    e.lineHandle = [];
                end
            end
        end

        function step(obj, dtSim, simTime)
            if isempty(obj.agents), return; end
            for k = 1:numel(obj.agents)
                a = obj.agents(k);
                if isprop(a, 'dwellRemaining') && a.dwellRemaining > 0
                    a.dwellRemaining = max(0, a.dwellRemaining - dtSim);
                    continue;
                end
                if isempty(a.path)
                    pol = obj.getPolicyForAgent(a);
                    adj = obj.buildAdjacency();
                    cmd = pol.plan(a, obj, adj, a.current_target_idx, simTime);
                    if ~isempty(cmd) && strcmp(cmd.kind, "move")
                        edge = obj.findEdge(a.current_target_idx, cmd.targetIdx);
                        if ~isempty(edge)
                            a.path = edge.curvePoints;
                            if norm(a.path(1,:) - a.state.pos) > norm(a.path(end,:) - a.state.pos)
                                a.path = flipud(a.path);
                            end
                            a.pathIndex = 1;
                            a.current_target_idx = cmd.targetIdx;
                        end
                    end
                end
                a.update(dtSim);
            end
        end

        function [e, ok, msg] = addEdgeByTargets(obj, t1Idx, t2Idx)
            e = Edge.empty; ok = false; msg = "";
            pStart = obj.targets(t1Idx).position;
            pEnd = obj.targets(t2Idx).position;
            pathPoints = obj.planShortestPath(pStart, pEnd);
            idx = numel(obj.edges) + 1;
            e = Edge(idx, [obj.targets(t1Idx), obj.targets(t2Idx)], pathPoints);
            obj.edges(end+1) = e;
            ok = true;
        end

        function path = planShortestPath(obj, pStart, pEnd)
            nodes = [pStart; pEnd];
            buffer = 1.2; 
            for i = 1:size(obj.walls, 1)
                w = obj.walls(i, :);
                p1 = [w(1),w(2)]; p2 = [w(3),w(4)];
                nodes = [nodes; obj.getBufferedCorners(p1, p2, buffer)];
            end
            nNodes = size(nodes, 1);
            adj = inf(nNodes, nNodes);
            for i = 1:nNodes
                for j = i+1:nNodes
                    if ~obj.isLineBlocked(nodes(i,:), nodes(j,:))
                        d = norm(nodes(i,:) - nodes(j,:));
                        adj(i,j) = d; adj(j,i) = d;
                    end
                end
            end
            pathIndices = obj.dijkstra(adj, 1, 2);
            rawPath = nodes(pathIndices, :);
            path = obj.smoothPath(rawPath);
        end

        function smoothed = smoothPath(obj, path)
            if size(path, 1) < 3
                smoothed = path;
                return;
            end
            t = 1:size(path, 1);
            ts = linspace(1, size(path, 1), size(path, 1) * 10);
            smoothed = [interp1(t, path(:,1), ts, 'pchip')', ...
                        interp1(t, path(:,2), ts, 'pchip')'];
        end

        function pts = getBufferedCorners(obj, p1, p2, dist)
            dir = (p2 - p1) / (norm(p2 - p1) + eps);
            perp = [-dir(2), dir(1)];
            offsets = {dir*dist+perp*dist, dir*dist-perp*dist, -dir*dist+perp*dist, -dir*dist-perp*dist};
            pts = [p1 + offsets{3}; p1 + offsets{4}; p2 + offsets{1}; p2 + offsets{2}];
            valid = false(size(pts,1),1);
            for i = 1:size(pts,1)
                if ~obj.isPointNearWall(pts(i,:)), valid(i) = true; end
            end
            pts = pts(valid, :);
        end

        function near = isPointNearWall(obj, pt)
            near = false;
            for i = 1:size(obj.walls, 1)
                w = obj.walls(i, :);
                p1 = [w(1), w(2)]; p2 = [w(3), w(4)];
                v = p2 - p1; w_vec = pt - p1;
                c1 = dot(w_vec, v);
                if c1 <= 0, d = norm(pt - p1);
                else
                    c2 = dot(v, v);
                    if c2 <= c1, d = norm(pt - p2);
                    else, b = c1 / c2; pb = p1 + b * v; d = norm(pt - pb); end
                end
                if d < 0.6, near = true; return; end
            end
        end

        function blocked = isLineBlocked(obj, p1, p2)
            blocked = false;
            mid = (p1 + p2) / 2;
            if obj.isPointNearWall(mid), blocked = true; return; end
            for i = 1:size(obj.walls, 1)
                w = obj.walls(i, :);
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
            hit = (ua > 0.005 && ua < 0.995 && ub > 0.005 && ub < 0.995);
        end

        function idxs = dijkstra(obj, adj, startNode, endNode)
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

        function [uNow, JNow] = updateTargetsAndLogObjective(obj, simTime, dtSim)
            uNow = 0; detectionRadius = 1.2; 
            for t = 1:numel(obj.targets)
                nearby = [];
                for a_idx = 1:numel(obj.agents)
                    a = obj.agents(a_idx);
                    distToTarget = norm(a.state.pos - obj.targets(t).position);
                    if distToTarget < detectionRadius
                        if isempty(nearby), nearby = a; else, nearby(end+1) = a; end
                    end
                end
                obj.targets(t).updateResidingAgents(nearby, simTime);
                obj.targets(t).updateUncertainty(dtSim);
                uNow = uNow + obj.targets(t).R;
            end
            if ~obj.hasLastSample
                obj.lastLogTime = simTime; obj.lastUncertainty = uNow; obj.hasLastSample = true;
            else
                dt = simTime - obj.lastLogTime;
                if dt > 0
                    obj.cumUncertaintyIntegral = obj.cumUncertaintyIntegral + 0.5 * (obj.lastUncertainty + uNow) * dt;
                    obj.lastLogTime = simTime; obj.lastUncertainty = uNow;
                end
            end
            JNow = obj.cumUncertaintyIntegral / max(eps, simTime);
        end

        function s = exportLayout(obj)
            s.targets = reshape([obj.targets.position], 2, []).';
            s.edges = [];
            for k = 1:numel(obj.edges)
                ids = [obj.edges(k).targets(1).index, obj.edges(k).targets(2).index];
                s.edges = [s.edges; ids];
            end
            s.walls = obj.walls;
        end

        function importLayout(obj, s)
            obj.clearAll();
            for k = 1:size(s.targets, 1)
                obj.addTarget(s.targets(k,:));
            end
            for k = 1:size(s.edges, 1)
                obj.addEdgeByTargets(double(s.edges(k,1)), double(s.edges(k,2)));
            end
            if isfield(s, 'walls'), obj.walls = s.walls; end
        end

        function t = addTarget(obj, pos)
            t = Target(numel(obj.targets) + 1, pos);
            obj.targets(end+1) = t;
        end

        function [a, ok, msg] = addAgentOnTarget(obj, clickPos, speed, tol, type)
            a = []; ok = false; msg = "";
            [tIdx, dist] = obj.findNearestTarget(clickPos);
            if isempty(tIdx) || dist > tol, msg = "Click near target"; return; end
            if nargin > 4 && strcmpi(type, "Energy")
                a = EnergyAgent(numel(obj.agents)+1, obj.targets(tIdx).position, speed);
            else
                a = DefaultAgent(numel(obj.agents)+1, obj.targets(tIdx).position, speed);
            end
            a.current_target_idx = tIdx;
            a.initialTargetIdx = tIdx;
            a.initialPosition = a.state.pos;
            if isempty(obj.agents), obj.agents = a; else, obj.agents(end+1) = a; end
            ok = true;
        end

        function clearAll(obj)
            obj.agents = []; obj.targets = Target.empty; obj.edges = Edge.empty;
            obj.walls = []; obj.cumUncertaintyIntegral = 0; obj.hasLastSample = false;
        end

        function resetSimulationState(obj)
            obj.cumUncertaintyIntegral = 0; obj.hasLastSample = false;
            for k = 1:numel(obj.agents), obj.agents(k).resetToInitial(); end
            for t = 1:numel(obj.targets), obj.targets(t).reset(); end
        end

        function [idx, dist] = findNearestTarget(obj, pos)
            idx = []; dist = inf;
            if isempty(obj.targets), return; end
            P = reshape([obj.targets.position], 2, []).';
            d = hypot(P(:,1)-pos(1), P(:,2)-pos(2));
            [dist, idx] = min(d);
        end

        function adj = buildAdjacency(obj)
            n = numel(obj.targets); adj = false(n,n);
            for k = 1:numel(obj.edges)
                e = obj.edges(k); i = e.targets(1).index; j = e.targets(2).index;
                if i>=1 && i<=n && j>=1 && j<=n, adj(i,j)=true; adj(j,i)=true; end
            end
        end

        function e = findEdge(obj, t1, t2)
            e = [];
            for k = 1:numel(obj.edges)
                ids = [obj.edges(k).targets.index];
                if all(ismember([t1, t2], ids)), e = obj.edges(k); return; end
            end
        end

        function pol = getPolicyForAgent(obj, a)
            if isprop(a, 'type') && obj.policyMap.isKey(a.type)
                pol = obj.policyMap(a.type);
            else
                pol = obj.policyMap('Default');
            end
        end
    end
end