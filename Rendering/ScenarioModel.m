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
        policyMap               % name -> persistent policy instance (experiments read stats off these)
        activePolicy = 'Random Walk'
        uncertaintyMode = 'Dynamic Uncertainty' % Added for Dynamic vs. Static support
        pathPlanner
        agentFactory
        bounds = [0 100 0 100]  % [xMin xMax yMin yMax]
    end

    properties (Dependent)
        policy
    end

    methods
        function obj = ScenarioModel(pathPlanner, agentFactory)
            if nargin < 1 || isempty(pathPlanner), pathPlanner = DijkstraCornerPlanner(); end
            if nargin < 2 || isempty(agentFactory)
                agentFactory = @(idx, pos, sp) DefaultAgent(idx, pos, sp);
            end
            obj.pathPlanner  = pathPlanner;
            obj.agentFactory = agentFactory;

            % One instance per registered policy, kept for the model's lifetime
            reg = ComponentRegistry();
            obj.policyMap = containers.Map('KeyType','char','ValueType','any');
            names = reg.keysFor('policy');
            for k = 1:numel(names)
                f = reg.factoryFor('policy', names{k});
                obj.policyMap(names{k}) = f();
            end
        end

        function pol = get.policy(obj)
            pol = obj.policyMap(obj.activePolicy);
        end

        function setPathPlanner(obj, planner)
            obj.pathPlanner = planner;
            obj.replanAllEdges();
        end

        function setAgentFactory(obj, factory)
            obj.agentFactory = factory;
        end

        function setUncertaintyMode(obj, mode)
            if ischar(mode) || isstring(mode)
                obj.uncertaintyMode = char(mode);
            end
        end

        function setActivePolicy(obj, policyName)
            if ischar(policyName) || isstring(policyName)
                obj.activePolicy = char(policyName);
            end
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

            % 1. UPDATE RESIDING AGENTS AT TARGETS FIRST
            detectionRadius = 1.2;
            if ~isempty(obj.targets)
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
                end
            end

            % 2. UPDATE TARGET UNCERTAINTIES & REFRESH SPIKE STATES
            % Both modes integrate with dtSim; mode string dictates if A(t) spikes or stays A_base
            if ~isempty(obj.targets)
                for tIdx = 1:numel(obj.targets)
                    obj.targets(tIdx).updateUncertainty(dtSim, simTime, obj.uncertaintyMode);
                end
            end

            % 3. UPDATE AGENT STATES AND EXECUTE POLICIES
            for k = 1:numel(obj.agents)
                a = obj.agents(k);
                if isprop(a, 'dwellRemaining') && a.dwellRemaining > 0
                    a.dwellRemaining = max(0, a.dwellRemaining - dtSim);
                    continue;
                end
                if isempty(a.path)
                    adj = obj.buildAdjacency();
                    cmd = obj.policy.plan(a, obj, adj, a.current_target_idx, simTime);
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

        function [uNow, JNow] = updateTargetsAndLogObjective(obj, simTime, dtSim)
            % 1. AGGREGATE CURRENT TOTAL UNCERTAINTY ACROSS TARGETS
            uNow = 0;
            for t = 1:numel(obj.targets)
                uNow = uNow + obj.targets(t).R;
            end

            % 2. INTEGRATE CUMULATIVE OBJECTIVE (TRAPEZOIDAL RULE)
            if ~obj.hasLastSample
                obj.lastLogTime = simTime;
                obj.lastUncertainty = uNow;
                obj.hasLastSample = true;
            else
                dt = simTime - obj.lastLogTime;
                if dt > 0
                    obj.cumUncertaintyIntegral = obj.cumUncertaintyIntegral + 0.5 * (obj.lastUncertainty + uNow) * dt;
                    obj.lastLogTime = simTime;
                    obj.lastUncertainty = uNow;
                end
            end

            % 3. COMPUTE RUNNING TIME-AVERAGED COST J
            JNow = obj.cumUncertaintyIntegral / max(eps, simTime);
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
            path = obj.pathPlanner.plan(pStart, pEnd, obj.walls, obj.bounds);
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
            if isfield(s, 'walls'), obj.walls = s.walls; end
            for k = 1:size(s.edges, 1)
                obj.addEdgeByTargets(double(s.edges(k,1)), double(s.edges(k,2)));
            end
        end

        function t = addTarget(obj, pos)
            t = Target(numel(obj.targets) + 1, pos);
            obj.targets(end+1) = t;
        end

        function [a, ok, msg] = addAgentOnTarget(obj, clickPos, speed, tol)
            a = []; ok = false; msg = "";
            [tIdx, dist] = obj.findNearestTarget(clickPos);
            if isempty(tIdx) || dist > tol, msg = "Click near target"; return; end
            a = obj.agentFactory(numel(obj.agents)+1, obj.targets(tIdx).position, speed);
            a.current_target_idx = tIdx;
            a.initialTargetIdx = tIdx;
            a.initialPosition = a.state.pos;
            if isempty(obj.agents), obj.agents = a; else, obj.agents(end+1) = a; end
            ok = true;
        end

        function clearAll(obj)
            obj.agents = []; obj.targets = Target.empty; obj.edges = Edge.empty;
            obj.walls = []; obj.cumUncertaintyIntegral = 0; obj.hasLastSample = false;
            obj.lastLogTime = 0; obj.lastUncertainty = 0;
            obj.resetPolicyReservations();
        end

        function resetPolicyReservations(obj)
            % RHCPolicy keeps a persistent reservation table; stale locks survive resets otherwise
            pols = obj.policyMap.values;
            for k = 1:numel(pols)
                if ismethod(pols{k}, 'resetReservations'), pols{k}.resetReservations(); end
            end
        end

        function resetSimulationState(obj)
            obj.cumUncertaintyIntegral = 0;
            obj.hasLastSample = false;
            obj.lastLogTime = 0;
            obj.lastUncertainty = 0;
            for k = 1:numel(obj.agents), obj.agents(k).resetToInitial(); end
            for t = 1:numel(obj.targets), obj.targets(t).reset(); end
            obj.resetPolicyReservations();
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
                % Exact match; ismember-based check made findEdge(i,i) return any edge touching i,
                % so "hold at current node" commands sent agents down a random edge
                if isequal(ids, [t1 t2]) || isequal(ids, [t2 t1]), e = obj.edges(k); return; end
            end
        end

    end
end