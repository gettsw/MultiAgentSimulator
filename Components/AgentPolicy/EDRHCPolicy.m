classdef EDRHCPolicy < handle
    % Event-driven receding horizon control (ED-RHC) for persistent monitoring on networks,
    % with the weighted RHCP3 objective of RHC-alpha: S. Welikala and C. G. Cassandras,
    % "Event-driven receding horizon control for distributed persistent monitoring in network
    % systems", Automatica 127 (2021) 109519, Secs. II-IV.
    %
    % Each agent decides for itself, at its events, using only local information: its target
    % i and the 1-hop neighbors of i (their R, A, B, the transit times of the edges (i, j), and
    % whether each neighbor is covered). An agent at target i solves a receding horizon control
    % problem (RHCP) at these events:
    %   RHCP1 on arrival:                 active time u_i (R_i decreasing)
    %   RHCP2 when R_i reaches 0:         inactive time v_i (R_i = 0)
    %   RHCP3 when u_i or v_i is over:    next target j, then it leaves
    % and re-solves RHCP1 (active) or RHCP2 (inactive) when a neighbor becomes covered or
    % uncovered. A target is covered while another agent is at it or on its way to it; covered
    % neighbors are left out of the RHCPs, so agents never share a target. Each RHCP plans
    % [u_i, v_i] at i, the transit rho_ij, and [u_j, v_j] at a neighbor j, over the variable
    % horizon w = u_i + v_i + rho_ij + u_j + v_j <= H, and minimizes the neighborhood cost
    %   (1/w) * sum over the neighborhood of int_t^{t+w} R_m(s) ds.
    % In RHCP3 the cost of j is weighted by alpha and the rest by 1 - alpha (Sec. IV-A);
    % alpha = 0.5 gives the unweighted ED-RHC of Sec. III.
    %
    % The RHCPs are solved numerically: each smooth case of the paper's case split (v > 0
    % only after R has been driven to 0) is searched on a grid and refined with fmincon,
    % instead of with the closed forms of the paper's Appendix A.
    %
    % Transit times are path length / maxSpeed, which holds for first-order agents moving at
    % constant speed (LinearAgent). Other agent types run, but these estimates are off.
    properties
        alpha = "nominal"    % RHCP3 weight of the next target: "nominal" = 1/|N_i|^2 (N_i
                             % includes i), or a number in [0, 1]; 0.5 = unweighted ED-RHC
        H = 250              % upper bound on the planning horizon w [s]
        recheckTime = 0      % wait before RHCP3 is tried again when every neighbor is covered
                             % (0 = every step, as in the JS simulator)
        gridSize = 15        % grid points per decision variable before the fmincon refinement
        traceDecisions = false   % record every RHCP decision in decisions
        decisions = zeros(0, 5)  % [time, agent, RHCP kind, target i, decision (u_i, v_i or j)]
    end

    properties (Access = private)
        states               % agent index -> struct(stage, target, covered)
        options              % fmincon options
    end

    properties (Constant, Access = private)
        tol = 1e-6
    end

    methods
        function obj = EDRHCPolicy(opts)
            arguments
                opts.Alpha = "nominal"
            end
            obj.alpha = opts.Alpha;
            obj.states = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');
        end

        function resetReservations(obj)
            % Called by ScenarioModel on every reset: forget the agents' stages and decisions
            obj.states = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.decisions = zeros(0, 5);
        end

        function cmd = plan(obj, agent, model, adj, curIdx, simTime)
            % Called when the agent has no path and no dwell left: on arrival (or at t = 0), and
            % when an active or inactive time is over
            cmd = struct();
            s = obj.stateOf(agent);
            if s.stage == "travel"
                u = obj.solve(1, agent, model, adj, curIdx, simTime);
                if u > obj.tol
                    obj.dwell(agent, model, adj, curIdx, "active", u);
                    return;
                end
            end
            if s.stage ~= "inactive" && model.targets(curIdx).R <= obj.tol
                v = obj.solve(2, agent, model, adj, curIdx, simTime);
                if v > obj.tol
                    obj.dwell(agent, model, adj, curIdx, "inactive", v);
                    return;
                end
            end
            j = obj.solve(3, agent, model, adj, curIdx, simTime);
            if isempty(j)
                % Every neighbor is covered: stay at i (still servicing it) and try RHCP3 again;
                % the agent leaves as soon as a neighbor is uncovered, or solves RHCP2 first if
                % R_i reaches 0 meanwhile (as the JS simulator does)
                stage = s.stage;
                if stage == "travel", stage = "active"; end
                obj.dwell(agent, model, adj, curIdx, stage, obj.recheckTime, true);
                return;
            end
            obj.states(agent.index) = struct('stage', "travel", 'target', j, 'covered', [], 'hold', false);
            cmd.kind = "move";
            cmd.targetIdx = j;
        end

        function onStep(obj, agent, model, simTime)
            % Covering and uncovering events: re-solve RHCP1 or RHCP2 for a dwell in progress
            if ~isKey(obj.states, agent.index) || agent.dwellRemaining <= 0, return; end
            s = obj.states(agent.index);
            if s.stage == "travel" || s.hold, return; end
            adj = model.buildAdjacency();
            covered = obj.coveredNeighbors(agent, model, adj, s.target);
            if isequal(covered, s.covered), return; end
            if s.stage == "active"
                agent.dwellRemaining = obj.solve(1, agent, model, adj, s.target, simTime);
            else
                agent.dwellRemaining = obj.solve(2, agent, model, adj, s.target, simTime);
            end
            s.covered = covered;
            obj.states(agent.index) = s;
        end

        function best = rhcp(obj, kind, agent, model, i, simTime, adj)
            % Solves RHCP<kind> (1, 2 or 3) for the agent at target i and returns the best plan:
            % best.j (next target; [] if every neighbor is covered), best.x = [u_i v_i u_j v_j],
            % best.cost, and best.uBi (active time that drives R_i to 0)
            if nargin < 7, adj = model.buildAdjacency(); end
            nbrs = find(adj(i, :));
            avail = nbrs(~obj.coveredNeighbors(agent, model, adj, i));
            hood = [i, avail];                    % the neighborhood without covered neighbors
            tg = model.targets(hood);
            R = [tg.R];
            A = arrayfun(@(t) t.growthRateAt(simTime, model.uncertaintyMode), tg);
            B = [tg.B];
            R(R < obj.tol) = 0;
            uBi = obj.clearTime(R(1), A(1), B(1));

            if kind == 3
                weights = obj.weights(numel(nbrs) + 1);
            else
                weights = [1 1];
            end
            best = struct('cost', inf, 'j', [], 'x', zeros(1, 4), 'uBi', uBi);
            for k = 2:numel(hood)
                rho = obj.transitTime(model, i, hood(k), agent);
                if rho > obj.H, continue; end
                d = struct('R', R, 'A', A, 'B', B, 'j', k, 'rho', rho, 'w', weights);
                for c = obj.cases(kind, d, uBi)
                    [cost, x] = obj.solveCase(d, c);
                    if cost < best.cost
                        best = struct('cost', cost, 'j', hood(k), 'x', x, 'uBi', uBi);
                    end
                end
            end
        end
    end

    methods (Static)
        function J = planCost(d, P)
            % Weighted neighborhood cost (1/w) * int R over [t, t + w] for plans P = [ui vi uj vj]
            % (one row each), by Theorem 1 of the paper on the piecewise linear R trajectories
            ui = P(:, 1); vi = P(:, 2); uj = P(:, 3); vj = P(:, 4);
            j = d.j;
            others = true(size(d.R)); others([1 j]) = false;
            w = ui + vi + d.rho + uj + vj;
            % target i: active (R falls), inactive (R = 0), then grows after the agent leaves
            Ri1 = max(d.R(1) - (d.B(1) - d.A(1)) * ui, 0);
            after = d.rho + uj + vj;
            Ji = ui .* (d.R(1) + Ri1) / 2 + after .* (2 * Ri1 + d.A(1) * after) / 2;
            % target j: grows until the agent arrives, falls while it is active, then 0
            arrive = ui + vi + d.rho;
            Rj1 = d.R(j) + d.A(j) * arrive;
            Jj = arrive .* (d.R(j) + Rj1) / 2 + uj .* (2 * Rj1 - (d.B(j) - d.A(j)) * uj) / 2;
            % every other target grows over the whole horizon
            Jm = w .* (2 * sum(d.R(others)) + sum(d.A(others)) * w) / 2;
            J = (d.w(1) * Jj + d.w(2) * (Ji + Jm)) ./ w;
        end
    end

    methods (Access = private)
        function s = stateOf(obj, agent)
            if isKey(obj.states, agent.index)
                s = obj.states(agent.index);
            else
                s = struct('stage', "travel", 'target', agent.current_target_idx, 'covered', [], 'hold', false);
            end
        end

        function dwell(obj, agent, model, adj, i, stage, duration, hold)
            % hold = true: waiting for a neighbor to be uncovered (no re-solve on events)
            if nargin < 8, hold = false; end
            agent.dwellRemaining = duration;
            obj.states(agent.index) = struct('stage', stage, 'target', i, ...
                'covered', obj.coveredNeighbors(agent, model, adj, i), 'hold', hold);
        end

        function covered = coveredNeighbors(~, agent, model, adj, i)
            % For each neighbor of i (in index order): is another agent at it or heading to it?
            nbrs = find(adj(i, :));
            covered = false(size(nbrs));
            for b = 1:numel(model.agents)
                other = model.agents(b);
                if other ~= agent
                    covered = covered | nbrs == other.current_target_idx;
                end
            end
        end

        function out = solve(obj, kind, agent, model, adj, i, simTime)
            % The decision of RHCP<kind> at target i: u_i (kind 1), v_i (kind 2), or the next
            % target j (kind 3; [] when every neighbor is covered)
            best = obj.rhcp(kind, agent, model, i, simTime, adj);
            switch kind   % with no neighbor to plan for, u_i = v_i = 0: try RHCP3 (as the JS)
                case 1
                    if isempty(best.j), out = 0; else, out = best.x(1); end
                case 2
                    if isempty(best.j), out = 0; else, out = best.x(2); end
                case 3
                    out = best.j;
            end
            if obj.traceDecisions
                value = out;
                if isempty(value), value = NaN; end
                obj.decisions(end+1, :) = [simTime, agent.index, kind, i, value];
            end
        end

        function w = weights(obj, hoodSize)
            % [weight of the planned next target j, weight of every other target]
            a = obj.alpha;
            if isstring(a) || ischar(a)
                assert(a == "nominal", 'EDRHCPolicy:Alpha', 'alpha must be "nominal" or a number');
                a = 1 / hoodSize^2;
            end
            w = [a, 1 - a];
        end

        function cs = cases(obj, kind, d, uBi)
            % The smooth cases of RHCP<kind>, each as [ui vi uj vj] = M*[x; y] + c over
            % x, y >= 0 with Aineq*[x; y] <= bineq (paper Sec. III, eqs. 20, 36, 46)
            Rj = d.R(d.j); Aj = d.A(d.j); Bj = d.B(d.j);
            room = obj.H - d.rho;
            if Bj > Aj
                c0 = (Rj + Aj * d.rho) / (Bj - Aj);   % u_j that drives R_j to 0 if j is reached at t + rho
                c1 = Aj / (Bj - Aj);                  % its growth per unit of waiting before leaving i
            else
                c0 = inf; c1 = 0;                     % R_j cannot be driven to 0
            end
            cs = struct('M', {}, 'c', {}, 'Aineq', {}, 'bineq', {});
            switch kind
                case 3   % u_i = v_i = 0; x = u_j or v_j
                    cs(end+1) = mk([0 0; 0 0; 1 0; 0 0], [0 0 0 0], [1 0; 0 1], [min(c0, room); 0]);
                    if c0 <= room
                        cs(end+1) = mk([0 0; 0 0; 0 0; 1 0], [0 0 c0 0], [1 0; 0 1], [room - c0; 0]);
                    end
                case 2   % u_i = 0; x = v_i
                    cs(end+1) = mk([0 0; 1 0; 0 1; 0 0], [0 0 0 0], [-c1 1; 1 1], [c0; room]);
                    if isfinite(c0)
                        cs(end+1) = mk([0 0; 1 0; c1 0; 0 1], [0 0 c0 0], [1 + c1 1], room - c0);
                    end
                case 1
                    % u_i <= uBi, v_i = 0
                    cs(end+1) = mk([1 0; 0 0; 0 1; 0 0], [0 0 0 0], [1 0; -c1 1; 1 1], [uBi; c0; room]);
                    if isfinite(c0)
                        cs(end+1) = mk([1 0; 0 0; c1 0; 0 1], [0 0 c0 0], [1 0; 1 + c1 1], [uBi; room - c0]);
                    end
                    % u_i = uBi (R_i driven to 0), v_i >= 0
                    if isfinite(uBi) && uBi <= room
                        cs(end+1) = mk([0 0; 1 0; 0 1; 0 0], [uBi 0 0 0], [-c1 1; 1 1], [c0 + c1 * uBi; room - uBi]);
                        if isfinite(c0)
                            cs(end+1) = mk([0 0; 1 0; c1 0; 0 1], [uBi 0 c0 + c1 * uBi 0], [1 + c1 1], room - uBi - c0 - c1 * uBi);
                        end
                    end
            end
            cs = cs(arrayfun(@(c) all(c.bineq >= -obj.tol), cs));   % drop empty cases

            function c = mk(M, c, Aineq, bineq)
                c = struct('M', M, 'c', c, 'Aineq', Aineq, 'bineq', bineq);
            end
        end

        function [cost, x] = solveCase(obj, d, c)
            % Grid search over the case's feasible polygon, then fmincon from the best point
            keep = isfinite(c.bineq);             % a bound of inf (R_j cannot be cleared) is no bound
            c.Aineq = c.Aineq(keep, :);
            c.bineq = c.bineq(keep);
            ub = obj.H * [1 1];
            for v = 1:2   % bounds implied by single-variable rows of Aineq
                rows = c.Aineq(:, v) > 0 & c.Aineq(:, 3 - v) >= 0;
                if any(rows), ub(v) = min([ub(v); c.bineq(rows) ./ c.Aineq(rows, v)]); end
            end
            ub = max(ub, 0);
            n = obj.gridSize;
            [X, Y] = meshgrid(linspace(0, ub(1), n), linspace(0, ub(2), n));
            Z = [X(:), Y(:)];
            Z = Z(all(Z * c.Aineq' <= c.bineq' + obj.tol, 2), :);
            if isempty(Z), Z = [0 0]; end
            costs = obj.planCost(d, Z * c.M' + c.c);
            [~, k] = min(costs);
            f = @(z) obj.planCost(d, z(:)' * c.M' + c.c);
            [z, fz] = fmincon(f, Z(k, :)', c.Aineq, c.bineq, [], [], [0; 0], ub', [], obj.options);
            if fz <= costs(k) && all(z >= -obj.tol) && all(c.Aineq * z <= c.bineq + obj.tol)
                cost = fz; x = max(z(:)', 0) * c.M' + c.c;
            else
                cost = costs(k); x = Z(k, :) * c.M' + c.c;
            end
        end

        function rho = transitTime(~, model, i, j, agent)
            pts = model.findEdge(i, j).curvePoints;
            rho = sum(vecnorm(diff(pts, 1, 1), 2, 2)) / agent.state.maxSpeed;
        end

        function u = clearTime(~, R, A, B)
            % Active time that drives R to 0 (inf if the agent cannot)
            if B > A, u = R / (B - A); else, u = inf; end
        end
    end
end
