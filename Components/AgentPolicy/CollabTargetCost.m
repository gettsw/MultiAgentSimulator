classdef CollabTargetCost
    % Target-level costs used by CollabEDRHCPolicy (Collab-ED-RHC).
    %
    % lindley: the local cost J_i(t0, tf) = int_{t0}^{tf} R_i(s) ds and the end state R_i(tf) of a
    % target under piecewise-constant occupancy (Lemma G of the collaboration paper). Between two
    % consecutive switching times s_k < s_{k+1} (arrivals and departures) the net rate
    % sigma_k = A_i - b_k is constant, where b_k is the total sensing rate of the agents present, and
    %   R(s_{k+1}) = max(0, R(s_k) + sigma_k * Delta_k),
    %   J(s_k, s_{k+1}) = Delta_k * (2 R(s_k) + sigma_k * Delta_k) / 2   if R(s_k) + sigma_k Delta_k >= 0,
    %                     R(s_k)^2 / (2 |sigma_k|)                          otherwise,
    % with Delta_k = s_{k+1} - s_k. This is the Lindley recursion of queueing; it covers any number of
    % agents, any ordering of their arrivals and departures, and A_i >= B_i.
    %
    % terminal: the terminal correction hhat^T of the paper (revisit model truncated at T). From
    % the evaluation time, the target waits delta (its uncertainty grows at rate g), then it is
    % cleared at rate c > 0; the cost is counted only up to the mission end, s time units later.
    %
    % Both are vectorized over candidate plans (rows).

    methods (Static)
        function [J, Rf] = lindley(R0, A, t0, tf, b0, ev, db)
            % R0, A, t0: scalars (the target's state, growth rate and the start time)
            % tf:  C x 1 end times (one per candidate plan)
            % b0:  C x 1 (or scalar) total sensing rate at t0 (agents present at t0)
            % ev:  C x E switching times (Inf = none); events outside (t0, tf) are ignored
            %      except that an event at or before t0 changes the rate from t0 on
            % db:  C x E change of the sensing rate at each switching time (+B on an arrival,
            %      -B on a departure)
            C = numel(tf);
            tf = tf(:);
            if isscalar(b0), b0 = repmat(b0, C, 1); end
            b = b0(:);
            if isempty(ev)
                ev = zeros(C, 0); db = zeros(C, 0);
            end
            if size(ev, 1) == 1 && C > 1
                ev = repmat(ev, C, 1); db = repmat(db, C, 1);
            end
            early = ev <= t0;                       % already in effect at t0
            b = b + sum(db .* early, 2);
            ev(early) = Inf; db(early) = 0;
            [ev, order] = sort(ev, 2);
            db = db(sub2ind(size(db), repmat((1:C)', 1, size(db, 2)), order));
            R = repmat(R0, C, 1);
            J = zeros(C, 1);
            t = repmat(t0, C, 1);
            for e = 1:size(ev, 2) + 1
                if e <= size(ev, 2)
                    s = min(ev(:, e), tf);
                else
                    s = tf;
                end
                D = max(s - t, 0);
                sigma = A - b;
                Rn = R + sigma .* D;
                hit = Rn < 0;
                J = J + (~hit) .* D .* (R + Rn) / 2;
                J(hit) = J(hit) + R(hit).^2 ./ (2 * abs(sigma(hit)));
                Rn(hit) = 0;
                R = Rn;
                t = max(t, s);
                if e <= size(ev, 2)
                    applies = ev(:, e) < tf;
                    b = b + db(:, e) .* applies;
                end
            end
            Rf = R;
        end

        function h = terminal(R, g, c, delta, s, lam)
            % Relative value of a target under the revisit model, counted up to the mission end:
            % the target waits delta at growth rate g >= 0, then it is cleared at rate c > 0 until
            % R = 0 (after q = (R + g delta)/c); h is its cost until then minus lam times the
            % time until then (renewal-reward form of the relative value, with lam the target's
            % average cost rate), all truncated to the remaining time s >= 0. With s = Inf,
            %   h = R delta + g delta^2/2 + (R + g delta)^2/(2c) - lam (delta + q).
            % lam = 0 (default) gives the cost alone. Inputs: arrays of one size, or scalars.
            if nargin < 6, lam = 0; end
            Z = zeros(size(R + g + c + delta + s + lam));      % common size (s may be Inf)
            R = R + Z; g = g + Z; c = c + Z; delta = delta + Z; s = s + Z; lam = lam + Z;
            w = min(delta, s);                                 % waiting time within the mission
            h = R .* w + g .* w.^2 / 2;
            Rw = R + g .* w;
            rest = max(s - delta, 0);                          % time left for clearing
            q = Rw ./ c;                                       % clearing time
            full = rest >= q;
            h(full) = h(full) + Rw(full).^2 ./ (2 * c(full));
            p = ~full;                                         % the mission ends while clearing
            h(p) = h(p) + Rw(p) .* rest(p) - c(p) .* rest(p).^2 / 2;
            h = h - lam .* min(delta + q, s);                  % time to the next clearing, truncated
            h(~isfinite(c) | c <= 0) = NaN;                    % caller guarantees c > 0
        end
    end
end
