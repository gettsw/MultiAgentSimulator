classdef Target < handle
    properties
        index
        position
        residingAgents
        recentArrivals
        arrivalTimes
        departureTimes
        color
        markerSize
        graphicHandle
        labelHandle
        % Uncertainty visuals
        barHandle            % main filled bar
        barFrameHandle       % faint container/frame bar
        % Uncertainty state
        R                    % Uncertainty state
        A                    % Dynamic growth rate A(t)
        B = 20.0             % Base decay rate per agent
        lastVisitedTime = 0  % Timestamp when an agent last visited/cleared uncertainty
        % Dynamic Spike Parameters (Scenario C)
        A_base = 1.0          % Baseline growth rate for the active mode (set each step)
        A_priorityBase = 1.0  % Baseline when target 1 is the high-priority target
        A_uniformBase = 1.0   % Baseline for every target in 'Static Uniform'
        A_spike = 28.0        % Emergency spike growth rate (1 agent: +8 net, 2 agents: -12 net)
        spikeInterval = 250.0 % Total cycle repeat time (5 targets * 50s step = 250s)
        spikeDuration = 10.0  % Active window duration of spike (seconds)
        spikePhaseOffset = 0  % Stagger offset (seconds)

        % Visited target properties
        t0
        tz
        dwellTime
        R0_val
        t0_val
        travelTime
        J_i
        % Avoided target properties
        tz_avoided
        % Precomputed calculation values
        A_val
        B_val
    end

    properties (Access = private)
        prevAgentIDs
        % Store initial state for reset
        initialR
        initialA
        initialB
        initialColor
        initialMarkerSize
        initialLastVisitedTime
        % Bar scaling settings
        barMaxDisplayHeight = 15   % height in axis units
        barMaxR = 25              % R value that maps to full height (expanded for spikes)
        barWidth = 2              % width of the bar
        barYOffset = 3            % offset above the target marker
    end

    methods
        function obj = Target(index, position)
            obj.index = index;
            obj.position = position;
            obj.residingAgents = [];
            obj.recentArrivals = [];
            obj.arrivalTimes = [];
            obj.departureTimes = [];
            obj.color = 'b';
            obj.markerSize = 8;
            obj.graphicHandle = [];
            obj.labelHandle = [];
            obj.barHandle = [];
            obj.barFrameHandle = [];

            obj.R = 0;
            obj.B = 20.0;

            % --- Scenario C Configuration (40s Delay Between Spikes) ---
            obj.A_base = 1.0;
            obj.A_spike = 28.0;
            obj.spikeDuration = 10.0;  % 10 seconds of active spike

            % Priority modes: Target 1 grows faster; 'Static Uniform' uses A_base = 1 for all
            if index == 1
                obj.A_priorityBase = 5.0;
            end
            obj.A_base = obj.A_priorityBase;

            % 10s active spike + 40s delay = 50s stagger between consecutive targets
            % Cycle time for 5 targets = 5 * 50s = 250s
            obj.spikeInterval = 250.0;
            obj.spikePhaseOffset = (index - 1) * 50.0;

            obj.A = obj.A_base;
            obj.lastVisitedTime = 0;
            obj.prevAgentIDs = [];

            obj.resetTypeSpecificProperties();

            % Save initial state for reset
            obj.initialR = obj.R;
            obj.initialA = obj.A_base;
            obj.initialB = obj.B;
            obj.initialLastVisitedTime = obj.lastVisitedTime;
            obj.initialColor = obj.color;
            obj.initialMarkerSize = obj.markerSize;
        end

        function updateGrowthRate(obj, globalTime, uncertaintyMode)
            % Evaluates dynamic growth rate A(t) based on periodic spike windows
            if nargin < 3, uncertaintyMode = 'Dynamic Uncertainty'; end
            [obj.A, obj.A_base] = obj.growthRateAt(globalTime, uncertaintyMode);
        end

        function A = maxGrowthRate(obj, uncertaintyMode)
            % Largest growth rate this target can ever have in the given mode
            [A, baseA] = obj.growthRateAt(0, uncertaintyMode);
            if ~startsWith(uncertaintyMode, 'Static')
                A = max(baseA, obj.A_spike);
            end
        end

        function [A, baseA] = growthRateAt(obj, globalTime, uncertaintyMode)
            % Growth rate at any (future) time without changing state; used by planners
            if strcmp(uncertaintyMode, 'Static Uniform')
                baseA = obj.A_uniformBase;   % Every target identical
            else
                baseA = obj.A_priorityBase;  % Target 1 high
            end
            A = baseA;                       % Static modes: constant baseline
            if ~startsWith(uncertaintyMode, 'Static')
                cycleTime = mod(globalTime + obj.spikePhaseOffset, obj.spikeInterval);
                if cycleTime <= obj.spikeDuration
                    A = obj.A_spike;         % Active emergency spike window
                end
            end
        end

        function updateUncertainty(obj, dt, globalTime, uncertaintyMode)
            if nargin < 4
                uncertaintyMode = 'Dynamic Uncertainty';
            end

            % Update growth rate (constant A_base for Static, spiking for Dynamic)
            if nargin >= 3 && ~isempty(globalTime)
                obj.updateGrowthRate(globalTime, uncertaintyMode);
            end

            Ni = length(obj.residingAgents);

            % Heaviside switch: Uncertainty grows if R > 0 or if growth rate exceeds total decay capacity
            if obj.B > 0
                vi = double(obj.R > 0 || (obj.A / obj.B) > Ni);
            else
                vi = double(obj.R > 0 || obj.A > 0);
            end

            % Integrate uncertainty dR/dt = (A - B*N_i)*v_i
            dR = (obj.A - obj.B * Ni) * vi * dt;
            obj.R = max(0, obj.R + dR);
        end

        function configure(obj, p)
            % Overrides uncertainty parameters (used by Experiments/runTrial)
            obj.B = p.B;
            obj.initialB = p.B;
            obj.A_spike = p.A_spike;
            obj.spikeDuration = p.SpikeDuration;
            obj.spikeInterval = p.SpikeInterval;
            obj.spikePhaseOffset = (obj.index - 1) * p.SpikeStagger;
            obj.A_uniformBase = p.A_base;
            if obj.index == 1
                obj.A_priorityBase = p.A_priority;
            else
                obj.A_priorityBase = p.A_base;
            end
            obj.A_base = obj.A_priorityBase;
            obj.A = obj.A_base;
        end

        function updateBar(obj)
            % Called once per rendered frame, not every physics step
            if ~isempty(obj.barHandle) && isvalid(obj.barHandle)
                set(obj.barHandle, 'Position', obj.computeBarPosition(), ...
                    'FaceColor', obj.computeBarColor());
            end
        end

        function reset(obj)
            % Reset dynamic simulation state
            obj.residingAgents = [];
            obj.recentArrivals = [];
            obj.arrivalTimes = [];
            obj.departureTimes = [];
            obj.prevAgentIDs = [];

            % Reset uncertainty model
            obj.R = obj.initialR;
            obj.A = obj.A_base;
            obj.B = obj.initialB;
            obj.lastVisitedTime = obj.initialLastVisitedTime;

            % Reset appearance
            obj.color = obj.initialColor;
            obj.markerSize = obj.initialMarkerSize;

            % Reset planning fields
            obj.resetTypeSpecificProperties();

            % Refresh graphics if already drawn
            if ~isempty(obj.graphicHandle) && isvalid(obj.graphicHandle)
                set(obj.graphicHandle, ...
                    'MarkerSize', obj.markerSize, ...
                    'MarkerFaceColor', obj.color);
            end
            if ~isempty(obj.labelHandle) && isvalid(obj.labelHandle)
                set(obj.labelHandle, 'Color', obj.color);
            end
            if ~isempty(obj.barHandle) && isvalid(obj.barHandle)
                set(obj.barHandle, 'Position', obj.computeBarPosition());
                set(obj.barHandle, 'FaceColor', obj.computeBarColor());
            end
            if ~isempty(obj.barFrameHandle) && isvalid(obj.barFrameHandle)
                set(obj.barFrameHandle, 'Position', obj.computeBarFramePosition());
            end
        end

        function draw(obj, ax)
            % --- Bar frame (container) ---
            obj.barFrameHandle = rectangle(ax, ...
                'Position', obj.computeBarFramePosition(), ...
                'EdgeColor', [0.7 0.7 0.7], ...
                'LineStyle', '--', ...
                'LineWidth', 1.0, ...
                'FaceColor', 'none', ...
                'HandleVisibility', 'off');

            % --- Filled bar ---
            obj.barHandle = rectangle(ax, ...
                'Position', obj.computeBarPosition(), ...
                'FaceColor', obj.computeBarColor(), ...
                'EdgeColor', 'k', ...
                'LineWidth', 1.2, ...
                'Curvature', 0.1, ...
                'HandleVisibility', 'off');

            % --- Target marker ---
            obj.graphicHandle = plot(ax, obj.position(1), obj.position(2), 'o', ...
                'MarkerSize', obj.markerSize, ...
                'MarkerFaceColor', obj.color, ...
                'MarkerEdgeColor', 'k');

            % --- Label (index) ---
            obj.labelHandle = text(ax, obj.position(1), obj.position(2) - 0.04, ...
                num2str(obj.index), ...
                'Color', obj.color, ...
                'FontSize', 10, ...
                'HorizontalAlignment', 'center');
        end

        function updateResidingAgents(obj, agentList, globalTime)
            if isempty(agentList)
                currentIDs = [];
            else
                currentIDs = zeros(1, numel(agentList));
                for idx = 1:numel(agentList)
                    if iscell(agentList)
                        currentIDs(idx) = agentList{idx}.index;
                    else
                        currentIDs(idx) = agentList(idx).index;
                    end
                end
            end

            if ~isempty(agentList)
                obj.lastVisitedTime = globalTime;
            end

            newArrivals = setdiff(currentIDs, obj.prevAgentIDs);
            departures = setdiff(obj.prevAgentIDs, currentIDs);

            for id = newArrivals(:)'
                obj.arrivalTimes(end+1) = globalTime;
                obj.recentArrivals(end+1) = double(id);
            end
            for id = departures(:)'
                obj.departureTimes(end+1) = globalTime;
                obj.recentArrivals(obj.recentArrivals == id) = [];
            end

            if isempty(obj.recentArrivals)
                obj.arrivalTimes = [];
                obj.departureTimes = [];
            end

            obj.prevAgentIDs = currentIDs;
            obj.residingAgents = agentList;
        end

        % ================= VISITED TARGET METHODS =================
        function initializeAsVisited(obj, baseTarget, t0, edge)
            props = properties(baseTarget);
            for i = 1:length(props)
                if isprop(obj, props{i}) && ~strcmp(props{i}, 'index') && ~strcmp(props{i}, 'position')
                    obj.(props{i}) = baseTarget.(props{i});
                end
            end
            obj.t0 = t0;
            obj.t0_val = t0;
            obj.R0_val = baseTarget.R;
            obj.A_val = baseTarget.A;
            obj.B_val = baseTarget.B;
            obj.travelTime = edge.length / 80; % distance / speed
        end

        function J = objectiveVisited_numeric(obj, dwellTime)
            dwellTime = max(dwellTime, 1e-6) - obj.travelTime;
            t_arr = obj.t0_val + obj.travelTime;
            t_z_val = t_arr + dwellTime;
            t_vals = linspace(t_arr, t_z_val, 200);
            R_raw = obj.R0_val + obj.A_val*(t_vals - obj.t0_val) - obj.B_val*(t_vals - t_arr);
            heaviside_mask = double(R_raw >= 0);
            R_vals = R_raw .* heaviside_mask;
            integral_val = trapz(t_vals, R_vals);
            J = integral_val / (obj.travelTime + dwellTime);
            obj.tz = t_z_val;
            obj.dwellTime = dwellTime;
            obj.J_i = J;
        end

        function H = clearTimeHere(obj, nAgents, H_min, H_max)
            % Dwell that drives this target's live R to 0 with nAgents present, clamped
            net = nAgents * obj.B - obj.A;
            if net <= 0
                H = H_max;   % cannot be cleared: stay as long as allowed
            else
                H = obj.R / net;
            end
            H = min(max(H, H_min), H_max);
        end

        % ================= AVOIDED TARGET METHODS =================
        function initializeAsAvoided(obj, baseTarget, t0)
            obj.R0_val = baseTarget.R;
            obj.A_val = baseTarget.A;
            obj.t0_val = t0;
        end

        function J = objectiveAvoided_numeric(obj, H)
            J = (obj.R0_val * H + 0.5 * obj.A_val * H^2) / H;
            obj.J_i = J;
        end

        % ================= HELPER METHODS =================
        function resetTypeSpecificProperties(obj)
            obj.t0 = [];
            obj.tz = [];
            obj.dwellTime = [];
            obj.R0_val = [];
            obj.t0_val = [];
            obj.travelTime = [];
            obj.J_i = [];
            obj.tz_avoided = [];
            obj.A_val = [];
            obj.B_val = [];
        end
    end

    methods (Access = private)
        function pos = computeBarPosition(obj)
            h = obj.computeBarHeight();
            x = obj.position(1) - obj.barWidth/2;
            y = obj.position(2) + obj.barYOffset;
            pos = [x, y, obj.barWidth, h];
        end

        function pos = computeBarFramePosition(obj)
            x = obj.position(1) - obj.barWidth/2;
            y = obj.position(2) + obj.barYOffset;
            pos = [x, y, obj.barWidth, obj.barMaxDisplayHeight];
        end

        function h = computeBarHeight(obj)
            if obj.barMaxR <= 0
                h = 0;
                return;
            end
            frac = min(max(obj.R / obj.barMaxR, 0), 1);
            h = frac * obj.barMaxDisplayHeight;
        end

        function c = computeBarColor(obj)
            if obj.barMaxR <= 0
                c = [0.2 0.8 0.2];
                return;
            end

            % Highlight active emergency spike with dynamic magenta color indicator
            if obj.A > obj.A_base
                c = [0.9 0.1 0.5]; % Magenta / Red-Violet alert state
            else
                alpha = min(max(obj.R / obj.barMaxR, 0), 1);
                cLow = [0.2 0.8 0.2];   % Green
                cHigh = [0.9 0.2 0.2];  % Red
                c = (1 - alpha) * cLow + alpha * cHigh;
            end
        end
    end
end