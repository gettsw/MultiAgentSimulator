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
        A                    % Growth rate
        B = 10               % Decay rate

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

        % NEW: store initial state for reset
        initialR
        initialA
        initialB
        initialColor
        initialMarkerSize

        % Bar scaling settings (tune as desired)
        barMaxDisplayHeight = 15   % height in axis units
        barMaxR = 10              % R value that maps to full height
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
            obj.A = 1;
            obj.prevAgentIDs = [];

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

            % Save initial state for reset
            obj.initialR = obj.R;
            obj.initialA = obj.A;
            obj.initialB = obj.B;
            obj.initialColor = obj.color;
            obj.initialMarkerSize = obj.markerSize;
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
            obj.A = obj.initialA;
            obj.B = obj.initialB;

            % Reset appearance
            obj.color = obj.initialColor;
            obj.markerSize = obj.initialMarkerSize;

            % Reset any planning fields
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

        function updateUncertainty(obj, dt)
            Ni = length(obj.residingAgents);
            vi = double(obj.R > 0 || obj.A / obj.B > Ni);
            dR = (obj.A - obj.B * Ni) * vi * dt;
            obj.R = max(0, obj.R + dR);

            % Update visuals
            if ~isempty(obj.barHandle) && isvalid(obj.barHandle)
                set(obj.barHandle, 'Position', obj.computeBarPosition());
                set(obj.barHandle, 'FaceColor', obj.computeBarColor());
            end
        end

        function updateResidingAgents(obj, agentList, globalTime)
            if isempty(agentList)
                currentIDs = [];
            elseif isobject(agentList)
                currentIDs = [agentList.index];
            elseif iscell(agentList)
                currentIDs = cellfun(@(x) x.index, agentList);
            else
                error('Invalid agentList type: %s', class(agentList));
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
            % Filled bar height is proportional to R (clamped)
            h = obj.computeBarHeight();
            x = obj.position(1) - obj.barWidth/2;
            y = obj.position(2) + obj.barYOffset;
            pos = [x, y, obj.barWidth, h];
        end

        function pos = computeBarFramePosition(obj)
            % Frame is constant "max" height
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
            % Green -> Red based on normalized R
            if obj.barMaxR <= 0
                c = [0.2 0.8 0.2];
                return;
            end
            alpha = min(max(obj.R / obj.barMaxR, 0), 1);
            cLow = [0.2 0.8 0.2];   % green
            cHigh = [0.9 0.2 0.2];  % red
            c = (1-alpha)*cLow + alpha*cHigh;
        end
    end
end