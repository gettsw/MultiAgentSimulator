classdef EnergyAgent < handle
    properties
        index
        state              % Instance of KinematicState
        controller

        path = []
        pathIndex = 1
        current_target_idx = []
        dwellRemaining double = 0
        mode string = "idle"

        e_total = 0;
        soc = 1.0;
        battery_percentage = 100;

        color = [0.8 0.2 0.2];
        size = 8;
        graphicHandle;
        textHandle;
        ax;

        initialPosition;
        initialTargetIdx;
    end

    properties (Constant)
        MASS = 10.0;
        ROLLING_FRICTION = 0.02;
        GRAVITY = 9.81;
        P0_BASE_MOTION = 5.82;
        ALPHA_VEL = 0.02 * 10 * 9.81;
        GAMMA_ACC = 0.5;

        CPU_MOVE = 5.5;
        CPU_IDLE = 4.0;
        COMP_ACTIVE = 11.7;
        COMP_STANDBY = 11.0;

        BATTERY_E_Wh = 26.0;
        ETA_CONV = 0.90;
    end

    methods
        function obj = EnergyAgent(index, position, maxSpeed)
            obj.index = index;
            % Initialize the separated Kinematic State
            obj.state = KinematicState(position, maxSpeed, 15);
            % Initialize the separated Controller
            obj.controller = PathFollowerController();

            obj.initialPosition = position;
        end

        function resetToInitial(obj)
            obj.state.pos = obj.initialPosition;
            obj.state.vel = [0 0];
            obj.state.acc = [0 0];
            obj.state.ori = 0;
            obj.e_total = 0;
            obj.soc = 1.0;
            obj.battery_percentage = 100;
            obj.path = [];
            obj.pathIndex = 1;
            obj.dwellRemaining = 0;
            obj.mode = "idle";
            obj.current_target_idx = obj.initialTargetIdx;
        end

        function update(obj, dt)
            if obj.battery_percentage <= 0
                obj.state.acc = [0 0];
                obj.state.vel = obj.state.vel * 0.9;
                obj.mode = "power_outage";
            elseif isempty(obj.path)
                obj.state.acc = [0 0];
                obj.state.vel = obj.state.vel * 0.8;
                obj.mode = "idle";
            else
                obj.mode = "traveling";

                [arrived, aCmd, nextIdx] = obj.controller.computeControl(obj.state, obj.path, obj.pathIndex, dt);

                obj.state.acc = aCmd;
                obj.pathIndex = nextIdx; 

                if arrived
                    obj.path = [];
                    obj.pathIndex = 1;
                end
            end

            PhysicalIntegrator.update(obj.state, dt);
            if ismethod(obj, 'calculate_energy'), obj.calculate_energy(dt); end
        end

        function calculate_energy(obj, dt)
            v_norm = norm(obj.state.vel);
            a_norm = norm(obj.state.acc);
            p_mot = obj.P0_BASE_MOTION + (obj.ALPHA_VEL * v_norm) + (obj.GAMMA_ACC * a_norm^2);

            if obj.mode == "traveling"
                p_cpu = obj.CPU_MOVE;
                p_comp = obj.COMP_ACTIVE;
            else
                p_cpu = obj.CPU_IDLE;
                p_comp = obj.COMP_STANDBY;
            end

            p_total = (p_cpu + p_comp + p_mot) / obj.ETA_CONV;
            energy_step = p_total * dt;
            obj.e_total = obj.e_total + energy_step;

            Q_joules = obj.BATTERY_E_Wh * 3600;
            obj.soc = obj.soc - (energy_step / Q_joules);
            obj.battery_percentage = max(0, obj.soc * 100);
        end

        function draw(obj, ax)
            obj.ax = ax;
            if ~isempty(obj.graphicHandle) && isvalid(obj.graphicHandle), delete(obj.graphicHandle); end
            [x, y] = obj.calculateVertices();
            obj.graphicHandle = patch(ax, x, y, obj.color, 'EdgeColor', 'k', 'FaceAlpha', 0.8);

            labelStr = sprintf('%d\n%.0f%%', obj.index, obj.battery_percentage);
            obj.textHandle = text(ax, obj.state.pos(1), obj.state.pos(2), labelStr, ...
                'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'w', 'FontSize', 7);
        end

        function updateVisuals(obj)
            if isempty(obj.graphicHandle) || ~isgraphics(obj.graphicHandle), return; end
            [x, y] = obj.calculateVertices();
            set(obj.graphicHandle, 'XData', x, 'YData', y);
            labelStr = sprintf('%d\n%.0f%%', obj.index, obj.battery_percentage);
            set(obj.textHandle, 'Position', [obj.state.pos(1), obj.state.pos(2), 0], 'String', labelStr);
        end
    end

    methods (Access = private)
        function [x, y] = calculateVertices(obj)
            h = obj.size * sqrt(3)/2;
            xb = [obj.size/2, -obj.size/2, -obj.size/2, obj.size/2];
            yb = [0, h/2, -h/2, 0];
            x = xb*cos(obj.state.ori) - yb*sin(obj.state.ori) + obj.state.pos(1);
            y = xb*sin(obj.state.ori) + yb*cos(obj.state.ori) + obj.state.pos(2);
        end
    end
end