classdef EnergyAgent < Agent
    % Second-order agent (PD path follower) with a battery: power draw from motion, CPU and
    % sensing, tracked as energy used and state of charge (SOC)
    properties
        mode string = "idle"

        e_total = 0;
        soc = 1.0;
        battery_percentage = 100;
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
            obj@Agent(index, position, maxSpeed);
            obj.labelFontSize = 7;
        end

        function resetToInitial(obj)
            resetToInitial@Agent(obj);
            obj.e_total = 0;
            obj.soc = 1.0;
            obj.battery_percentage = 100;
            obj.mode = "idle";
        end

        function s = label(obj)
            s = sprintf('%d\n%.0f%%', obj.index, obj.battery_percentage);
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
            obj.calculate_energy(dt);
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
    end
end
