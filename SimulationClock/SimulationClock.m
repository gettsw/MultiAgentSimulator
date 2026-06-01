classdef SimulationClock < handle
    properties
        currentTime double = 0
        endTime double = 60
        dt double = 0.01  % Fixed stable delta (e.g., 10ms)
        timeScale double = 1.0
    end

    methods
        function obj = SimulationClock(endTime, dt)
            if nargin >= 1 && ~isempty(endTime), obj.endTime = endTime; end
            if nargin >= 2 && ~isempty(dt), obj.dt = dt; end
        end

        function reset(obj)
            obj.currentTime = 0;
        end

        % We remove the multiplication here to keep physics stable
        function dtSim = tick(obj)
            dtSim = obj.dt; 

            remaining = obj.endTime - obj.currentTime;
            if remaining <= 0
                dtSim = 0;
                obj.currentTime = obj.endTime;
                return;
            end

            if dtSim > remaining
                dtSim = remaining;
            end

            obj.currentTime = obj.currentTime + dtSim;
        end

        function tf = isFinished(obj)
            tf = obj.currentTime >= (obj.endTime - 1e-12);
        end
    end
end