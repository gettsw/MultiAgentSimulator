classdef PathFollowerController < handle
    properties
        kp = 8.0            
        kd = 12.0           
        lookAheadDist = 1.0 
        arrivePosTol = 0.5  
    end

    methods
        function [arrived, aCmd, pathIndex] = computeControl(obj, state, path, pathIndex, ~)
            arrived = false;
            if isempty(path), aCmd = -obj.kd * state.vel; return; end

            targetPos = path(pathIndex, :);
            distToTarget = norm(targetPos - state.pos);

            while distToTarget < obj.lookAheadDist && pathIndex < size(path, 1)
                pathIndex = pathIndex + 1;
                targetPos = path(pathIndex, :);
                distToTarget = norm(targetPos - state.pos);
            end

            if pathIndex == size(path, 1) && distToTarget < obj.arrivePosTol
                aCmd = -obj.kd * state.vel;
                if norm(state.vel) < 0.1, arrived = true; aCmd = [0 0]; end
            else
                aCmd = obj.kp * (targetPos - state.pos) - obj.kd * state.vel;
            end

            if norm(aCmd) > state.maxAccel
                aCmd = (aCmd / norm(aCmd)) * state.maxAccel;
            end
        end
    end
end