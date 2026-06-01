classdef KinematicState < handle
    properties
        pos double = [0 0]
        vel double = [0 0]
        acc double = [0 0]
        ori double = 0
        maxSpeed = 5
        maxAccel = 15
        wallDetected logical = false
        lastWallPoint double = [NaN NaN]
    end
    
    methods
        function obj = KinematicState(pos, maxSpeed, maxAccel)
            if nargin > 0 && ~isempty(pos), obj.pos = pos; end
            if nargin > 1 && ~isempty(maxSpeed), obj.maxSpeed = maxSpeed; end
            if nargin > 2 && ~isempty(maxAccel), obj.maxAccel = maxAccel; end
        end
    end
end