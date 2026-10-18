classdef (Abstract) PathPlanner < handle
    methods (Abstract)
        path = plan(obj, pStart, pEnd, walls, bounds)
    end
end
