classdef (Abstract) PathPlanner < handle
    % Base class of the path planners. plan(pStart, pEnd, walls, bounds) returns the N x 2
    % points of a path from pStart to pEnd that avoids the walls (rows [x1 y1 x2 y2]) and stays
    % within bounds ([xMin xMax yMin yMax]). The model plans each edge once, when it is added
    % (and again when the walls or the planner change). A planner that finds no wall-free route
    % returns the straight line and sets noRoute, which the model reports.
    properties
        noRoute = false   % true when the last plan found no wall-free route
    end

    methods (Abstract)
        path = plan(obj, pStart, pEnd, walls, bounds)
    end
end
