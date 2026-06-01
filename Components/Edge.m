classdef Edge < handle
    properties
        index
        targets
        curvePoints 
        lineHandle
    end
    
    methods
        function obj = Edge(idx, targets, pathPoints)
            obj.index = idx;
            obj.targets = targets;
            obj.curvePoints = pathPoints;
            obj.lineHandle = [];
        end
    end
end