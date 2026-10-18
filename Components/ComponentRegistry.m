classdef ComponentRegistry < handle
    % Central dictionary of modular components.
    % To register a new algorithm, add one line to the constructor.
    properties
        pathPlanners   % name -> @() planner
        policies       % name -> @() policy
        agentTypes     % name -> @(idx, pos, speed) agent
    end

    methods
        function obj = ComponentRegistry()
            obj.pathPlanners = containers.Map('KeyType','char','ValueType','any');
            obj.pathPlanners('Dijkstra Corners') = @() DijkstraCornerPlanner();
            obj.pathPlanners('Energy Efficient') = @() EnergyEfficientPlanner();

            obj.policies = containers.Map('KeyType','char','ValueType','any');
            obj.policies('Random Walk')       = @() RandomWalkPolicy();
            obj.policies('Battery Efficient') = @() BatteryEfficientPolicy();
            obj.policies('Classical RHC [1]') = @() RHCPolicy();
            obj.policies('Divergent RHC')     = @() DivergentRHCPolicy();
            obj.policies('Joint RHC')         = @() JointRHCPolicy();
            obj.policies('Joint RHC N')       = @() JointRHCPolicyN();

            obj.agentTypes = containers.Map('KeyType','char','ValueType','any');
            obj.agentTypes('Default Agent') = @(idx, pos, sp) DefaultAgent(idx, pos, sp);
            obj.agentTypes('Energy Agent')  = @(idx, pos, sp) EnergyAgent(idx, pos, sp);
            obj.agentTypes('Linear Agent')  = @(idx, pos, sp) LinearAgent(idx, pos, sp);
        end

        function names = keysFor(obj, kind)
            names = obj.mapFor(kind).keys;
        end

        function h = factoryFor(obj, kind, name)
            m = obj.mapFor(kind);
            h = m(name);
        end
    end

    methods (Access=private)
        function m = mapFor(obj, kind)
            switch lower(kind)
                case 'pathplanner', m = obj.pathPlanners;
                case 'policy',      m = obj.policies;
                case 'agenttype',   m = obj.agentTypes;
                otherwise
                    error('ComponentRegistry:UnknownKind', 'Unknown kind: %s', kind);
            end
        end
    end
end
