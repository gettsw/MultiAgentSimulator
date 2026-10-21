classdef ComponentRegistry < handle
    % Central dictionary of modular components.
    % To register a new algorithm, add one line to the constructor.
    properties
        pathPlanners   % name -> @() planner
        policies       % name -> @() policy
        agentTypes     % name -> @(idx, pos, speed) agent
        aliases        % older agent type names -> @(idx, pos, speed) agent; resolved by factoryFor
                       % but not listed by keysFor (so not offered in the GUI)
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
            obj.policies('ED-RHC')            = @() EDRHCPolicy();
            obj.policies('ED-ORHC-SO')        = @() EDRHCPolicy(Transit="SO");
            obj.policies('ED-ORHC-FO')        = @() EDRHCPolicy(Transit="FO");

            obj.agentTypes = containers.Map('KeyType','char','ValueType','any');
            obj.agentTypes('Abstract Agent')   = @(idx, pos, sp) AbstractAgent(idx, pos, sp);
            obj.agentTypes('Physical Agent') = @(idx, pos, sp) PhysicalAgent(idx, pos, sp);

            % Agent types before the consolidation of 2026-10-20, kept so older scripts and
            % layouts run unchanged (e.g. William's Joint RHC studies use 'Linear Agent')
            obj.aliases = containers.Map('KeyType','char','ValueType','any');
            obj.aliases('Linear Agent')       = @(idx, pos, sp) AbstractAgent(idx, pos, sp);
            obj.aliases('Second-Order Agent') = @(idx, pos, sp) AbstractAgent(idx, pos, sp, Profile="SO");
            obj.aliases('Default Agent')      = @(idx, pos, sp) PhysicalAgent(idx, pos, sp);
            obj.aliases('Energy Agent')       = @(idx, pos, sp) PhysicalAgent(idx, pos, sp);
        end

        function names = keysFor(obj, kind)
            names = obj.mapFor(kind).keys;
        end

        function h = factoryFor(obj, kind, name)
            m = obj.mapFor(kind);
            if ~isKey(m, name) && strcmpi(kind, 'agenttype') && isKey(obj.aliases, name)
                h = obj.aliases(name);
            else
                h = m(name);
            end
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
