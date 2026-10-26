classdef SimpleTarget < Target
    % SIMPLETARGET  A minimal new target type: random events that expire unless detected.
    %
    % Part of the beginner's example in Examples/EventDetection (a new monitoring problem built
    % from four small subclasses: SimpleTarget, SimpleAgent, SimplePolicy and SimpleMetric).
    %
    % The problem: incidents (events) occur at random at the target, at eventRate per second (a
    % Poisson process, sampled once per time step), and each one lasts lifetime seconds. An
    % agent resting at the target detects every event in progress there; an event that ends
    % before an agent arrives is missed. The counters detected and missed record both outcomes.
    %
    % How it plugs in:
    %   - It subclasses Target, the default target type ("Uncertainty Target", dR/dt = A - B N).
    %   - It overrides updateUncertainty(dt, simTime, mode), which the model calls for every
    %     target at every step after it has found the agents at rest at each target
    %     (residingAgents). The uncertainty mode is ignored here.
    %   - R is the target's state: here the number of events in progress. The target's bar
    %     shows R, and the default objective J(t) averages the sum of R over the targets.
    %   - It overrides reset (calling reset@Target first) to clear its own state when a run
    %     restarts.
    %   - Registered in Components/ComponentRegistry.m as
    %       obj.targetTypes('Simple Target (example)') = @(idx, pos) SimpleTarget(idx, pos);
    %     so the GUI's Target Type dropdown, ScenarioModel.setTargetType and runTrial's
    %     TargetType parameter can select it. The Museum layout stores it as its target type.
    %
    % To make it your own: change updateUncertainty (e.g. events that grow more urgent with
    % age, or a detection that takes time), and give the targets different eventRate values.
    properties
        eventRate = 0.02   % events per second (a Poisson process)
        lifetime = 25      % seconds an event lasts
        startTimes = []    % start times of the events in progress
        detected = 0       % events detected so far
        missed = 0         % events missed so far
    end

    methods
        function obj = SimpleTarget(index, position)
            obj@Target(index, position);
        end

        function updateUncertainty(obj, dt, simTime, ~)
            if rand() < obj.eventRate * dt                     % a new event
                obj.startTimes(end+1) = simTime;
            end
            over = simTime - obj.startTimes >= obj.lifetime;   % events that ended undetected
            obj.missed = obj.missed + nnz(over);
            obj.startTimes(over) = [];
            if ~isempty(obj.residingAgents)                    % an agent is here: detect all
                obj.detected = obj.detected + numel(obj.startTimes);
                obj.startTimes = [];
            end
            obj.R = numel(obj.startTimes);
        end

        function reset(obj)
            reset@Target(obj);
            obj.startTimes = []; obj.detected = 0; obj.missed = 0;
            obj.R = 0;
        end
    end
end
