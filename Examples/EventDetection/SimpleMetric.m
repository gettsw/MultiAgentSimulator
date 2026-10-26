classdef SimpleMetric < Metric
    % SIMPLEMETRIC  A minimal new performance metric: the share of events missed.
    %
    % Part of the beginner's example in Examples/EventDetection (a new monitoring problem built
    % from four small subclasses: SimpleTarget, SimpleAgent, SimplePolicy and SimpleMetric).
    %
    % The metric: J_M(t) = sum_i M_i(t) / sum_i (M_i(t) + D_i(t)), where M_i and D_i are the
    % numbers of events missed and detected at target i by time t (the counters missed and
    % detected of SimpleTarget). It is NaN when the scenario has no SimpleTarget targets.
    %
    % How it plugs in:
    %   - It subclasses Metric, the abstract base of the performance metrics, and implements
    %     value(model, simTime), its one required method. The model keeps a list of metrics
    %     (ScenarioModel.metrics): the first is the objective plotted over time in the GUI's
    %     objective window, and runTrial returns the final value of each.
    %   - A metric that accumulates over a run also implements update(model, simTime, dt),
    %     called after every step, and reset(model), called when a run starts. This one needs
    %     neither: the targets keep the counters.
    %   - label names the objective plot's y-axis; plotMax is the axis' initial top (a share
    %     is at most 1).
    %   - Registered in Components/ComponentRegistry.m as
    %       obj.metrics('Simple Metric (example)') = @() SimpleMetric();
    %     so the GUI's Objective dropdown and runTrial's Metrics parameter can select it
    %     (runTrial output: out.metricValues.SimpleMetric).
    %
    % To make it your own: report the mean delay from an event's start to its detection, or
    % the worst share over the targets. AverageUncertainty (Components/Metrics) is the default
    % objective J(t).
    methods
        function obj = SimpleMetric()
            obj.label = "Share of events missed";
            obj.plotMax = 1;
        end

        function v = value(~, model, ~)
            tg = model.targets(arrayfun(@(t) isa(t, 'SimpleTarget'), model.targets));
            v = NaN;
            if isempty(tg), return; end
            n = sum([tg.missed]) + sum([tg.detected]);
            v = sum([tg.missed]) / max(n, 1);
        end
    end
end
