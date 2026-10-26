classdef (Abstract) Metric < handle
    % Base class of the performance metrics. The model keeps a list of metrics
    % (ScenarioModel.metrics): it calls reset(model) when a run starts and update(model, simTime,
    % dt) after every step, and value(model, simTime) gives the metric so far. The first metric
    % in the list is the objective plotted over time in the GUI; runTrial returns the final
    % value of each (parameter Metrics, output metricValues).
    % A new metric implements value (and update, if it accumulates over the run), sets label,
    % and is registered in ComponentRegistry (metrics). AverageUncertainty is the default
    % objective; Examples/EventDetection/SimpleMetric.m is a small example.
    properties
        label string = ""      % axis label of the objective plot
        plotMax = 10           % initial top of the objective plot's y-axis (grows with the data)
    end

    methods (Abstract)
        v = value(obj, model, simTime)
    end

    methods
        function reset(~, ~)
            % Nothing to clear by default
        end

        function update(~, ~, ~, ~)
            % Nothing to accumulate by default
        end
    end
end
