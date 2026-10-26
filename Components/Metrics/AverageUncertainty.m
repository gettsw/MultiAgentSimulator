classdef AverageUncertainty < Metric
    % The persistent monitoring objective J(t) = (1/t) int_0^t sum_i R_i(s) ds, the time average
    % of the total target uncertainty (the model integrates it every step)
    methods
        function obj = AverageUncertainty()
            obj.label = "J(t) = (1/t) \int_0^t \Sigma_i R_i ds";
        end

        function v = value(~, model, simTime)
            v = model.cumUncertaintyIntegral / max(eps, simTime);
        end
    end
end
