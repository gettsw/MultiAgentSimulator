function m = trialMetrics(log)
% Per-agent metrics of a logged run (runTrial with Log = true), each 1 x nAgents:
%   L      path length                      (pathLength)
%   Theta  total heading change [rad]       (headingChange)
%   Ea     integrated squared acceleration  (accelCost)
%   SOC    final state of charge [%]        (NaN for agents without a battery)
%   Energy energy drawn from the battery [J] by the end (NaN without a battery)
nA = size(log.pos, 2);
m = struct('L', zeros(1, nA), 'Theta', zeros(1, nA), 'Ea', zeros(1, nA), ...
    'SOC', 100 * log.soc(end, :), 'Energy', log.energy(end, :));
for a = 1:nA
    m.L(a) = pathLength(squeeze(log.pos(:, a, :)));
    m.Theta(a) = headingChange(log.ori(:, a));
    m.Ea(a) = accelCost(squeeze(log.acc(:, a, :)), log.dt);
end
end
