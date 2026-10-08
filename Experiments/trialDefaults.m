function p = trialDefaults()
% Default trial parameters. Shared by runTrial (fills unset parameters) and
% analyzeResults (knows which table columns are parameters, fills missing ones).
p = struct( ...
    'Policy', "Joint RHC", ...                    % "Joint RHC" | "Classical RHC [1]"
    'UncertaintyMode', "Dynamic Uncertainty", ... % | "Static Priority Target" | "Static Uniform"
    'Layout', "ExampleLayout1.mat", ...           % file in Configurations/
    'NumAgents', 2, ...                           % ignored when AgentStarts is given
    'AgentStarts', "auto", ...                    % "auto" = spread evenly, or target indices e.g. [1 5]
    'B', 20, ...                                  % decay rate per agent
    'A_base', 1, ...                              % baseline growth (all targets but target 1)
    'A_priority', 5, ...                          % target 1 baseline (priority + dynamic modes)
    'A_spike', 28, ...                            % growth during a spike (dynamic mode)
    'SpikeDuration', 10, ...
    'SpikeInterval', 250, ...
    'SpikeStagger', 50, ...                       % spike offset between consecutive targets
    'AgentSpeed', 15, ...
    'EndTime', 500, ...
    'Dt', 0.05);
end
