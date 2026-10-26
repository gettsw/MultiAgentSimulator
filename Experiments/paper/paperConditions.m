function cond = paperConditions(name)
% Conditions of the simulator paper's case studies (Sec. VII), as input to paperRuns. Each
% condition overrides paperDefaults (reference lab scenario, ED-RHC, Dijkstra Corners, Physical
% Agent, T = 500 s) with Params, and runs once per seed in Seeds (Random Walk is the only
% stochastic policy, so the others run once):
%   "policies"  Table IV, Fig. 9: Random Walk, ED-RHC, ED-ORHC-SO and ED-ORHC-FO-3, each with
%               Abstract and Physical Agents
%   "planners"  Table V, Fig. 10: Dijkstra Corners, Energy Efficient and Grid A* (ED-RHC,
%               Physical Agents)
%   "tradeoff"  Fig. 11: ED-ORHC-SO with the energy weight set by beta (App. A of the
%               energy-aware paper), Abstract Agents with the speed and acceleration limits
%   "battery"   Table VI, Fig. 12: office floor with a charging dock, 2.5 Wh batteries that stop
%               the agents when empty, T = 1500 s: Battery-Aware ED-RHC, ED-RHC and Random Walk,
%               with Abstract and Physical Agents, and ED-RHC with unlimited batteries
arguments
    name (1,1) string
end
rwSeeds = 1:10;
switch name
    case "policies"
        pols = ["Random Walk", "ED-RHC", "ED-ORHC-SO", "ED-ORHC-FO-3"];
        types = ["Abstract Agent", "Physical Agent"];
        labels = strings(0); params = {}; seeds = {};
        for t = types
            for p = pols
                labels(end+1) = p + " / " + extractBefore(t, " Agent"); %#ok<AGROW>
                params{end+1} = struct('Policy', p, 'AgentType', t); %#ok<AGROW>
                if p == "Random Walk", seeds{end+1} = rwSeeds; else, seeds{end+1} = 1; end %#ok<AGROW>
            end
        end
    case "planners"
        names = ["Dijkstra Corners", "Energy Efficient", "Grid A*"];
        labels = names;
        params = arrayfun(@(n) struct('Planner', n), names, 'UniformOutput', false);
        seeds = repmat({1}, 1, numel(names));
    case "tradeoff"
        betas = [0.02 0.05 0.1 0.2 0.35 0.5 0.7 0.9];
        labels = "beta = " + string(betas);
        params = arrayfun(@(b) struct('Policy', "ED-ORHC-SO", 'AgentType', "Abstract Agent", ...
            'EnergyBeta', b, 'Policy_speedLimit', 15, 'Policy_accelLimit', 15), betas, 'UniformOutput', false);
        seeds = repmat({1}, 1, numel(betas));
    case "battery"
        dock = struct('Layout', "OfficeDock.mat", 'EndTime', 1500, 'Agent_batteryWh', 2.5, ...
            'Agent_stopWhenEmpty', true);
        cases = {
            "Battery-Aware ED-RHC / Abstract", "Battery-Aware ED-RHC", "Abstract Agent", true
            "ED-RHC / Abstract",               "ED-RHC",               "Abstract Agent", true
            "Random Walk / Abstract",          "Random Walk",          "Abstract Agent", true
            "ED-RHC unlimited / Abstract",     "ED-RHC",               "Abstract Agent", false
            "Battery-Aware ED-RHC / Physical", "Battery-Aware ED-RHC", "Physical Agent", true
            "ED-RHC / Physical",               "ED-RHC",               "Physical Agent", true
            "ED-RHC unlimited / Physical",     "ED-RHC",               "Physical Agent", false};
        labels = string(cases(:, 1))';
        params = cell(1, size(cases, 1));
        for k = 1:size(cases, 1)
            p = dock;
            p.Policy = cases{k, 2}; p.AgentType = cases{k, 3};
            if ~cases{k, 4}, p.Agent_batteryWh = 1e6; p.Agent_stopWhenEmpty = false; end
            params{k} = p;
        end
        seeds = repmat({1}, 1, numel(labels));
    otherwise
        error('paperConditions:UnknownName', 'Unknown experiment "%s"', name);
end
cond = struct('Label', num2cell(labels), 'Params', params, 'Seeds', seeds);
end
