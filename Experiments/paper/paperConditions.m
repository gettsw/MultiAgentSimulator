function cond = paperConditions(name, opts)
% Conditions of the simulator paper's experiments (Sec. IV), as input to paperRuns. Each
% condition overrides paperDefaults (Battery Efficient, Dijkstra Corners, Physical Agent):
%   "policy"   Table II, Figs. 10-11: Random Walk vs Battery Efficient
%   "planner"  Table III, Fig. 12: Dijkstra Corners vs Energy Efficient (Battery Efficient)
%   "battery"  Table IV, Fig. 13: as "planner" (every agent now carries a battery)
% NoDwell = true sets the policies' dwell to zero, which recreates the conditions of the draft's
% runs (made before ScenarioModel honoured cmd.dwellSeconds).
arguments
    name (1,1) string
    opts.NoDwell (1,1) logical = false
end
switch name
    case "policy"
        labels = ["Random Walk", "Battery Efficient"];
        params = {struct('Policy', "Random Walk"), struct('Policy', "Battery Efficient")};
    case "planner"
        labels = ["Dijkstra", "Energy Efficient"];
        params = {struct('Planner', "Dijkstra Corners"), struct('Planner', "Energy Efficient")};
    case "battery"
        labels = ["Dijkstra", "Energy Efficient"];
        params = {struct('Planner', "Dijkstra Corners", 'AgentType', "Physical Agent"), ...
                  struct('Planner', "Energy Efficient", 'AgentType', "Physical Agent")};
    otherwise
        error('paperConditions:UnknownName', 'Unknown experiment "%s"', name);
end
if opts.NoDwell
    for k = 1:numel(params)
        params{k}.Policy_dwellRange = [0 0];   % Battery Efficient
        params{k}.Policy_dwellScale = 0;       % Random Walk
    end
end
cond = struct('Label', num2cell(labels), 'Params', params);
end
