function T = paperTables(kind, varargin)
% The simulator paper's tables, as MATLAB tables:
%   paperTables("parameters", base)    every fixed parameter of the case studies, read from the
%                                      code (the paper states them in the text)
%   paperTables("policies", R)         Table IV: per policy and agent type, J(T) (mean and
%                                      standard deviation over the seeds), J_e, J_T and battery
%                                      energy (means)
%   paperTables("planners", R, base)   Table V: per planner, the total length of the planned
%                                      edges, the total of the Physical Agent's shortest transit
%                                      times over them, and J(T), J_e and energy of the run
%   paperTables("tradeoff", R)         Fig. 11 data: beta, energy weight, J(T), J_e, J_T
%   paperTables("battery", R)          Table VI: J(T), lowest SOC, charging sessions, time the
%                                      first battery ran empty
%   paperTables("validation", Ted, Teo) Tables II-III: ED-RHC and ED-ORHC against the JS
%                                      simulator (validateEDRHC, validateEDORHC outputs)
% R is the output of paperRuns.
switch kind
    case "parameters", T = parameterTable(varargin{:});
    case "policies",   T = meanTable(varargin{1}, ["J", "Je", "JT", "Energy"]);
    case "planners",   T = plannerTable(varargin{:});
    case "tradeoff",   T = tradeoffTable(varargin{:});
    case "battery",    T = meanTable(varargin{1}, ["J", "MinSOC", "Charges", "Empty"]);
    case "validation", T = validationTable(varargin{:});
    otherwise, error('paperTables:UnknownKind', 'Unknown table "%s"', kind);
end
end

function T = meanTable(R, metrics)
% Mean of each metric per condition (and the standard deviation of J over the seeds)
T = table(R.Labels', 'VariableNames', {'Condition'});
for m = metrics
    T.(m) = arrayfun(@(c) mean(R.Runs.(m)(R.Runs.Condition == c)), R.Labels');
end
T.Jstd = arrayfun(@(c) std(R.Runs.J(R.Runs.Condition == c)), R.Labels');
T.Runs = arrayfun(@(c) nnz(R.Runs.Condition == c), R.Labels');
end

function T = plannerTable(R, base)
T = meanTable(R, ["J", "Je", "Energy"]);
root = fileparts(which('MainUI'));
tmp = load(fullfile(root, 'Configurations', char(base.Layout)), 's');
reg = ComponentRegistry();
L = zeros(height(T), 1); Tr = L;
for k = 1:height(T)
    f = reg.factoryFor('pathplanner', char(R.Conditions(k).Params.Planner));
    m = ScenarioModel(f());
    m.importLayout(tmp.s);
    a = PhysicalAgent(1, [0 0], base.AgentSpeed);
    for e = m.edges
        L(k) = L(k) + sum(vecnorm(diff(e.curvePoints), 2, 2));
        Tr(k) = Tr(k) + a.shortestTransit(e.curvePoints);
    end
end
T.Length = L;
T.Transit = Tr;
end

function T = tradeoffTable(R)
T = meanTable(R, ["J", "Je", "JT"]);
T.Beta = arrayfun(@(c) c.Params.EnergyBeta, R.Conditions)';
end

function T = validationTable(Ted, Teo)
% One row per JS preset: ED-RHC (alpha 0.5 and nominal) and ED-ORHC (SO and FO-3) J values of
% the JS simulator and of this simulator, with the difference in percent (columns JsJT_SO,
% JsJT_FO3, ...)
Ted.Preset = erase(string(Ted.Preset), "JS_"); Ted.Alpha = string(Ted.Alpha);
presets = unique(Ted.Preset, 'stable');
T = table(presets, 'VariableNames', {'Preset'});
pick = @(tab, col, key, val, p) tab.(col)(tab.Preset == p & tab.(key) == val);
for p = 1:numel(presets)
    for a = ["alpha 0.5", "alpha nominal"]
        tag = erase(replace(a, "alpha ", ""), ".");          % "05", "nominal"
        T.("JsRHC_" + tag)(p) = pick(Ted, 'JsJ', 'Alpha', a, presets(p));
        T.("MatRHC_" + tag)(p) = pick(Ted, 'MatlabJ', 'Alpha', a, presets(p));
    end
end
if nargin > 1 && ~isempty(Teo)
    Teo.Preset = erase(string(Teo.Preset), "JS_"); Teo.Method = string(Teo.Method);
    for m = unique(Teo.Method, 'stable')'
        tag = erase(m, "-");                                % "SO", "FO3"
        T.("JsJT_" + tag) = nan(height(T), 1); T.("MatJT_" + tag) = nan(height(T), 1);
        for p = 1:numel(presets)
            k = Teo.Preset == presets(p) & Teo.Method == m;
            if any(k)
                T.("JsJT_" + tag)(p) = Teo.JsJT(k);
                T.("MatJT_" + tag)(p) = Teo.MatlabJT(k);
            end
        end
    end
end
end

function T = parameterTable(base)
% Values come from the classes and the layout file, so the list follows the code
tmp = load(fullfile(fileparts(which('MainUI')), 'Configurations', char(base.Layout)), 's');
s = tmp.s;
agent = PhysicalAgent(1, [0 0], base.AgentSpeed);
ctrl = TrackingController();
bat = Battery();
rw = RandomWalkPolicy();
ed = EDRHCPolicy();
dj = DijkstraCornerPlanner(); ee = EnergyEfficientPlanner(); ga = GridAStarPlanner();
rows = {
    "Simulation duration",                 "T",            sprintf('%g s', base.EndTime)
    "Time step",                           "dt",           sprintf('%g s', base.Dt)
    "Targets, edges, walls",               "",             sprintf('%d, %d, %d', size(s.targets, 1), size(s.edges, 1), size(s.walls, 1))
    "Agents (start targets)",              "",             sprintf('%d (%s)', numel(s.agentStarts), strjoin(string(s.agentStarts), ', '))
    "Uncertainty mode",                    "",             char(base.UncertaintyMode)
    "Growth rate (every target)",          "A_i",          sprintf('%g', base.A_base)
    "Decay rate per agent",                "B_i",          sprintf('%g', base.B)
    "Initial uncertainty",                 "R_i(0)",       sprintf('%g', unique(s.R0))
    "Detection radius",                    "",             sprintf('%g m', ScenarioModel().detectionRadius)
    "Maximum speed",                       "v_max",        sprintf('%g m/s', agent.state.maxSpeed)
    "Maximum acceleration",                "a_max",        sprintf('%g m/s^2', agent.state.maxAccel)
    "Tracking gains (Physical Agent)",     "K_p, K_d",     sprintf('%g, %g', ctrl.kp, ctrl.kd)
    "Share of the limits planned",         "",             sprintf('%g', agent.planMargin)
    "Corner buffer (Dijkstra, Energy Efficient)", "",      sprintf('%g, %g m', dj.cornerBuffer, ee.cornerBuffer)
    "Turn penalty (Energy Efficient)",     "kappa",        sprintf('%g', ee.turnPenalty)
    "Grid cell, clearance (Grid A*)",      "",             sprintf('%g m, %g m', ga.resolution, ga.clearance)
    "Corner arcs: largest radius, clearance", "",          sprintf('%g m, %g m', dj.filletRadius, dj.filletClearance)
    "Dwell, random walk",                  "tau_dwell",    sprintf('U(%g, %g) x time to empty the target', rw.dwellScale * (1 - rw.dwellSpread), rw.dwellScale * (1 + rw.dwellSpread))
    "Planning horizon (ED-RHC)",           "H",            sprintf('%g s', ed.H)
    "Next-target weight (ED-RHC)",         "alpha",        sprintf('1/|N_i|^2 (%s)', ed.alpha)
    "Energy weight (ED-ORHC)",             "",             sprintf('%g', ed.energyWeight)
    "Motion power",                        "P_0, alpha, gamma", sprintf('%g W, %.4g, %g', bat.P0, bat.alphaVel, bat.gammaAcc)
    "Processor power (moving, idle)",      "P_cpu",        sprintf('%g W, %g W', bat.cpuMove, bat.cpuIdle)
    "Communication power (active, standby)", "P_comp",     sprintf('%g W, %g W', bat.commActive, bat.commStandby)
    "Converter efficiency",                "eta_conv",     sprintf('%g', bat.eta)
    "Battery capacity",                    "Q",            sprintf('%g Wh (%g J)', bat.capacityWh, 3600 * bat.capacityWh)
    };
T = cell2table(rows, 'VariableNames', {'Parameter', 'Symbol', 'Value'});
T.Parameter = string(T.Parameter); T.Symbol = string(T.Symbol); T.Value = string(T.Value);
end
