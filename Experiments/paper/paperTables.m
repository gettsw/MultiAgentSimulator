function T = paperTables(kind, varargin)
% The simulator paper's tables, as MATLAB tables:
%   paperTables("parameters", base, seeds)  Table I: every fixed parameter, read from the code
%   paperTables("policy", R)                Table II: J(T) per run, mean, and std/mean per policy
%   paperTables("planner", R)               Table III: J(T), L, Theta, E_a per planner (mean over
%                                           runs, summed over agents) and the change in percent
%   paperTables("battery", R)               Table IV: final SOC and team energy per planner (mean
%                                           over runs) and the change in energy in percent
% R is the output of paperRuns; the first condition is the reference for the changes.
switch kind
    case "parameters", T = parameterTable(varargin{:});
    case "policy",     T = policyTable(varargin{:});
    case "planner",    T = changeTable(varargin{1}, ["J", "L", "Theta", "Ea"]);
    case "battery",    T = changeTable(varargin{1}, ["SOC", "Energy"]);
    otherwise, error('paperTables:UnknownKind', 'Unknown table "%s"', kind);
end
end

function T = policyTable(R)
nS = numel(R.Seeds);
J = zeros(numel(R.Labels), nS);
for c = 1:numel(R.Labels)
    J(c, :) = R.Runs.J(R.Runs.Condition == R.Labels(c))';
end
T = array2table(J, 'VariableNames', "R" + (1:nS));
T = addvars(T, R.Labels', 'Before', 1, 'NewVariableNames', 'Policy');
T.Mean = mean(J, 2);
T.StdOverMean = 100 * std(J, 0, 2) ./ T.Mean;   % percent
end

function T = changeTable(R, metrics)
% Mean of each metric per condition, plus a row with the change of the last condition
% relative to the first, in percent
T = table(R.Labels', 'VariableNames', {'Condition'});
for m = metrics
    T.(m) = arrayfun(@(c) mean(R.Runs.(m)(R.Runs.Condition == c)), R.Labels');
end
change = table("Change (%)", 'VariableNames', {'Condition'});
for m = metrics
    change.(m) = 100 * (T.(m)(end) - T.(m)(1)) / T.(m)(1);
end
T = [T; change];
end

function T = parameterTable(base, seeds)
% Values come from the classes and the layout file, so the table follows the code
tmp = load(fullfile(fileparts(which('MainUI')), 'Configurations', char(base.Layout)), 's');
s = tmp.s;
agent = DefaultAgent(1, [0 0], base.AgentSpeed);
ctrl = PathFollowerController();
rw = RandomWalkPolicy();
be = BatteryEfficientPolicy();
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
    "PD gains",                            "K_p, K_d",     sprintf('%g, %g', ctrl.kp, ctrl.kd)
    "Turn penalty (energy-efficient planner)", "kappa",    sprintf('%g', EnergyEfficientPlanner().turnPenalty)
    "Dwell, random walk",                  "tau_dwell",    sprintf('U(%g, %g) s', rw.dwellRange)
    "Dwell, battery efficient",            "tau_dwell",    sprintf('U(%g, %g) s; U(%g, %g) s below %g%% SOC', be.dwellRange, be.lowBatteryDwellRange, be.criticalBatteryThreshold)
    "Motion power",                        "P_0, alpha, gamma", sprintf('%g W, %.4g, %g', EnergyAgent.P0_BASE_MOTION, EnergyAgent.ALPHA_VEL, EnergyAgent.GAMMA_ACC)
    "Processor power (moving, idle)",      "P_cpu",        sprintf('%g W, %g W', EnergyAgent.CPU_MOVE, EnergyAgent.CPU_IDLE)
    "Communication power (active, standby)", "P_comp",     sprintf('%g W, %g W', EnergyAgent.COMP_ACTIVE, EnergyAgent.COMP_STANDBY)
    "Converter efficiency",                "eta_conv",     sprintf('%g', EnergyAgent.ETA_CONV)
    "Battery capacity",                    "Q",            sprintf('%g Wh (%g J)', EnergyAgent.BATTERY_E_Wh, 3600 * EnergyAgent.BATTERY_E_Wh)
    "Runs per condition (seeds)",          "",             sprintf('%d (%s)', numel(seeds), strjoin(string(seeds), ', '))
    };
T = cell2table(rows, 'VariableNames', {'Parameter', 'Symbol', 'Value'});
T.Parameter = string(T.Parameter); T.Symbol = string(T.Symbol); T.Value = string(T.Value);
end
