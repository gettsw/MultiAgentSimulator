function files = writePaperOutputs(outDir, S)
% Writes the regenerated results of the simulator paper into outDir:
%   runs.csv                    every run of every case study (one row per run)
%   table_validation_rhc.tex    Table II: ED-RHC against the JS simulator
%   table_validation_energy.tex Table III: ED-ORHC (SO and FO-3) against the JS simulator
%   table_policies.tex          Table IV: policies and agent types
%   table_planners.tex          Table V: path planners
%   table_battery.tex           Table VI: battery-aware ED-RHC
%   macros.tex                  \newcommand values quoted in the text (prefix \pr)
% S has fields base (paperDefaults), validation (paperTables("validation", ...)), and
% policies, planners, tradeoff, battery (paperRuns outputs).
if ~isfolder(outDir), mkdir(outDir); end
names = ["runs.csv", "table_validation_rhc.tex", "table_validation_energy.tex", ...
    "table_policies.tex", "table_planners.tex", "table_battery.tex", "macros.tex"];
files = fullfile(outDir, names);

exps = ["policies", "planners", "tradeoff", "battery"];
runs = cell(numel(exps), 1);
for k = 1:numel(exps)
    r = S.(exps(k)).Runs;
    runs{k} = addvars(r, repmat(exps(k), height(r), 1), 'Before', 1, 'NewVariableNames', 'Experiment');
end
writetable(vertcat(runs{:}), files(1));

% ---- Table II: ED-RHC against the JS simulator ----
V = S.validation;
d = @(a, b) 100 * (a - b) ./ b;
cells = [presetName(V.Preset), f2(V.JsRHC_05), f2(V.MatRHC_05), sgn(d(V.MatRHC_05, V.JsRHC_05)), ...
         f2(V.JsRHC_nominal), f2(V.MatRHC_nominal), sgn(d(V.MatRHC_nominal, V.JsRHC_nominal))];
writelines(tabular("|l|c|c|c|c|c|c|", ...
    ["\multirow{2}{*}{Problem} & \multicolumn{3}{c|}{RHC} & \multicolumn{3}{c|}{RHC-$\alpha$}"; ...
     " & JS & Ours & Diff. (\%) & JS & Ours & Diff. (\%)"], cells), files(2));

% ---- Table III: ED-ORHC against the JS simulator ----
cells = [presetName(V.Preset), f2(V.JsJT_SO), f2(V.MatJT_SO), sgn(d(V.MatJT_SO, V.JsJT_SO)), ...
         f2(V.JsJT_FO3), f2(V.MatJT_FO3), sgn(d(V.MatJT_FO3, V.JsJT_FO3))];
writelines(tabular("|l|c|c|c|c|c|c|", ...
    ["\multirow{2}{*}{Problem} & \multicolumn{3}{c|}{SO} & \multicolumn{3}{c|}{FO-3}"; ...
     " & JS & Ours & Diff. (\%) & JS & Ours & Diff. (\%)"], cells), files(3));

% ---- Table IV: policies and agent types ----
P = paperTables("policies", S.policies);
pol = extractBefore(P.Condition, " /"); agt = extractAfter(P.Condition, "/ ");
J = compose("%.1f", P.J);
rw = P.Runs > 1;
J(rw) = J(rw) + " $\pm$ " + compose("%.1f", P.Jstd(rw));
JT = P.JT; JT(~startsWith(pol, "ED-ORHC")) = NaN;           % J_T only for the energy-aware policies
cells = [pol, agt, J, f0(P.Je ./ 1000), f1(JT), f1(P.Energy)];
writelines(tabular("|l|l|c|c|c|c|", ["Policy", "Agents", "$J(T)$", ...
    "$J_e$ ($10^3$)", "$J_T$", "Energy (kJ)"], cells), files(4));

% ---- Table V: path planners ----
Q = paperTables("planners", S.planners, S.base);
cells = [Q.Condition, boldBest(Q.Length, "%.1f", @min), boldBest(Q.Transit, "%.1f", @min), ...
         boldBest(Q.J, "%.1f", @min), boldBest(Q.Je / 1000, "%.0f", @min), boldBest(Q.Energy, "%.1f", @min)];
writelines(tabular("|l|c|c|c|c|c|", ["Planner", "Length (m)", "Transit (s)", "$J(T)$", ...
    "$J_e$ ($10^3$)", "Energy (kJ)"], cells), files(5));

% ---- Table VI: battery-aware ED-RHC ----
B = paperTables("battery", S.battery);
pol = extractBefore(B.Condition, " /"); agt = extractAfter(B.Condition, "/ ");
empty = compose("%.0f", B.Empty); empty(isnan(B.Empty)) = "--";
cells = [pol, agt, f1(B.J), f1(B.MinSOC), compose("%d", round(B.Charges)), empty];
writelines(tabular("|l|l|c|c|c|c|", ["Policy", "Agents", "$J(T)$", stack("Lowest", "SOC (\%)"), ...
    "Charges", stack("First", "empty (s)")], cells), files(6));

% ---- Macros quoted in the text ----
T = paperTables("tradeoff", S.tradeoff);
allRHC = [d(V.MatRHC_05, V.JsRHC_05); d(V.MatRHC_nominal, V.JsRHC_nominal)];
allEn = [d(V.MatJT_SO, V.JsJT_SO); d(V.MatJT_FO3, V.JsJT_FO3)];
pick = @(tab, label, col) tab.(col)(tab.Condition == label);
mac = [
    macro("prValRHCMin", sprintf('%+.2f', min(allRHC)))
    macro("prValRHCMax", sprintf('%+.2f', max(allRHC)))
    macro("prValRHCMeanAbs", sprintf('%.2f', mean(abs(allRHC))))
    macro("prValEnMin", sprintf('%+.2f', min(allEn)))
    macro("prValEnMax", sprintf('%+.2f', max(allEn)))
    macro("prValEnMeanAbs", sprintf('%.2f', mean(abs(allEn))))
    macro("prRWSeeds", sprintf('%d', max(P.Runs)))
    macro("prJRWAbs", sprintf('%.1f', pick(P, "Random Walk / Abstract", 'J')))
    macro("prJRWPhys", sprintf('%.1f', pick(P, "Random Walk / Physical", 'J')))
    macro("prJEDAbs", sprintf('%.1f', pick(P, "ED-RHC / Abstract", 'J')))
    macro("prJEDPhys", sprintf('%.1f', pick(P, "ED-RHC / Physical", 'J')))
    macro("prJSOAbs", sprintf('%.1f', pick(P, "ED-ORHC-SO / Abstract", 'J')))
    macro("prJSOPhys", sprintf('%.1f', pick(P, "ED-ORHC-SO / Physical", 'J')))
    macro("prJFOThreePhys", sprintf('%.1f', pick(P, "ED-ORHC-FO-3 / Physical", 'J')))
    macro("prPolicyReduction", sprintf('%.1f', 100 * (1 - pick(P, "ED-RHC / Physical", 'J') / pick(P, "Random Walk / Physical", 'J'))))
    macro("prPhysicalIncrease", sprintf('%.1f', 100 * (pick(P, "ED-RHC / Physical", 'J') / pick(P, "ED-RHC / Abstract", 'J') - 1)))
    macro("prPlannerLengthDJ", sprintf('%.1f', Q.Length(1)))
    macro("prPlannerLengthEE", sprintf('%.1f', Q.Length(2)))
    macro("prPlannerLengthGA", sprintf('%.1f', Q.Length(3)))
    macro("prPlannerTransitDJ", sprintf('%.1f', Q.Transit(1)))
    macro("prPlannerTransitEE", sprintf('%.1f', Q.Transit(2)))
    macro("prPlannerTransitGA", sprintf('%.1f', Q.Transit(3)))
    macro("prPlannerJDJ", sprintf('%.1f', Q.J(1)))
    macro("prPlannerJEE", sprintf('%.1f', Q.J(2)))
    macro("prPlannerJGA", sprintf('%.1f', Q.J(3)))
    macro("prTradeBetaMin", sprintf('%g', T.Beta(1)))
    macro("prTradeBetaMax", sprintf('%g', T.Beta(end)))
    macro("prTradeJMin", sprintf('%.1f', T.J(1)))
    macro("prTradeJMax", sprintf('%.1f', T.J(end)))
    macro("prTradeJeMin", sprintf('%.0f', T.Je(1) / 1000))
    macro("prTradeJeMax", sprintf('%.0f', T.Je(end) / 1000))
    macro("prBatJBAAbs", sprintf('%.1f', pick(B, "Battery-Aware ED-RHC / Abstract", 'J')))
    macro("prBatJBAPhys", sprintf('%.1f', pick(B, "Battery-Aware ED-RHC / Physical", 'J')))
    macro("prBatJEDAbs", sprintf('%.0f', pick(B, "ED-RHC / Abstract", 'J')))
    macro("prBatJEDPhys", sprintf('%.0f', pick(B, "ED-RHC / Physical", 'J')))
    macro("prBatJUnlAbs", sprintf('%.1f', pick(B, "ED-RHC unlimited / Abstract", 'J')))
    macro("prBatJUnlPhys", sprintf('%.1f', pick(B, "ED-RHC unlimited / Physical", 'J')))
    macro("prBatMinSOCBAPhys", sprintf('%.1f', pick(B, "Battery-Aware ED-RHC / Physical", 'MinSOC')))
    macro("prBatChargesBAPhys", sprintf('%d', round(pick(B, "Battery-Aware ED-RHC / Physical", 'Charges'))))
    macro("prBatEmptyEDPhys", sprintf('%.0f', pick(B, "ED-RHC / Physical", 'Empty')))
    macro("prBatEmptyRW", sprintf('%.0f', pick(B, "Random Walk / Abstract", 'Empty')))];
writelines(["% Values regenerated by PaperResults.mlx; do not edit by hand"; mac], files(7));
end

function lines = tabular(spec, header, cells)
% header: a row of column titles, or a column of finished header lines (with & inside)
if size(header, 1) == 1, header = join(header, " & "); end
head = header(:) + " \\";
if numel(head) > 1, head(1:end-1) = head(1:end-1) + " \cline{2-" + size(cells, 2) + "}"; end
lines = ["\begin{tabular}{" + spec + "}"; "\hline"; head; "\hline"; join(cells, " & ", 2) + " \\"; ...
         "\hline"; "\end{tabular}"];
end

function s = stack(top, bottom)
% Column title on two lines, to keep a table within one column
s = "\begin{tabular}{@{}c@{}}" + top + " \\ " + bottom + "\end{tabular}";
end

function s = presetName(p)
% "1Agent_3T" -> "1 agent, 3 targets"
s = regexprep(string(p), '^(\d)Agent_(\d)T$', '$1-agent, $2-target');
s = regexprep(s, '^(\d)Agent_(\w+)$', '$1-agent, $2');
s = replace(s, ["General", "Maze", "Room"], ["general", "maze", "room"]);
end

function s = f0(x), s = compose("%.0f", x); s(isnan(x)) = "--"; end
function s = f1(x), s = compose("%.1f", x); s(isnan(x)) = "--"; end
function s = f2(x), s = compose("%.2f", x); s(isnan(x)) = "--"; end
function s = sgn(x), s = compose("%+.2f", x); s(isnan(x)) = "--"; end

function s = boldBest(x, fmt, pick)
% Formatted values with the best one (pick = @min or @max) in bold
s = compose(fmt, x);
[~, i] = pick(x);
s(i) = "\textbf{" + s(i) + "}";
end

function m = macro(name, value)
m = "\newcommand{\" + name + "}{" + value + "}";
end
