function T = paperComparison(current, draftConditions)
% Numbers reported in the draft of the simulator paper next to the regenerated ones.
% current and draftConditions are structs with fields policy, planner and battery (paperRuns
% outputs), run with the current code and with paperConditions(..., NoDwell=true).
% Draft values: Paper_PerMonSimulator/ArXiv.tex as of 2026-10-20 (Tables II-IV), produced by
% hand in the GUI before Random Walk and Battery Efficient dwelled at targets.
draft = [ ...
    "Table II",  "J(T), Random Walk, mean",            1534.25
    "Table II",  "J(T), Random Walk, std/mean (%)",    3.15
    "Table II",  "J(T), Battery Efficient, mean",      950.76
    "Table II",  "J(T), Battery Efficient, std/mean (%)", 0
    "Table III", "J(T), Dijkstra",                     950.8
    "Table III", "J(T), Energy Efficient",             995.4
    "Table III", "L, Dijkstra",                        7159
    "Table III", "L, Energy Efficient",                7101
    "Table III", "Theta, Dijkstra",                    1167
    "Table III", "Theta, Energy Efficient",            1154
    "Table III", "E_a, Dijkstra",                      68656
    "Table III", "E_a, Energy Efficient",              67829
    "Table IV",  "SOC (%), Dijkstra",                  74.9
    "Table IV",  "SOC (%), Energy Efficient",          75.0
    "Table IV",  "Energy (J), Dijkstra",               117468
    "Table IV",  "Energy (J), Energy Efficient",       117000];
T = table(draft(:, 1), draft(:, 2), double(draft(:, 3)), ...
    'VariableNames', {'Table', 'Quantity', 'Draft'});
T.DraftConditions = values(draftConditions);
T.Current = values(current);
T.DraftConditionsVsDraft = 100 * (T.DraftConditions - T.Draft) ./ T.Draft;   % percent
end

function v = values(S)
% The regenerated numbers in the row order of the draft table above
p = paperTables("policy", S.policy);
pl = paperTables("planner", S.planner);
b = paperTables("battery", S.battery);
v = [p.Mean(1); p.StdOverMean(1); p.Mean(2); p.StdOverMean(2); ...
     pl.J(1); pl.J(2); pl.L(1); pl.L(2); pl.Theta(1); pl.Theta(2); pl.Ea(1); pl.Ea(2); ...
     b.SOC(1); b.SOC(2); b.Energy(1); b.Energy(2)];
end
