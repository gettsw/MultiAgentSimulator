function M = collabMaps()
% The simulator's maps in the collaboration paper's map study: layout files in Configurations/
% with a background image, walls and per-target growth rates A_i (runTrial TargetRates =
% "layout"). The GUI's Scenario dropdown lists them under the same names (scenarioCatalog).
M = table(["ReportLayout"; "OfficeFloor"; "CityPatrol"; "HarborPatrol"; "Campus"], ...
    ["Lab floor"; "Office floor"; "City patrol"; "Harbor patrol"; "Campus"], ...
    'VariableNames', {'Layout', 'Name'});
end
