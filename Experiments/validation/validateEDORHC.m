function T = validateEDORHC(opts)
% Compares the energy-aware policies ED-ORHC SO and ED-ORHC FO-3 (SecondOrderAgent) with the
% legacy JavaScript simulator (Simulator_PerMon_JS, methods 9 and 11), which produced the
% results of Welikala and Cassandras, arXiv 2102.12963 (CDC 2021). Presets and settings as in
% validateEDRHC (Configurations/Legacy/, lengths / 6, A = 1, B = 10, R(0) = 0.5, T = 500 s,
% dt = 0.01 s, H = 250), with the JS conventions of the energy-aware methods:
%   clearFirst = true   every visit clears the target (no RHCP1; plans clear j)
%   energyWeight        2 x 36 x 213.3e-6: the JS weight 213.3e-6 (600 x 600 space) is x36 on
%                       our canvas, and the JS picks rho by minimizing the alpha-weighted sensing
%                       cost plus weight * E, which our RHCP3 objective (2 x weighted sensing
%                       cost + weight * E, the paper's objective at alpha = 0.5) matches with
%                       twice the weight
% J_T is reported with the JS weight, J_e on the JS scale (x36), v_max and u_max x6.
%   T = validateEDORHC()                 all presets, both methods (~10 min)
%   T = validateEDORHC(Presets=[1 4])    a subset
% The JS values were read from simulateHybridSystemFast on 2026-10-20 (RHCalpha2 = 2.1333e-4).
% 2-Agent 5T, 3-Agent Maze, 3-Agent General and 3-Agent Room are PC2, PC6, PC1 and PC7 of the
% paper's Table I.
arguments
    opts.Presets (1,:) double = 1:8
    opts.Methods (1,:) string = ["SO", "FO-3"]
end
files = ["JS_1Agent_3T", "JS_1Agent_4T", "JS_1Agent_5T", "JS_2Agent_5T", ...
    "JS_3Agent_9T", "JS_3Agent_Maze", "JS_3Agent_General", "JS_3Agent_Room"];
% JS [J_s, J_e, J_T, v_max, u_max] per preset; SO (method 9), then FO-3 (method 11)
js.SO = [
    23.3392   585855.6  148.3218   81.335   71.016
    47.8605   518547.4  158.4840   73.363   71.761
    94.2482   700274.3  243.6401  118.839  101.624
    32.2304  1551152.8  363.1430  127.485   87.352
    89.9028  1441525.2  397.4282   74.138   73.286
   131.0053  1412894.4  432.4228   94.771   75.382
   103.8396  1657366.9  457.4112   84.563   87.911
   107.2108  1808715.0  493.0700  106.600  113.376];
js.FO3 = [
    24.1591   582326.2  148.3887   78.979   50.222
    49.6020   519518.0  160.4325   71.242   50.754
    97.5396   694394.8  245.6772  115.395   71.909
    33.5091  1538084.7  361.6338  123.795   61.973
   102.3072  1457585.1  413.2587   72.503   52.567
   142.1941  1401724.6  441.2287   91.135   53.322
   108.2840  1639455.9  458.0346   82.112   62.223
   111.3817  1802620.0  495.9407   98.134   80.179];
w = 2.133333333333333e-4;
rows = {};
for k = opts.Presets
    s = load(fullfile(fileparts(which('MainUI')), 'Configurations', 'Legacy', files(k) + ".mat"), 's');
    for method = opts.Methods
        p = struct('Layout', fullfile("Legacy", files(k) + ".mat"), 'AgentStarts', "layout", ...
            'Policy', "ED-ORHC " + method, 'Planner', "Dijkstra Corners", 'AgentType', "Second-Order Agent", ...
            'UncertaintyMode', "Static Uniform", 'A_base', 1, 'B', 10, 'EndTime', 500, 'Dt', 0.01, ...
            'DetectionRadius', 1e-6, 'Policy_H', 250, 'Policy_clearFirst', true, ...
            'Policy_energyWeight', 2 * 36 * w, 'AgentSpeed', s.s.agentSpeed(1));
        tic;
        out = runTrial(p);
        ref = js.(erase(method, "-"))(k, :);
        mine = [out.finalJ, 36 * out.Je, 36 * w * out.Je + out.finalJ, 6 * out.vmax, 6 * out.umax];
        rows(end+1, :) = {erase(files(k), "JS_"), method, ref(3), mine(3), pct(mine(3), ref(3)), ...
            ref(1), mine(1), pct(mine(1), ref(1)), ref(2), mine(2), pct(mine(2), ref(2)), ...
            pct(mine(4), ref(4)), pct(mine(5), ref(5)), toc}; %#ok<AGROW>
        fprintf(['%-14s %-5s J_T JS %7.2f MATLAB %7.2f (%+5.1f%%)  J_s %+5.1f%%  J_e %+5.1f%%  ' ...
            'v_max %+5.1f%%  u_max %+5.1f%%  %.0f s\n'], rows{end, [1:5 8 11:14]});
    end
end
T = cell2table(rows, 'VariableNames', {'Preset', 'Method', 'JsJT', 'MatlabJT', 'DiffJT', ...
    'JsJs', 'MatlabJs', 'DiffJs', 'JsJe', 'MatlabJe', 'DiffJe', 'DiffVmax', 'DiffUmax', 'Seconds'});
end

function d = pct(a, b)
d = 100 * (a - b) / b;
end
