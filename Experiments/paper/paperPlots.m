function f = paperPlots(kind, R, file, opts)
% Result plots of the simulator paper, sized for one IEEE column and saved as file.png/.pdf:
%   paperPlots("objective", R, file, Condition=label)  Figs. 10-11: J(t) of every run of one
%                                                      condition (one line per seed)
%   paperPlots("soc", R, file)                         Fig. 13: mean SOC(t) of the agents per
%                                                      condition, averaged over the runs
% R is the output of paperRuns. The figure opens on the secondary monitor when there is one.
arguments
    kind (1,1) string
    R (1,1) struct
    file (1,1) string
    opts.Condition (1,1) string = ""
end
f = paperFigure(3.5, 2.4);
ax = axes(f);
hold(ax, 'on');
switch kind
    case "objective"
        c = R.Labels == opts.Condition;
        assert(nnz(c) == 1, 'paperPlots:UnknownCondition', 'No condition "%s" in R', opts.Condition);
        for s = 1:numel(R.Seeds)
            cv = R.Curves{c, s};
            plot(ax, cv.t, cv.J, 'LineWidth', 1.2, 'DisplayName', sprintf('Run %d', s));
        end
        ylabel(ax, 'Time-averaged objective: $J(t)$', 'Interpreter', 'latex');
        legend(ax, 'Location', 'northwest', 'Interpreter', 'latex', 'Box', 'on');
    case "soc"
        for c = 1:numel(R.Labels)
            soc = mean(cell2mat(cellfun(@(cv) cv.SOC, R.Curves(c, :), 'UniformOutput', false)), 2);
            plot(ax, R.Curves{c, 1}.t, soc, 'LineWidth', 1.2, 'DisplayName', char(R.Labels(c)));
        end
        ylabel(ax, 'State of charge: SOC$(t)$ [\%]', 'Interpreter', 'latex');
        legend(ax, 'Location', 'northeast', 'Interpreter', 'latex', 'Box', 'on');
    otherwise
        error('paperPlots:UnknownKind', 'Unknown plot "%s"', kind);
end
xlabel(ax, 'Time: $t$ [s]', 'Interpreter', 'latex');
set(ax, 'FontSize', 9, 'TickLabelInterpreter', 'latex', 'Box', 'on');
grid(ax, 'on');
ax.GridAlpha = 0.15;
xlim(ax, [0 R.Curves{1, 1}.t(end)]);
saveFigure(f, file);
end

function saveFigure(f, file)
[d, ~] = fileparts(file);
if d ~= "" && ~isfolder(d), mkdir(d); end
exportgraphics(f, file + ".png", 'Resolution', 300);
exportgraphics(f, file + ".pdf", 'ContentType', 'vector');
end
