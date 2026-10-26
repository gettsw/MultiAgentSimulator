function f = paperPlots(kind, R, file)
% Result plots of the simulator paper, sized for one IEEE column and saved as file.png/.pdf:
%   paperPlots("objectives", R, file)  Fig. 9: J(t) of every policy, with Abstract Agents (a) and
%                                      Physical Agents (b); Random Walk as the mean over its
%                                      seeds with the range shaded
%   paperPlots("tradeoff", R, file)    Fig. 11: J(T) against J_e for ED-ORHC-SO as the energy
%                                      weight grows (beta values marked)
%   paperPlots("soc", R, file)         Fig. 12: SOC(t) of every agent under Battery-Aware ED-RHC
%                                      (a) and ED-RHC (b), Physical Agents on the charging-dock
%                                      scenario
% R is the output of paperRuns. The figure opens on the secondary monitor when there is one.
arguments
    kind (1,1) string
    R (1,1) struct
    file (1,1) string
end
switch kind
    case "objectives"
        f = paperFigure(3.5, 4.2);
        t = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        for g = ["Abstract", "Physical"]
            ax = nexttile(t); hold(ax, 'on');
            idx = find(endsWith(R.Labels, "/ " + g));
            for c = idx
                name = extractBefore(R.Labels(c), " /");
                curves = R.Curves(c, ~cellfun(@isempty, R.Curves(c, :)));
                tt = curves{1}.t;
                J = cell2mat(cellfun(@(cv) cv.J(:), curves, 'UniformOutput', false));
                if size(J, 2) > 1
                    fill(ax, [tt; flipud(tt)], [min(J, [], 2); flipud(max(J, [], 2))], [0.7 0.7 0.7], ...
                        'EdgeColor', 'none', 'FaceAlpha', 0.4, 'HandleVisibility', 'off');
                end
                plot(ax, tt, mean(J, 2), 'LineWidth', 1.2, 'DisplayName', char(name));
            end
            ylabel(ax, 'Objective: $J(t)$', 'Interpreter', 'latex');
            title(ax, sprintf('(%s) %s Agents', char('a' + (g == "Physical")), g), 'FontWeight', 'normal', ...
                'Interpreter', 'latex', 'FontSize', 9);
            style(ax);
            set(ax, 'YScale', 'log');
            xlim(ax, [0 tt(end)]);
            if g == "Abstract", legend(ax, 'Location', 'northeast', 'Interpreter', 'latex', 'Box', 'on', 'FontSize', 7); end
        end
        xlabel(ax, 'Time: $t$ [s]', 'Interpreter', 'latex');
    case "tradeoff"
        f = paperFigure(3.5, 2.5);
        ax = axes(f); hold(ax, 'on');
        T = paperTables("tradeoff", R);
        plot(ax, T.Je, T.J, '-o', 'LineWidth', 1.2, 'MarkerSize', 4, 'MarkerFaceColor', 'w');
        set(ax, 'XScale', 'log');
        xlabel(ax, 'Energy objective: $J_e$', 'Interpreter', 'latex');
        ylabel(ax, 'Objective: $J(T)$', 'Interpreter', 'latex');
        style(ax);
        lx = log10([min(T.Je) max(T.Je)]); ly = [min(T.J) max(T.J)];
        xlim(ax, 10 .^ (lx + 0.12 * diff(lx) * [-1 1]));   % room for the labels of the end points
        ylim(ax, ly + 0.12 * diff(ly) * [-1 1]);
        labelPoints(ax, T.Je, T.J, compose("%g", T.Beta));
    case "soc"
        f = paperFigure(3.5, 4.0);
        t = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        picks = ["Battery-Aware ED-RHC / Physical", "ED-RHC / Physical"];
        for k = 1:2
            ax = nexttile(t); hold(ax, 'on');
            c = find(R.Labels == picks(k), 1);
            cv = R.Curves{c, 1};
            for a = 1:size(cv.SOCs, 2)
                plot(ax, cv.t, cv.SOCs(:, a), 'LineWidth', 1.1, 'DisplayName', sprintf('Agent %d', a));
            end
            ylabel(ax, 'SOC$(t)$ [\%]', 'Interpreter', 'latex');
            title(ax, sprintf('(%s) %s', char('a' + k - 1), extractBefore(picks(k), " /")), ...
                'FontWeight', 'normal', 'Interpreter', 'latex', 'FontSize', 9);
            style(ax);
            ylim(ax, [0 100]); xlim(ax, [0 cv.t(end)]);
            if k == 1, legend(ax, 'Location', 'southeast', 'Interpreter', 'latex', 'Box', 'on', 'FontSize', 7); end
        end
        xlabel(ax, 'Time: $t$ [s]', 'Interpreter', 'latex');
    otherwise
        error('paperPlots:UnknownKind', 'Unknown plot "%s"', kind);
end
saveFigure(f, file);
end

function labelPoints(ax, x, y, labels)
% Labels each point of the curve (x, y) (log x axis) on the first side, of NE, NW, SE and SW,
% where its label stays inside the axes and covers neither the curve nor an earlier label
drawnow;
xl = log10(ax.XLim); yl = ax.YLim;
px = (log10(x(:)) - xl(1)) / diff(xl); py = (y(:) - yl(1)) / diff(yl);   % normalized
s = linspace(0, 1, 15);
cx = px(1:end-1) + (px(2:end) - px(1:end-1)) .* s; cy = py(1:end-1) + (py(2:end) - py(1:end-1)) .* s;
curve = [cx(:), cy(:)];
boxes = zeros(0, 4);
sides = [1 1; -1 1; 1 -1; -1 -1];
halign = ["right", "", "left"]; valign = ["top", "", "bottom"];
for k = 1:numel(px)
    for c = 1:size(sides, 1)
        sx = sides(c, 1); sy = sides(c, 2);
        h = text(ax, px(k) + 0.012 * sx, py(k) + 0.02 * sy, labels(k), 'Units', 'normalized', ...
            'FontSize', 7, 'Interpreter', 'latex', 'HorizontalAlignment', halign(sx + 2), ...
            'VerticalAlignment', valign(sy + 2));
        e = h.Extent;
        inside = e(1) >= 0 && e(2) >= 0 && e(1) + e(3) <= 1 && e(2) + e(4) <= 1;
        hitsCurve = any(curve(:, 1) > e(1) & curve(:, 1) < e(1) + e(3) & ...
                        curve(:, 2) > e(2) & curve(:, 2) < e(2) + e(4));
        hitsLabel = any(boxes(:, 1) < e(1) + e(3) & e(1) < boxes(:, 1) + boxes(:, 3) & ...
                        boxes(:, 2) < e(2) + e(4) & e(2) < boxes(:, 2) + boxes(:, 4));
        if (inside && ~hitsCurve && ~hitsLabel) || c == size(sides, 1), break; end
        delete(h);
    end
    boxes(end+1, :) = e; %#ok<AGROW>
end
end

function style(ax)
set(ax, 'FontSize', 8, 'TickLabelInterpreter', 'latex', 'Box', 'on');
grid(ax, 'on');
ax.GridAlpha = 0.15;
end

function saveFigure(f, file)
[d, ~] = fileparts(file);
if d ~= "" && ~isfolder(d), mkdir(d); end
exportgraphics(f, file + ".png", 'Resolution', 300);
exportgraphics(f, file + ".pdf", 'ContentType', 'vector');
end
