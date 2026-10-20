function f = paperFigure(width, height)
% White figure of the given size in inches, placed on the secondary monitor when there is one
mons = get(groot, 'MonitorPositions');
m = mons(MainUI.secondaryMonitor(), :);
f = figure('Color', 'w', 'Units', 'pixels', 'Position', [m(1) + 40, m(2) + m(4) - 500, 500, 350]);
f.Units = 'inches';
f.Position(3:4) = [width height];
end
