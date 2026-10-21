function f = showImages(files)
% Shows image files in one figure, up to three per row, titled by file name (for the live script)
files = string(files);
cols = min(3, numel(files));
f = paperFigure(3.2 * cols, 3.2 * ceil(numel(files) / cols));
t = tiledlayout(f, ceil(numel(files) / cols), cols, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:numel(files)
    ax = nexttile(t);
    image(ax, imread(files(k)));
    axis(ax, 'image', 'off');
    [~, name] = fileparts(files(k));
    title(ax, name, 'Interpreter', 'none', 'FontWeight', 'normal', 'FontSize', 8);
end
end
