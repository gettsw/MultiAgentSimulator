function mlx = buildPaperResults()
% Rebuilds PaperResults.mlx (repository root) from its plain-text source,
% Experiments/paper/paperResultsSource.m. Edit the source, then run this; the live script is a
% binary file, so the source is what version control can diff.
%   matlab -batch "addpath(genpath(pwd)); buildPaperResults"
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
mlx = fullfile(root, 'PaperResults.mlx');
if isfile(mlx), delete(mlx); end
matlab.internal.liveeditor.openAndSave(fullfile(here, 'paperResultsSource.m'), mlx);
fprintf('Built %s\n', mlx);
end
