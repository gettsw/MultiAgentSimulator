function mlx = buildCollabResults()
% Rebuilds CollabResults.mlx (repository root) from its plain-text source,
% Experiments/collab/collabResultsSource.m. Edit the source, then run this; the live script is a
% binary file, so the source is what version control can diff.
%   matlab -batch "addpath(genpath(pwd)); buildCollabResults"
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
mlx = fullfile(root, 'CollabResults.mlx');
if isfile(mlx), delete(mlx); end
matlab.internal.liveeditor.openAndSave(fullfile(here, 'collabResultsSource.m'), mlx);
fprintf('Built %s\n', mlx);
end
