% Run this file
function runMainUI()
root = fileparts(mfilename('fullpath'));
addpath(genpath(root));
ui = MainUI(); %#ok<NASGU>
end
