% Run this file
function runMainUI(varargin)
% runMainUI or runMainUI('Monitor', k) to open the windows on monitor k
root = fileparts(mfilename('fullpath'));
addpath(genpath(root));
ui = MainUI(varargin{:}); %#ok<NASGU>
end
