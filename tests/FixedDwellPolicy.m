classdef FixedDwellPolicy < Policy
    % Test helper (smokeTest): waits dwell seconds at each target through cmd.dwellSeconds, then
    % moves to its lowest-numbered neighbor. It exercises the model's handling of a dwell
    % requested in a move command, which no registered policy uses.
    properties
        dwell = 2   % seconds
    end

    methods
        function cmd = plan(obj, ~, ~, adj, i, ~)
            cmd = struct();
            j = find(adj(i, :), 1);
            if isempty(j), return; end
            cmd.kind = "move";
            cmd.targetIdx = j;
            cmd.dwellSeconds = obj.dwell;
        end
    end
end
