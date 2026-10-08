classdef ReservationTable < handle
    % Space-time node/edge reservations shared by RHC-style policies.
    % Each policy owns one table, so reservations never leak between policies.
    properties
        nodeRes = struct('nodeIdx', {}, 'tStart', {}, 'tEnd', {}, 'agentID', {});
        edgeRes = struct('u', {}, 'v', {}, 'tStart', {}, 'tEnd', {}, 'agentID', {});
    end

    methods
        function result = manage(obj, action, data, tStart, tEnd, agentID)
            result = true;

            switch action
                case "resetAll"
                    obj.nodeRes = obj.nodeRes([]);
                    obj.edgeRes = obj.edgeRes([]);

                case "clearStale"
                    currTime = data;
                    if ~isempty(obj.nodeRes), obj.nodeRes = obj.nodeRes([obj.nodeRes.tEnd] > currTime); end
                    if ~isempty(obj.edgeRes), obj.edgeRes = obj.edgeRes([obj.edgeRes.tEnd] > currTime); end

                case "clearAgent"
                    id = data;
                    if ~isempty(obj.nodeRes), obj.nodeRes = obj.nodeRes([obj.nodeRes.agentID] ~= id); end
                    if ~isempty(obj.edgeRes), obj.edgeRes = obj.edgeRes([obj.edgeRes.agentID] ~= id); end

                case "isNodeFree"
                    nodeIdx = data;
                    for i = 1:length(obj.nodeRes)
                        res = obj.nodeRes(i);
                        if res.nodeIdx == nodeIdx && res.agentID ~= agentID
                            if max(tStart, res.tStart) < min(tEnd, res.tEnd)
                                result = false;
                                return;
                            end
                        end
                    end

                case "isEdgeFree"
                    u = data(1); v = data(2);
                    for i = 1:length(obj.edgeRes)
                        res = obj.edgeRes(i);
                        if ((res.u == u && res.v == v) || (res.u == v && res.v == u)) && res.agentID ~= agentID
                            if max(tStart, res.tStart) < min(tEnd, res.tEnd)
                                result = false;
                                return;
                            end
                        end
                    end

                case "hasAny"
                    result = any([obj.nodeRes.agentID] == agentID);

                case "reserveNode"
                    obj.nodeRes(end+1) = struct('nodeIdx', data, 'tStart', tStart, 'tEnd', tEnd, 'agentID', agentID);

                case "reserveEdge"
                    obj.edgeRes(end+1) = struct('u', data(1), 'v', data(2), 'tStart', tStart, 'tEnd', tEnd, 'agentID', agentID);
            end
        end

        function [occStart, occEnd] = occupancyWindow(~, agent, edge, simTime, dwellHere, H_upper)
            % Time interval the agent occupies the destination node. The model dwells at the
            % current node BEFORE leaving, then the agent travels at its real speed and dwells at
            % the destination for its next (not yet chosen, <= H_upper) dwell. Margin covers
            % detection radius / step lag. Re-reserved exactly when the agent replans on arrival.
            margin = 0.5;
            pts = edge.curvePoints;
            travel = sum(vecnorm(diff(pts, 1, 1), 2, 2)) / max(agent.state.maxSpeed, eps);
            occStart = simTime + dwellHere + travel - margin;
            occEnd   = simTime + dwellHere + travel + H_upper + margin;
        end

        function tf = isOccupiedByUnplanned(obj, model, nodeIdx, agent, idFcn)
            % Agents that have not planned yet (t = 0) sit on a node with no reservation
            residing = model.targets(nodeIdx).residingAgents;
            if iscell(residing), residing = [residing{:}]; end
            tf = false;
            for r = residing
                if r ~= agent && ~obj.manage("hasAny", [], [], [], idFcn(r))
                    tf = true; return;
                end
            end
        end
    end
end
