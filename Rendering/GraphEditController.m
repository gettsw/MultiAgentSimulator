classdef GraphEditController < handle
    properties
        model
        renderer
        mode = "idle"
        edgePick = []      
        wallPoints = []    
        clickTol = 3.0
        statusCallback 
        gridStep = 5
        xMin = 0
        xMax = 100
        yMin = 0
        yMax = 100
    end

    methods
        function obj = GraphEditController(model, renderer, statusCallback)
            obj.model = model;
            obj.renderer = renderer;
            obj.statusCallback = statusCallback;
        end

        function setMode(obj, m)
            obj.mode = string(m);
            obj.edgePick = [];
            obj.wallPoints = []; 
            obj.renderer.resetEdgePreview();
            obj.say("Mode: " + obj.mode);
        end

        function importBackground(obj)
            obj.renderer.loadBackgroundImage();
            obj.say("Background image loaded.");
        end

        function clearAll(obj)
            obj.renderer.clearAxes();  
            obj.model.clearAll();
            obj.renderer.renderAll(obj.model);
            obj.setMode("idle");
        end

        function onCanvasClick(obj, pos, agentSpeed)
            snappedPos = obj.snapToGrid(pos, obj.gridStep);
            snappedPos = obj.clampToBounds(snappedPos);
            switch obj.mode
                case "addTarget"
                    obj.model.addTarget(snappedPos);
                    obj.renderer.renderAll(obj.model);
                case "addAgent"
                    % selectedType = "Energy"; 
                    selectedType = "Default"; 
                    [~, ok, msg] = obj.model.addAgentOnTarget(pos, agentSpeed, obj.clickTol, selectedType);
                    if ~ok && msg ~= ""
                        obj.say(msg);
                        return;
                    end
                    obj.renderer.renderAll(obj.model);
                case "addEdge"
                    obj.handleEdgePick(pos);
                case "addWall"
                    if isempty(obj.wallPoints)
                        obj.wallPoints = snappedPos;
                        obj.say("Start point set.");
                        obj.renderer.updateEdgePreview(obj.wallPoints, snappedPos);
                    else
                        obj.model.addWall(obj.wallPoints, snappedPos);
                        obj.wallPoints = [];
                        obj.renderer.resetEdgePreview();
                        obj.model.replanAllEdges();
                        obj.renderer.renderAll(obj.model);
                        obj.say("Wall added and paths re-planned.");
                    end
            end
        end

        function onCanvasMove(obj, pos)
            if obj.mode == "addEdge" && ~isempty(obj.edgePick)
                p1 = obj.model.targets(obj.edgePick(1)).position;
                obj.renderer.updateEdgePreview(p1, pos);
            elseif obj.mode == "addWall" && ~isempty(obj.wallPoints)
                snappedPos = obj.snapToGrid(pos, obj.gridStep);
                snappedPos = obj.clampToBounds(snappedPos);
                obj.renderer.updateEdgePreview(obj.wallPoints, snappedPos);
            end
        end
    end

    methods (Access=private)
        function handleEdgePick(obj, pos)
            if numel(obj.model.targets) < 2
                obj.say("Add at least 2 targets.");
                return;
            end
            [tIdx, dist] = obj.model.findNearestTarget(pos);
            if isempty(tIdx) || dist > obj.clickTol
                return;
            end
            if any(obj.edgePick == tIdx)
                return;
            end
            obj.edgePick(end+1) = tIdx;
            if isscalar(obj.edgePick)
                obj.say(sprintf("Picked T%d. Pick second...", tIdx));
                p1 = obj.model.targets(tIdx).position;
                obj.renderer.updateEdgePreview(p1, pos);
                return;
            end
            i = obj.edgePick(1);
            j = obj.edgePick(2);
            [~, ok, msg] = obj.model.addEdgeByTargets(i, j);
            if ok
                obj.say(sprintf("Shortest path planned T%d to T%d", i, j));
            else
                obj.say(msg);
            end
            obj.edgePick = [];
            obj.renderer.resetEdgePreview();
            obj.renderer.renderAll(obj.model);
        end

        function say(obj, msg)
            if ~isempty(obj.statusCallback)
                obj.statusCallback(char(msg));
            end
        end

        function p = snapToGrid(~, p, step)
            p = step * round(p ./ step);
        end

        function p = clampToBounds(obj, p)
            p(1) = min(max(p(1), obj.xMin), obj.xMax);
            p(2) = min(max(p(2), obj.yMin), obj.yMax);
        end
    end
end