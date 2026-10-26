classdef (Abstract) Policy < handle
    % Base class of the movement policies. The model calls
    %   cmd = plan(agent, model, adj, i, simTime)
    % whenever an agent at target i has no path and no dwell left (adj: the target adjacency
    % matrix). plan returns a move command,
    %   cmd.kind = "move", cmd.targetIdx = j (a neighbor of i),
    %   cmd.dwellSeconds = d (optional: wait d seconds at i before leaving),
    % or struct() to stay at i (the model asks again at the next step). A policy that plans
    % transits may also hand the agent its plan (agent.transitTime, transitProfile,
    % transitLimits) or set agent.dwellRemaining itself, as the RHC policies do.
    % Optional:
    %   onStep(agent, model, simTime)   called for every agent at every step, before plan (to
    %                                   react to events while the agent dwells)
    %   resetReservations()             called when the simulation is reset (clear the
    %                                   policy's memory of earlier runs)
    %   agentAssumption (Constant)      what the policy assumes of the agents' motion: "any",
    %                                   "plans transit" or "FO-0" (constant speed)
    % Register a new policy in ComponentRegistry (policies) to list it in the GUI and runTrial.
    % Examples/EventDetection/SimplePolicy.m is a small example.
    methods (Abstract)
        cmd = plan(obj, agent, model, adj, curIdx, simTime)
    end

    methods
        function resetReservations(~)
            % Nothing to clear by default
        end
    end
end
