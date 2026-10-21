function p = paperDefaults()
% runTrial parameters of the simulator paper's reference scenario (Sec. IV): ReportLayout.mat
% (Stevens Smart Lab, 11 targets, 10 edges, 3 walls) with 5 agents on its stored start targets,
% constant growth A = 1 on every target, decay B = 10 per agent, R(0) = 0, max speed 15,
% T = 500 s, dt = 0.05 s. Each experiment overrides Policy, Planner and AgentType.
p = trialDefaults();
p.Layout = "ReportLayout.mat";
p.AgentStarts = "layout";
p.Policy = "Battery Efficient";
p.Planner = "Dijkstra Corners";
p.AgentType = "Physical Agent";
p.UncertaintyMode = "Static Uniform";
p.A_base = 1;
p.B = 10;
p.AgentSpeed = 15;
p.EndTime = 500;
p.Dt = 0.05;
p.Log = true;
end
