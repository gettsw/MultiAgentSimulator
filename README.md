# MultiAgentSimulator

A MATLAB simulator for experimenting with multi-agent path planning, navigation policies, and agent behavior. Agents move between targets on a user-drawn graph, avoiding walls, while an objective function tracks how well the swarm covers the environment over time.

The project is designed around three **modular areas** so new algorithms can be dropped in and compared side-by-side from the UI:

- **Path Planner** — how a path between two targets is computed (e.g. shortest distance, energy-efficient turning).
- **Agent Policy** — how an agent decides which target to go to next.
- **Agent Type** — the agent class itself (its dynamics, state, energy model, rendering, etc.).

## Examples

### Main UI
The application on launch — Modular Components bar on top, Tools panel on the left, Time Control below, canvas on the right.

![Main UI overview](screenshots/overview.png)

### Building a scenario
Targets, edges, and walls drawn on the canvas before running the simulation.

![Scenario with targets, edges, and walls](screenshots/scenario.png)

### Simulation running
Agents following their paths mid-simulation. The Modular Components dropdowns are locked while the sim is active.

![Simulation in progress](screenshots/running.png)

### Objective plot
The objective function `J(t)` tracked over the course of a run.

![Objective function over time](screenshots/objective.png)

## Running the project

1. Open MATLAB and set the working directory to the repo root (`MultiAgentSimulator`).
2. In the Command Window, run:
   ```matlab
   runMainUI
   ```
   This adds all subfolders to the path and opens the UI.
3. Use the **Modular Components** bar at the top to pick a Path Planner, Agent Policy, and Agent Type. These can only be changed while the simulation is **not** running.
4. Use the **Tools** panel to add targets, agents, edges, and walls. Use the **Time Control** panel to play/pause the simulation.

## Adding a new modular class

Each area has a dedicated folder under `Components/`. Creating a new algorithm takes two steps:

1. Create a new `.m` class file in the matching folder.
2. Register it in `Components/ComponentRegistry.m` — this makes it appear in the UI dropdown automatically.

### 1. New Path Planner

Folder: `Components/PathPlanner/`

Create a class that inherits from `PathPlanner` and implements `plan`:

```matlab
classdef MyPlanner < PathPlanner
    methods
        function path = plan(obj, pStart, pEnd, walls, bounds)
            % Return an Nx2 matrix of [x y] waypoints from pStart to pEnd.
            path = [pStart; pEnd];
        end
    end
end
```

Register it in `ComponentRegistry.m`:
```matlab
obj.pathPlanners('My Planner') = @() MyPlanner();
```

### 2. New Agent Policy

Folder: `Components/AgentPolicy/`

Create a class with a `plan` method that returns a move command struct:

```matlab
classdef MyPolicy < handle
    methods
        function cmd = plan(obj, agent, model, adj, curIdx, simTime)
            cmd = struct();
            nbrs = find(adj(curIdx, :));
            if isempty(nbrs), return; end
            cmd.kind = "move";
            cmd.targetIdx = nbrs(1);          % index of the target to move to
            cmd.dwellSeconds = 0.5;           % optional dwell after arrival
        end
    end
end
```

Register it:
```matlab
obj.policies('My Policy') = @() MyPolicy();
```

### 3. New Agent Type

Folder: `Components/AgentTypes/`

Copy `DefaultAgent.m` as a starting template. An agent class must expose:
- Constructor `MyAgent(index, position, maxSpeed)`
- `state` property (a `KinematicState`)
- `path`, `pathIndex`, `current_target_idx`, `dwellRemaining`, `initialPosition`, `initialTargetIdx` properties
- Methods: `resetToInitial()`, `update(dt)`, `draw(ax)`, `updateVisuals()`

Register it:
```matlab
obj.agentTypes('My Agent') = @(idx, pos, sp) MyAgent(idx, pos, sp);
```

That's it — restart `runMainUI` and the new component will appear in its dropdown.
