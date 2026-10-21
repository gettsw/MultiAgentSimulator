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
   This adds all subfolders to the path and opens the UI. `runMainUI('Monitor', 2)` opens the windows on
   another monitor (a row of `get(groot, 'MonitorPositions')`), sized to fit it.
3. Use the dropdowns in the **Tools** panel to pick a Path Planner, Agent Policy, Agent Type, and Uncertainty mode. These can only be changed while the simulation is **not** running.
4. Use the **Tools** panel to add targets, agents, edges, and walls. **Agent Speed**, **Agent Accel** and **Speed Profile** set the next agents you place (see [Agent types](#agent-types)). Use the **Time Control** panel to play/pause the simulation.

## Target uncertainty and defaults

Each target's uncertainty follows `dR_i/dt = (A_i - B_i N_i) v_i`, where `N_i` is the number of agents at rest at the target (see [Agent types](#agent-types)) and `v_i` keeps `R_i >= 0`. The defaults match the simulator paper:

| Parameter | Default |
|---|---|
| Uncertainty mode | `Static Uniform` (constant `A_i`) |
| Growth rate `A_i` | 1 for every target |
| Decay rate `B_i` (per agent) | 10 |
| Initial uncertainty `R_i(0)` | 0 |

The other modes are opt-in from the **Uncertainty** dropdown:
- `Static Priority Target` — target 1 grows at `A = 5`, the rest at their `A_i`.
- `Dynamic Uncertainty` — as above, plus periodic spikes: every target's rate jumps to `A_spike = 14` for 10 s every 250 s, staggered by 50 s from one target to the next. With `B = 10`, one agent cannot bring a spiking target down (`14 - 10 > 0`) but two can (`14 - 2*10 < 0`).

The experiment harness (`Experiments/runTrial`, `Experiments/trialDefaults.m`) sets its own uncertainty parameters for every trial (by default `Dynamic Uncertainty` with `B = 20` and `A_spike = 28`, the same one-agent/two-agent balance), so its results do not depend on these GUI defaults.

## Layout files

**Save Layout...** / **Load Layout...** (and `ScenarioModel.exportLayout` / `importLayout`) use a `.mat` file holding a struct `s`:

| Field | Meaning |
|---|---|
| `targets` | `n x 2` target positions |
| `edges` | `m x 2` target index pairs |
| `walls` | `k x 4` wall segments `[x1 y1 x2 y2]` |
| `agentStarts` | start target index of each agent |
| `agentSpeed` | maximum speed of each agent (or one value for all) |
| `agentAccel` | maximum acceleration of each agent (or one value for all) |
| `agentProfile` | speed profile of each agent: `"constant"`, `"SO"` or `"FO"` |
| `uncertaintyMode` | one of the modes above |
| `A`, `B`, `R0` | per-target `A_i`, `B_i`, `R_i(0)` (`n x 1`, or one value for all) |

Only `targets` and `edges` are required, so files saved by older versions still load: missing fields keep the defaults above (agents take the GUI's Agent Accel and Speed Profile) and no agents are placed. Loaded agents use the Agent Type selected in the dropdown. `runTrial` ignores stored agents and places its own from the `AgentStarts` parameter.

`Configurations/ReportLayout.mat` is the reference scenario of the simulator paper (Stevens Smart Lab, background `LabLayout.png`): 11 targets, 10 edges, 3 walls, and 5 agents at speed 15 starting on targets **1, 6, 2, 3, 5** (agent 1 on target 1, agent 2 on target 6, and so on), in `Static Uniform` mode with `A = 1`, `B = 10`, `R(0) = 0`.

## Smoke test

Run from the repo root before opening a pull request:
```
matlab -batch "addpath(genpath(pwd)); smokeTest"
```
`tests/smokeTest.m` checks the defaults and the layout file round trip, then runs every registered policy x path planner x agent type for 20 s on `ReportLayout.mat` and checks that each run finishes with a finite objective `J`. It also checks the agent types (older names, every speed profile, service only at rest, feedback agents keeping pace, smooth heading) and that William's Joint RHC harness runs. It prints one line per check and exits with an error if any check fails (about 20 s). New components registered in `ComponentRegistry` are picked up automatically.

`tests/guiTest.m` drives the real GUI with MATLAB's App Testing Framework: it loads `ReportLayout.mat` over the lab background and runs to the end, plays, pauses and resets, saves, clears and reloads a layout, builds a small scenario by clicking, and checks that a GUI run and a headless `runTrial` give the same `J`. It needs a display; the windows open on a secondary monitor if there is one and close when it finishes (about 50 s):
```
matlab -batch "addpath(genpath(pwd)); assertSuccess(runtests('guiTest'))"
```
Screenshots of each stage are written to `results/gui/` (git-ignored).

## Agent types

There are two agent types (`Components/AgentTypes/`): the **Abstract Agent**, an idealized model that executes its plan exactly (the agents of the RHC papers), and the **Physical Agent**, which executes the same plan through feedback control and actuator limits. Both follow the path the planner draws, carry a battery (power model in `Agent.m`, reported as SOC), and record their energy `J_e`, peak speed and peak acceleration.

| | Abstract Agent (`AbstractAgent`) | Physical Agent (`PhysicalAgent`) |
|---|---|---|
| Motion | Follows its speed profile exactly along the path (arc length), at rest on the targets | Double integrator with a PD law tracking a reference that moves along the path with the same profile; speed and acceleration saturate at `maxSpeed` and `maxAccel` |
| Timing | Every transit takes exactly the planned time | Keeps the planned pace, but lags where the plan asks more than its limits allow; arrives within 0.5 of the target once nearly still |
| Model of | The event-driven RHC papers: first-order agent (`"constant"` profile) or energy-optimal second-order agent (`"SO"`, `"FO"`) | A physical robot executing the same plan with feedback |
| `J_e` | Tangential acceleration only (NaN with `"constant"`: impulses) | Full commanded acceleration |

**Speed profiles** (`SpeedProfile.m`, from the agent's `profile` or handed over per transit by a planning policy):
- `"constant"`: constant speed.
- `"SO"`: minimum energy.
- `"FO"`: minimum-energy trapezoid.

Without a planned transit time an agent crosses an edge of length `y` in `y/maxSpeed` (`"constant"`) or `1.5 y/maxSpeed` (`"SO"`, `"FO"`, peak speed `maxSpeed`). On curved paths the profile depends on the path length only; the heading follows the path and turns at `omega = kappa * v` (`state.omega`), as in Theorem 2 of the energy-aware paper.

**Customizing.**
- **In the GUI:** **Agent Type**, **Agent Speed**, **Agent Accel** and **Speed Profile** apply to the next agents placed. They are stored per agent in saved layouts.
- **In `runTrial`:**
  - `AgentType` and `AgentSpeed` set the type and speed.
  - `Agent_<property>` sets any agent, state or controller property on every agent, e.g. `'Agent_profile', "SO"`, `'Agent_maxAccel', 10`, `'Agent_kp', 16`, `'Agent_kd', 8`, `'Agent_stopWhenEmpty', true`.
- **In code:** set `profile`, `state.maxSpeed`, `state.maxAccel`, `controller.kp` / `kd` / `arrivePosTol` on the agent object.

**Service.** A target is serviced only by agents at rest there: arrived (no path) or waiting to leave. Agents in transit service nothing, even when they pass close to a target. `ScenarioModel.detectionRadius` (1.2) is only the tolerance for "at the target".

**Older names.** `"Linear Agent"` (= Abstract, `"constant"`), `"Second-Order Agent"` (= Abstract, `"SO"`), `"Default Agent"` and `"Energy Agent"` (= Physical) still work in scripts and layouts, but are not listed in the GUI.

**Policies and agent types.**
- ✓ = works as designed.
- ≈ = runs, but the agent's transit times differ from what the policy plans.

| Policy | Abstract | Physical | Notes |
|---|---|---|---|
| Random Walk | ✓ | ✓ | No timing assumptions; dwell from the current `R` at arrival |
| Battery Efficient | ✓ | ✓ | Every agent has a battery |
| ED-RHC | ✓ | ≈ | Hands the agent its planned transit `y/maxSpeed`. `"constant"` is the Automatica model; `"SO"` runs it with the SO shape (the JS SO#) |
| ED-ORHC-SO / -FO | ✓ | ≈ (✓ when `speedLimit` and `accelLimit` are within the agent's `maxSpeed` and `maxAccel`) | Hands over the optimal transit time, profile and limits |
| Classical RHC [1], Divergent RHC, Joint RHC, Joint RHC N | ✓ with `"constant"`; ≈ with `"SO"`/`"FO"` (1.5× slower than planned) | ≈ | Plan transits as path length / `maxSpeed`. Joint RHC needs exactly 2 agents |

## Random Walk

`Random Walk` (`Components/AgentPolicy/RandomWalkPolicy.m`) is the baseline policy. It uses the same event-driven structure as ED-RHC, but an agent acts only at two of its own events:
- **On arrival**, it dwells for a uniform random time between 0.8 and 1.2 times the time to empty its target, `R_i / (N_i B_i - A_i)`. Here `N_i` counts the agents already there, which are assumed to stay; they keep their own plans.
- **When the dwell ends**, it moves to a neighbor chosen uniformly at random.

Agents may share targets. During a spike that the agents present cannot overcome, the target's baseline growth rate is used instead. The parameters are `dwellSpread` (default 0.2) and `dwellScale` (default 1; 0 means no dwell).

## Event-driven receding horizon control (ED-RHC)

`ED-RHC` (`Components/AgentPolicy/EDRHCPolicy.m`) implements the distributed event-driven receding horizon controller of S. Welikala and C. G. Cassandras, "Event-driven receding horizon control for distributed persistent monitoring in network systems", *Automatica* 127 (2021) 109519.

- **Local and event-driven.** Each agent decides on its own, using only its target `i` and the 1-hop neighbors of `i`: their `R`, `A`, `B`, the edges `(i, j)`, and whether each neighbor is covered.
  - It solves RHCP1 on arrival (active time `u_i`), RHCP2 when `R_i` reaches 0 (inactive time `v_i`), and RHCP3 when it leaves (next target `j`).
  - It re-solves RHCP1/RHCP2 when a neighbor becomes covered or uncovered.
- **Agents avoid each other.** A target is covered while another agent is at it or on its way to it. Covered neighbors are left out, so two agents never share a target.
- **Variable horizon.** Each RHCP plans `[u_i, v_i]` at `i`, the transit to `j`, and `[u_j, v_j]` at `j` over a horizon `w <= H` (`H = 250` by default). It minimizes the neighborhood cost `(1/w) * sum of the integrals of R`.
- **alpha** (a parameter). RHCP3 weights the next target's cost by `alpha` and the others' by `1 - alpha`.
  - The default `"nominal"` is the paper's `alpha = 1/|N_i|^2` (Sec. IV-A, the RHC-alpha method; `N_i` includes `i`), as in the JS simulator.
  - `alpha = 0.5` gives the paper's unweighted RHC method (Sec. III).
  - Set it with `'Policy_alpha'` in `runTrial`.
- **Solver.** The RHCPs are solved numerically (grid search plus `fmincon` on each smooth case) instead of with the paper's closed forms.
- **Agents.** The method assumes first-order agents moving at constant speed: use the **Abstract Agent** with the `"constant"` profile (the default), whose transit times equal path length / speed. See the compatibility table under [Agent types](#agent-types) for the others.

`Experiments/validation/validateEDRHC` compares `alpha = 0.5` and the nominal alpha with the legacy JavaScript simulator on its presets (`Configurations/Legacy/`).

### Energy-aware ED-RHC (ED-ORHC-SO, ED-ORHC-FO)

`ED-ORHC-SO` and `ED-ORHC-FO` (`EDRHCPolicy` with `transit = "SO"` or `"FO"`) implement the energy-aware variant of S. Welikala and C. G. Cassandras, "Event-driven receding horizon control of energy-aware dynamic agents for distributed persistent monitoring", arXiv 2102.12963 (IEEE CDC 2021). Use them with the **Abstract Agent**; the policy hands each transit's profile, time and limits to the agent, whatever its own profile.

- **Objective.** `J_T = energyWeight * J_e + J_s`. Here `J_e` is the integral of the squared tangential acceleration `u(t)^2` over all agents, and `J_s` is the usual sensing objective.
  - The default `energyWeight = 7.68e-3` is the paper's `213.3e-6` (for a 600 x 600 space) rescaled to the 100 x 100 canvas. Energy grows with length squared, so the factor is 36.
  - `EDRHCPolicy.weightFromBeta(beta, model)` gives the paper's App. A rule, eq. (77): `beta/(1 - beta) * ybar^2 / vmax^4`, where `ybar` is the mean edge length and `vmax` the agents' `maxSpeed`. With `"accel"` as a third argument it gives `beta/(1 - beta) / umax^2`.
  - In `runTrial`, `'EnergyBeta', beta` (optionally with `'EnergyBound', "accel"`) applies this rule.
- **Transit time as a decision.** Events, locality, covering and alpha are as in ED-RHC. In addition, each RHCP chooses the transit time `rho_ij` and adds the energy `E(rho)` of the speed profile on that edge. In RHCP3 the alpha-weighted sensing cost is doubled, so that `alpha = 0.5` gives the paper's objective.
- **Speed and acceleration limits** (parameters `speedLimit` and `accelLimit`, `Inf` by default; `'Policy_speedLimit'` and `'Policy_accelLimit'` in `runTrial`). The agent uses the minimum-energy profile of the method's family for the planned `y` and `rho`, rest to rest:
  - **SO** (any profile). Without limits, `v(t) = 6 y t (rho - t)/rho^3` with `E = 12 y^2/rho^3` (Theorem 3).
    - Under a speed limit, `u` falls linearly to 0 as the speed reaches the limit, then the agent cruises.
    - Under an acceleration limit, `u` is held at the limit, then falls linearly (the paper's SO-A).
    - Under both, it accelerates at the limit, ramps `u` down to 0 as it reaches the speed limit, cruises, and mirrors this at the end.
  - **FO** (trapezoids, the paper's FO-3). Without limits, the trapezoid accelerates for `rho/3`, cruises at `3y/(2 rho)`, brakes for `rho/3`, and has `E = 27 y^2/(2 rho^3)` (Theorem 5). With limits, the cruise speed is clamped to what they allow (the paper's FO-V and FO-A).
  - The RHCP never plans a transit shorter than the limits allow: `y/v + v/u`, or `2 sqrt(y/u)` on short edges.
  - These profiles generalize the paper's Apps. D-E to both limits at once. The speed-limited SO keeps `u` continuous, which uses 2-6% less energy than the paper's SO-V profile.
- **Agents.** The Abstract Agent follows each planned transit exactly and accumulates `J_e` exactly. Under `ED-RHC` it runs each fixed transit `y/maxSpeed` with its own profile (`"SO"`: the JS simulator's SO#). The Physical Agent tracks the same plans with feedback.
- **Metrics.** `runTrial` also returns `out.Je`, `out.JT`, `out.vmax` and `out.umax` (paper eqs. 3 and 73), and `out.log.Je` holds the per-step energy.
- **`clearFirst`** (default `false`, which follows the paper). Set it to `true` for the JS simulator's energy-aware convention: every visit clears the target, there is no RHCP1 on arrival, and plans only include visits that clear `j`.

`Experiments/validation/validateEDORHC` compares both methods (without limits) with the JS simulator (methods 9 and 11) under the JS conventions.

## Headless experiments

`Experiments/runTrial(params)` runs one simulation without the GUI. Unset parameters take the values in `Experiments/trialDefaults.m`; component parameters take any `ComponentRegistry` name:

```matlab
out = runTrial(struct('Layout', "ReportLayout.mat", 'AgentStarts', "layout", ...
    'Policy', "Battery Efficient", 'Planner', "Energy Efficient", 'AgentType', "Physical Agent", ...
    'UncertaintyMode', "Static Uniform", 'A_base', 1, 'B', 10, 'EndTime', 500, 'Seed', 1, 'Log', true));
```

- `AgentStarts`: `"auto"` (spread evenly), `"layout"` (the start targets stored in the layout file), or target indices such as `[1 5]`.
- `Seed`: the `rng` seed set before simulating (`-1`, the default, leaves `rng` as it is).
- `Policy_<property>`: sets a property of the active policy, e.g. `'Policy_dwellScale', 0` turns off the Random Walk dwell and `'Policy_dwellRange', [0 0]` the Battery Efficient dwell.
- `Log = true`: `out.log` holds the per-step time, `J(t)`, agent positions, velocities, accelerations, headings, SOC, battery energy and `J_e`; `out.metrics` holds per-agent path length `L`, heading change `Θ`, integrated squared acceleration `E_a`, final SOC and energy (`Experiments/metrics/`).

`Experiments/runSweep(grid)` runs every combination of parameter values, and `analyzeResults` compares the variants.

## Paper results

`PaperResults.mlx` (repository root) regenerates every result of the simulator paper: Table I from the code, Tables II–IV and Figs. 10–13 from headless runs, and the GUI figures (Figs. 2, 3, 5–9 and 12) from `MainUI`. Open it and click **Run All**, or run it without the Live Editor to get a PDF report of everything:
```
matlab -batch "addpath(genpath(pwd)); exportPaperResults"
matlab -batch "addpath(genpath(pwd)); exportPaperResults(Quick=true)"
```
The full run (500 s runs, 4 seeds) takes about ten minutes and writes to `results/paper/`; `Quick=true` (60 s runs, 2 seeds, about 80 s) writes to `results/paper_quick/`. The output folder holds `PaperResults.pdf`, `runs.csv` (every run), `comparison.csv`, LaTeX tables and `macros.tex` for the paper to `\input`, and `figures/` (PNG; the plots also as PDF).

The experiments are defined in `Experiments/paper/paperDefaults.m` (the reference scenario) and `Experiments/paper/paperConditions.m` (what each experiment varies). The live script is binary, so it is built from a plain-text source: edit `Experiments/paper/paperResultsSource.m` and run `buildPaperResults`.

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
            cmd.dwellSeconds = 0.5;           % optional: wait at the current target before leaving
        end
    end
end
```

Returning `struct()` (no `kind`) leaves the agent where it is; it asks again on the next step. Instead of `dwellSeconds`, a policy may set `agent.dwellRemaining` itself, as the RHC policies do.

Register it:
```matlab
obj.policies('My Policy') = @() MyPolicy();
```

### 3. New Agent Type

Folder: `Components/AgentTypes/`

Subclass `Agent` and implement `update(dt)`:

```matlab
classdef MyAgent < Agent
    methods
        function obj = MyAgent(index, position, maxSpeed)
            obj@Agent(index, position, maxSpeed);
        end

        function update(obj, dt)
            % Move obj.state along obj.path (N x 2 waypoints from the path planner) for dt
            % seconds; set obj.path = [] once the last waypoint is reached.
        end
    end
end
```

`Agent` holds what every agent shares:
- the state (`state`, a `KinematicState`), `path`, `dwellRemaining` and the start target;
- the speed `profile` and the transit plan a policy may hand over (`transitTime`, `transitProfile`, `transitLimits`);
- the battery (`calculate_energy`, `soc`) and the metrics `Je`, `peakSpeed`, `peakAccel`;
- `resetToInitial`, `draw` and `updateVisuals`.

Helpers for path-following agents:
- `beginLeg()` gives the arc-length parametrized path and the speed profile of the transit starting now.
- `pointAt` and `headingAt` give position, tangent, heading and curvature along it.

Override `resetToInitial` (calling `resetToInitial@Agent(obj)` first) to reset extra state, or `label` to change the text drawn on the agent. `AbstractAgent` is the simplest complete example; `PhysicalAgent` adds feedback control with saturation. Agents of different types can be mixed in one scenario.

Register it:
```matlab
obj.agentTypes('My Agent') = @(idx, pos, sp) MyAgent(idx, pos, sp);
```

That's it — restart `runMainUI` and the new component will appear in its dropdown.
