# Event detection: a new monitoring problem from scratch

Security robots patrol a small museum. Incidents (events) occur at random at the targets and expire after a while; a robot at a target detects the incidents in progress there. The goal is to miss as few as possible.

Four classes define the problem, one per extension point of the simulator. Their names are generic on purpose: copy them as the starting point of your own problem. The header of each file explains the class, when the simulator calls it, how it is registered and selected, and what to change to make it your own.

| File | Base class | Code lines | What it does |
|---|---|---|---|
| `SimpleTarget.m` | `Target` | 32 | Events arrive at rate `eventRate` (a Poisson process) and last `lifetime` s; an agent at the target detects those in progress. `R` = events in progress |
| `SimpleAgent.m` | `Agent` | 32 | Moves at its top speed from path point to path point |
| `SimplePolicy.m` | `Policy` | 19 | Goes to the free neighbor with the most events in progress, else to the one visited longest ago |
| `SimpleMetric.m` | `Metric` | 15 | Share of the events that ended before an agent detected them |

`Components/ComponentRegistry.m` registers each with one line (as "Simple Target (example)", "Simple Agent (example)", "Simple Policy (example)" and "Simple Metric (example)"), so they appear in the GUI (Target Type, Agent Type, Active Policy and Objective dropdowns) and in `runTrial` (`TargetType`, `AgentType`, `Policy`, `Metrics`).

## Walkthrough

Open `eventDetectionExample.m` and run it section by section (Ctrl+Enter) while watching the GUI. Each section does what the named clicks do:

1. Open the simulator and import the floor plan (Import Map...).
2. Draw the walls (Add Wall, two clicks per wall).
3. Mark the targets (Add Target, one click each).
4. Connect them (Add Edge, two clicks each); the path planner routes every edge around the walls.
5. Choose the new classes in the dropdowns and place three robots (Add Agent).
6. Run ten minutes (End time, Run to End); the objective window plots the share of events missed.
7. Save the layout and compare policies without the GUI (`runTrial`, ten seeds each).

Each step saves the canvas to `results/example/`. The finished scenario is also in the Scenario dropdown as "Museum (event detection example)". Typical result: SimplePolicy misses about 13% of the events and Random Walk about 44%.

## Making it your own

- Change the problem: edit `updateUncertainty` in `SimpleTarget` (for example, events that grow more urgent with age).
- Change the robots: give `SimpleAgent` a slower turn, a battery limit, or replace it with `PhysicalAgent`.
- Change the decisions: `SimplePolicy.plan` sees the whole model (targets, edges, other agents).
- Change the score: add a `Metric` (for example, the mean time from an event's start to its detection).
