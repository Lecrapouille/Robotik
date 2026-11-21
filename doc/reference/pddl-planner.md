# The PDDL planner: reference

[⏫ Reference](README.md)

## `controller.planner`

| Key | Default | Meaning |
|-----|---------|---------|
| `domain` | required | The PDDL domain, relative to the package root |
| `method` | `search` | `search`: a planner finds the plan; `pabt`: a reactive tree grows from the goal ([Without search: a tree grown from the goal (PA-BT)](../concepts/pddl-planner.md#without-search-a-tree-grown-from-the-goal-pa-bt)) |
| `facts` | none | True atoms that never change and are not observed |
| `goal` | required (or `goal_pddl`) | Literals to reach: `(in a b)`, `(not (holding a))`, `hand_empty` |
| `goal_pddl` | none | The goal in PDDL, for what a list cannot say (`(forall ...)`). Not checked on the beliefs. Not with `pabt` |
| `replan.max` | 3 | New plans allowed after the first one (`pabt`: new tries after a failed action) |
| `replan.delay` | 0.5 | Simulated seconds waited before a new plan |
| `verify` | true | Check the effects of each action on the beliefs. `observed`: an effect counts only when a belief was seen again after the start of the step (a failed grasp is not taken for a held object); `false`: no check |
| `parallel` | true | Run together the steps that do not depend on each other ([Steps that run together](../concepts/pddl-planner.md#steps-that-run-together)); `false` runs the plan step by step |
| `search` | `lama` | Kairos search: `ff`, `lama`, `eager-lama`, `novelty`, `lama-novelty`, `ehc`, `goals` |
| `timeout` | 10 | Wall-clock limit of one search, in seconds |

A scenario has `behavior_tree` **or** `planner`, not both.

Beliefs older than `perception.max_age` (seconds, infinite by default) are unknown to the planner: an observer does not plan on what the robot saw long ago.

## Metrics

| Name | Value |
|------|-------|
| `planner.plans` | Plans made in the run |
| `planner.replans` | Plans made after a problem |
| `planner.steps` | Actions of the current plan |
| `planner.done` | Actions done and checked |

Use them in assertions: `(== planner.replans 0)` says "the first plan was the
right one".

## C++

| Class (header) | Role |
|----------------|------|
| `PlanningDomain` (`Robotik/Planning/PlanningDomain.hpp`) | `observe(predicate, observer)`, `bind(action, nodes, parameters)` |
| `PlanExecutor` (`Robotik/Planning/PlanExecutor.hpp`) | The planner controller: `plan()`, `done()` (steps finished), `running(step)`, `after()` and `order()` (what each step waits for), `history()`, `observe(context)`, `problem(context)`, `replan(cause)` |
| `orderSteps`, `StepFootprint`, `StepOrder` (`Robotik/Planning/PlanGraph.hpp`) | The partial order of a plan: start and end events, what each one waits for |
| `PlanningEngine` (`Robotik/Planning/PlanningEngine.hpp`) | The planner interface; `makeKairosPlanner()`. No Kairos type leaves `src/Robotik/Planning/Kairos/` |
| `Literal`, `Atom`, `DomainSchema` (`Robotik/Planning/Pddl.hpp`) | Symbols, and what RobotIK reads of a domain |
| `BackchainTree`, `achievers`, `groundPreconditions`, `ancestors` (`Robotik/Task/Planning/Backchaining.hpp`) | The tree of PA-BT as data: conditions, the actions that make them true; `PlanExecutor::backchain()` gives the one of the run |

`Simulation::planning()` gives the `PlanningDomain` to fill;
`Simulation::planner()` the executor, or null without a planner.

## Debugging

- **Simulator**: the **Planner** panel shares the middle of Mission with
  Execution graph. It opens by default for a planned scenario. The goal,
  state, progress and **Replan** stay on top. **Beliefs** and **History**
  are separate tabs; click the Value column header to show true atoms only.
  An execution preview stays below both tabs, with a link to the full
  Execution graph. **Open PDDL files** in History opens the selected plan's
  texts. **Replan** drops the plan. The same symbolic beliefs are in the
  **World model** panel of the Autonomy workspace.
- **Texts**: **Plugin → Files → Planner** shows the **Domain**, the
  generated **Problem**, the planner **Output** and the **Generated tree** of
  one plan (the last one, or the one picked in History). **Save PDDL**
  writes all the texts in `planner/<scenario id>/` of the current folder.
- **Headless**: `--trace` prints one line per plan, as in [the tutorial](../tutorials/06-a-planned-pick-and-place.md). The summary
  line of each run gives the final state of the planner, the number of plans
  and of replans. `--report r.json` adds a `planner` object to each run:

  ```json
  "planner": {
    "state": "succeeded", "replans": 2,
    "failure": "1. (detect red_cube): skill Detect(red_cube) FAILED (unavailable: wrist_camera)",
    "plans": [
      { "time": 0, "cause": "start", "result": "solved", "expanded": 3,
        "milliseconds": 0.77, "steps": ["(detect red_cube)", "(pick red_cube)", "(place red_cube box)"] }
    ]
  }
  ```

  `milliseconds` is wall-clock time of the search: it changes from one
  machine to another. Everything else is the same for the same seed.
