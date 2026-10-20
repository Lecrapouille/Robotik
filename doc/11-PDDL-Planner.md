# 11. The PDDL planner

**What you will learn:** how a scenario can give a **goal** instead of a behavior tree, how the planner finds the steps, how it reacts when reality does not follow the plan, and what a plugin writes to use it.

You should know [skills and behavior trees](05-Skills-and-Behavior-Trees.md) and [worlds and scenarios](03-Worlds-and-Scenarios.md) first.

---

## 1. Why a planner

A behavior tree says **how**: "detect, approach, reach, grasp, carry, release". You write every step, in order. That fits when the sequence is almost always the same.

The planner works from **what**: the scenario gives a **goal** instead of a tree: "the cube must be in the box". You still write **skills** (motions, sensors) and a **domain** (symbolic actions and their rules written in PDDL syntax). The solver chooses the order of actions from what is true when it **plans**. You do not hand-write the full sequence.

That matters when the order depends on the situation. If the robot already holds the cube, there is nothing to pick. If the camera already saw the box, there is nothing to detect. If the brush is dirty, it must be rinsed first. A tree needs a branch for each case. A planner skips steps that are already satisfied and **replans** when a step fails or the world changes.

The planner reads the robot's **belief** (what perception says), not the **ground truth** (where things really are in the simulation). Skills work the same way. Assertions at the end still use ground truth to say whether the mission really succeeded. [03 §7](03-Worlds-and-Scenarios.md#7-truth-description-and-belief) defines belief, ground truth, description, and oracle.

The planner returns a **plan**: symbolic steps like `(pick red_cube)`, not joint trajectories. **Skills** run on the robot (`Approach`, `Grasp`, …). The plugin connects each PDDL action to skills with a **binding** (§3). The executor turns the plan into a behavior tree for the run. If a skill fails, the rest of the plan is dropped and Kairos plans again (§4).

The words of this chapter:

| Word | Meaning |
|------|---------|
| **PDDL** | *Planning Domain Definition Language*: a file syntax for describing domains and problems. You need a planner (solver) to read these files and searches for a plan. |
| **Domain** | The types of objects, the predicates and the actions of a task. Written once per plugin, in a `.pddl` file. |
| **Problem** | The objects, what is true now, and the goal. Also written in a `.pddl` file but RobotIK writes it for you. |
| **Kairos** | The PDDL solver RobotIK uses: it takes a problem and returns a plan. |
| **Predicate** | A symbolic fact (true or false). You declare names in the domain. Kairos reads their value from **observers** (belief) and from `planner.facts` (§3). Examples: `holding(red_cube)`, `in(red_cube, box)`, `hand_empty`. |
| **Action** | A symbolic step in the domain (not a skill): preconditions and effects in PDDL. Example: `pick` means "the robot should end up holding the object", not how to move the arm. |
| **Plan** | The ordered list of actions Kairos returns, e.g. `(detect red_cube) (pick red_cube) (place red_cube box)`. It is a recipe on paper: it does not run the robot. |
| **Skill** | C++ code that moves the robot or reads sensors for one step, through `RobotContext` ([05](05-Skills-and-Behavior-Trees.md)). Examples: `Detect`, `Approach`, `Grasp`. |
| **Mission** | The plugin C++ class for a scenario (`PickPlaceMission`, `PainterMission`, …): it registers skills, publishes metrics, and ends the run. With a planner, it also registers **observers** and **bindings** on `PlanningDomain` ([04](04-Your-First-Plugin.md)). |
| **Ground truth** | The real state of the simulated world (poses, contacts, what is inside what): what `Simulation::pose` and `Simulation::query` report. Assertions, metrics, and RL read it; the planner and skills do not. |
| **Belief** | What the robot thinks is true: the `WorldModel`, updated by perception. Observers turn belief into PDDL predicates for the planner. |

---

## 2. Tutorial: run a planned pick and place

```bash
./build/Robotik-Headless build/plugins/PickAndPlace/scenarios/pick_and_place_pddl_faults.scenario.yaml --trace
```

The camera is down at the start, back at 1 s, then down for good at 3 s. The trace (real output) ends with one line per plan:

```text
  0 s  Detect(red_cube)  FAILED (unavailable: wrist_camera)  (0 s)
  0.51 s  Detect(red_cube)  FAILED (unavailable: wrist_camera)  (0 s)
  1.02 s  Approach(red_cube)  SUCCEEDED  (2.37 s)
  ...
  6.01 s  Release  SUCCEEDED  (0.01 s)
  plan 0 at 0 s (start): solved (detect red_cube) (pick red_cube) (place red_cube box)
  plan 1 at 0.51 s (1. detect(red_cube): skill Detect(red_cube) FAILED (unavailable: wrist_camera)): solved (detect red_cube) (pick red_cube) (place red_cube box)
  plan 2 at 1.02 s (1. detect(red_cube): skill Detect(red_cube) FAILED (unavailable: wrist_camera)): solved (pick red_cube) (place red_cube box)
[PASS] controller.success  (controller.success = 1)
[PASS] inside(red_cube, box)  (inside(red_cube, box) = 1)
...
[PASS] planner.replans >= 1  (planner.replans = 2)
```

Read the plans:

1. At 0 s the robot has seen nothing: the plan starts with `detect`.
2. `Detect` fails (camera down). The executor waits 0.5 s of simulated time and plans again: same plan, same failure.
3. At 1.02 s the camera is back and has already seen the cube. The new plan has no `detect` at all.

In the simulator, open **Demos → Pick and place → PDDL planner**, press Play, and look at the **Planner** panel of the **Mission** workspace (§6).

---

## 3. What goes where

In RobotIK, planning is split across **four places**: the domain file, the world, the scenario YAML, and the mission (C++). Each row below is one job; keep them separate so PDDL stays symbolic and skills stay in C++.

| Job | Where you write it | Example |
|-----|-------------------|---------|
| Vocabulary: types, predicate names, symbolic actions | `scenarios/<name>.domain.pddl` | `(:predicates (holding ?o - graspable))`, `(:action pick ...)` |
| Problem objects (which things planner can name) | World YAML: `kind` on an entity | `kind: cube` on `red_cube` |
| Facts that never change: true for the whole run | Scenario: `planner.facts` | `before(red, black)` for the painter robot. |
| Goal (what must become true) | Scenario: `planner.goal` | `in(red_cube, box)` for the pick-and-place robot |
| Current value of each predicate (from belief) | Mission: one **observer** per predicate | `holding` reads the vacuum sensor |
| Real execution of each PDDL action | Mission: one **binding** per action | `pick` → `Approach`, `Reach`, `Grasp`, `Approach` |

RobotIK builds `problem.pddl` from the world, scenario, and observers before
each search. Bindings are used only at execution time (§4).

### The domain

`plugins/PickAndPlace/scenarios/pick_and_place.domain.pddl`:

```lisp
(define (domain pick_and_place)
  (:requirements :strips :typing :negative-preconditions)

  (:types
    graspable bin - object             ; things you can pick up, and bins you drop into
    cube - graspable)                  ; a cube in the world counts as something pickable

  (:predicates
    (seen ?o - object)                 ; the robot has already looked at this object
    (holding ?o - graspable)           ; the vacuum is holding this cube
    (hand_empty)                       ; the vacuum holds nothing
    (in ?o - graspable ?b - bin))      ; this cube is inside this bin

  ; The robot has not spotted this object yet. Use the camera until it knows
  ; where the object is. Once that is done, the planner skips detect for it.
  (:action detect
    :parameters (?o - object)
    :precondition (not (seen ?o))
    :effect (seen ?o))

  ; The robot knows where the cube is and its hand is empty. Move in, grasp,
  ; and lift. After that it holds the cube and the hand is no longer empty.
  (:action pick
    :parameters (?o - graspable)
    :precondition (and (seen ?o) (hand_empty))
    :effect (and (holding ?o) (not (hand_empty))))

  ; The robot is holding the cube. Carry it above the bin and release. The bin
  ; stays where the world file put it; you do not need a "go to bin" action.
  (:action place
    :parameters (?o - graspable ?b - bin)
    :precondition (holding ?o)
    :effect (and (in ?o ?b) (hand_empty) (not (holding ?o)))))
```

`?o` is a parameter. `cube - graspable` means "a cube is a kind of graspable". The whole PDDL language is available (`forall`, `imply`, derived predicates…): the Painter domain uses `forall` to paint the layers in order.

### The objects: `kind` in the world

```yaml
- id: red_cube
  type: prop
  kind: cube                # type of the PDDL planner (scenarios/*.domain.pddl)
...
- id: box
  type: prop
  kind: bin
```

Only entities with a `kind` are objects of the problem. Their PDDL name is their id in lower case.

### The goal: the scenario

`plugins/PickAndPlace/scenarios/pick_and_place_pddl.scenario.yaml`:

```yaml
  controller:
    planner:
      domain: scenarios/pick_and_place.domain.pddl
      goal:
        - in(red_cube, box)
```

Write one literal per line. A flow list `[ in(red_cube, box) ]` is cut at the comma by YAML.

### The mission: observers and bindings

`plugins/PickAndPlace/src/PickPlacePlanning.cpp` (shortened):

```cpp
// --- State, from the beliefs only --------------------------------------
p_domain.observe("seen", [](RobotContext const& p_context, Args p_args) {
    robotik::WorldObject const* object = p_context.world_model.find(p_args[0]);
    return robotik::truth(object != nullptr && object->observed());
});

p_domain.observe("holding", [](RobotContext const& p_context, Args p_args) {
    return robotik::truth(robotik::heldObject(p_context.robot) == p_args[0]);
});

...

// --- Actions, as skills of PickPlaceMission -----------------------------
p_domain.bind("detect", { "Detect({0})" });
p_domain.bind("pick", { "Approach({0})", "Reach({0})", "Grasp({0})", "Approach({0})" });
p_domain.bind("place", { "Transit({1})", "Approach({1})", "Release" });
```

The mission calls this in `Mission::setup`, on `p_simulation.planning()`. An observer gets the entity ids (`p_args`) and returns `Truth::True`, `Truth::False` or `Truth::Unknown`. A binding lists the behavior tree nodes of the action; `{0}` is its first argument.

A behavior tree node may read a value on its blackboard instead of its name. The Painter gestures read the color:

```cpp
domain.bind("dip", { "Dip", "Home" }, { { "color", "{0}" } });
```

**Everything is checked at load time.** A predicate without observer, fact or `:derived` rule, an action without binding, an unknown `kind`, a goal with a wrong predicate, arity or type: the scenario is refused with the full list, one problem per line, such as:

```text
- predicate 'holding' has no observer (PlanningDomain::observe in the mission) and no fact (planner.facts)
- action 'place' has no binding (PlanningDomain::bind in the mission)
```

---

## 4. How a plan runs

```text
first step, or after a problem
  observers (beliefs) + facts + objects + goal  ->  problem.pddl
  Kairos                                        ->  plan
  plan  ->  behavior tree, one subtree per action:
            the nodes of its binding, then "Check <n>. <action>"
tick the tree
  a skill fails                 -> cancel, wait replan.delay, plan again
  Check: an effect is false     -> cancel, wait replan.delay, plan again
  tree done, goal false         -> wait replan.delay, plan again
  tree done, goal not false     -> success
  replan.max reached, or no valid problem -> failure
```

Before its skills, each subtree asks "Done already?": when every effect of the action already holds in the beliefs (all `True`, none `Unknown`), the step is **skipped**. The beliefs may change between the plan and the step: an object seen on the way does not need a `detect` any more.

The generated tree is the tree of the run: the **Behavior tree** panel shows it, with its colours, as for a written tree.

**Why a step failed.** The reason comes from the scheduler (a resource `unavailable: wrist_camera`, `preempted`…) or from the skill itself: a skill may explain its failure with `Skill::why()` (`red_cube not seen by the camera within 2 s`). The Planner panel, the Skills panel and the headless trace show it.

**Unknown is not false.** For the planner, an atom that is not true is false (PDDL works like that). But when the executor checks an effect or the goal, `Unknown` is accepted: "not seen" does not mean "false". In the fault scenario the camera is down at the end, so the robot cannot see that the cube is in the box: `in` says `Unknown`, the plan ends, and the **assertions** (ground truth) say whether it really worked.

### Steps that run together

The planner gives a list of actions. Many of them do not depend on each
other: the conveyor can bring the next part while an arm carries the last
one. The executor finds these steps alone; the domain says nothing about
parallelism.

Each step has two **events**: its **start** (its skills begin; its
precondition must hold) and its **end** (its skills are done; its effects
hold). Two events of different steps keep the order of the plan when:

- one changes an atom the other reads, or both change it;
- their skills need the same resource and one of them alone (an arm, a cup,
  the conveyor): the first step ends before the second one starts.

Every other pair may happen in any order. The executor then builds one
`Parallel` node with every step; each step waits for the end of the steps
it needs (`After 2, 4: 7. convey(p2, feeder, zone_right)`), runs once, and
the first failure fails
the plan (cancel, plan again, as before). A plan where each step waits for
the previous one keeps its `Sequence`: the PickAndPlace and Painter plans do
not change.

The DualArm conveyor ([09-Demos.md](09-Demos.md#8-dual-arm)) plans 12 steps
for two arms and one conveyor. The same plan, run both ways:

```text
conveyor (parallel: true)
  0 s  Convey(p4, feeder, zone_right)  SUCCEEDED  (2.56 s)
  2.57 s  right.Hover(p4)  SUCCEEDED  (0.95 s)
  2.57 s  Convey(p3, feeder, zone_left)  SUCCEEDED  (2.91 s)
  ...
  5.49 s  left.Hover(p3)  SUCCEEDED  (0.95 s)
  5.49 s  Convey(p2, feeder, zone_right)  SUCCEEDED  (2.91 s)
  ...
[PASS] default                  seed=42     controller succeeded at 16.85 s (oracle perception)  time=16.85

conveyor_sequential (parallel: false)
[PASS] default                  seed=42     controller succeeded at 33.27 s (oracle perception)  time=33.27
```

**Durative actions.** In a temporal domain (`:durative-action`), the
planner gives a start time and a duration to each action. The start event
reads the `at start` and `over all` conditions and changes the `at start`
effects; the end event reads the `at end` and `over all` conditions and
changes the `at end` effects. This gives **required concurrency**: in the
DualArm tray scenario, `drill` needs `(over all (in_motion ?t))`, which
`carry` adds at its start and removes at its end. So the drilling starts once
the carry has started (`After start of 2: 3. drill(right, tray)`), and
the carry ends only once the drilling has ended (`Until 3: 2.
carry(left, tray, pad_a, pad_b)`, a node before its Check). The carry motion
is done at 25.52 s, but the tray stays held until 30.09 s:

```text
  5.53 s  left.Carry(pad_b)  SUCCEEDED  (19.99 s)
  5.53 s  right.Trace(tray)  SUCCEEDED  (24.56 s)
  30.11 s  left.Lower  SUCCEEDED  (1.99 s)
```

The durations of the domain only order the plan: the executor waits for the
skills, not for the clock. A skill that ends early does not wait for its
planned duration.

**Reproducible.** The planner runs inside a simulation step: simulated time does not move during the search. It uses one thread, a fixed search (`search: lama`) and no "anytime" improvement. The same seed gives the same plans. `timeout` is only a safety net on the wall clock.

---

## 5. Reference

### `controller.planner`

| Key | Default | Meaning |
|-----|---------|---------|
| `domain` | required | The PDDL domain, relative to the package root |
| `facts` | none | True atoms that never change and are not observed |
| `goal` | required (or `goal_pddl`) | Literals to reach: `in(a, b)`, `not holding(a)`, `hand_empty` |
| `goal_pddl` | none | The goal in PDDL, for what a list cannot say (`(forall ...)`). Not checked on the beliefs |
| `replan.max` | 3 | New plans allowed after the first one |
| `replan.delay` | 0.5 | Simulated seconds waited before a new plan |
| `verify` | true | Check the effects of each action on the beliefs. `observed`: an effect counts only when a belief was seen again after the start of the step (a failed grasp is not taken for a held object); `false`: no check |
| `parallel` | true | Run together the steps that do not depend on each other ([§4](#steps-that-run-together)); `false` runs the plan step by step |
| `search` | `lama` | Kairos search: `ff`, `lama`, `eager-lama`, `novelty`, `lama-novelty`, `ehc`, `goals` |
| `timeout` | 10 | Wall-clock limit of one search, in seconds |

A scenario has `behavior_tree` **or** `planner`, not both.

Beliefs older than `perception.max_age` (seconds, infinite by default) are unknown to the planner: an observer does not plan on what the robot saw long ago.

### Metrics

| Name | Value |
|------|-------|
| `planner.plans` | Plans made in the run |
| `planner.replans` | Plans made after a problem |
| `planner.steps` | Actions of the current plan |
| `planner.done` | Actions done and checked |

Use them in assertions: `planner.replans == 0` says "the first plan was the
right one".

### C++

| Class (header) | Role |
|----------------|------|
| `PlanningDomain` (`Robotik/Planning/PlanningDomain.hpp`) | `observe(predicate, observer)`, `bind(action, nodes, parameters)` |
| `PlanExecutor` (`Robotik/Planning/PlanExecutor.hpp`) | The planner controller: `plan()`, `done()` (steps finished), `running(step)`, `after()` and `order()` (what each step waits for), `history()`, `observe(context)`, `problem(context)`, `replan(cause)` |
| `orderSteps`, `StepFootprint`, `StepOrder` (`Robotik/Planning/PlanGraph.hpp`) | The partial order of a plan: start and end events, what each one waits for |
| `PlanningEngine` (`Robotik/Planning/PlanningEngine.hpp`) | The planner interface; `makeKairosPlanner()`. No Kairos type leaves `src/Robotik/Planning/Kairos/` |
| `Literal`, `Atom`, `DomainSchema` (`Robotik/Planning/Pddl.hpp`) | Symbols, and what RobotIK reads of a domain |

`Simulation::planning()` gives the `PlanningDomain` to fill;
`Simulation::planner()` the executor, or null without a planner.

### Debugging

- **Simulator**: the **Planner** panel (Mission workspace, beside Skills;
  it comes to the front when a planned scenario is loaded) shows the
  **goal** on top, the state with a progress bar, then the **Plan** (each
  step done, skipped, running or failed with its reason in the tooltip), the
  **Beliefs** (every observed atom, live; tick *Only true* to hide the
  others) and the **History** of the plans and why each was made.
  **Replan** drops the plan. The same symbolic beliefs are in the
  **World model** panel of the Autonomy workspace.
- **Texts**: **Plugin → Files → Planner** shows the **Domain**, the
  generated **Problem**, the planner **Output** and the **Generated tree** of
  one plan (the last one, or the one picked in History). **Save PDDL**
  writes all the texts in `planner/<scenario id>/` of the current folder.
- **Headless**: `--trace` prints one line per plan, as in §2. The summary
  line of each run gives the final state of the planner, the number of plans
  and of replans. `--report r.json` adds a `planner` object to each run:

  ```json
  "planner": {
    "state": "succeeded", "replans": 2,
    "failure": "1. detect(red_cube): skill Detect(red_cube) FAILED (unavailable: wrist_camera)",
    "plans": [
      { "time": 0, "cause": "start", "result": "solved", "expanded": 3,
        "milliseconds": 0.77, "steps": ["detect(red_cube)", "pick(red_cube)", "place(red_cube, box)"] }
    ]
  }
  ```

  `milliseconds` is wall-clock time of the search: it changes from one
  machine to another. Everything else is the same for the same seed.

---

## 6. Limits

- The order between steps is read from the atoms each end of an action
  names. A quantified variable (`forall (?y - zone) ...`) matches any
  object, so a step with a `forall` waits for more steps than needed.
- Two steps that overlap in a temporal plan and need the same exclusive
  resource are not ordered by the executor: the scheduler makes the second
  one wait for the resource.
- Two steps running together share the blackboard of the tree: two
  `parameters` of the same name in parallel bindings would clash.
- The PickAndPlace, Painter and DualArm demos have PDDL scenarios.
  LineFollower and HumanoidWalk do not.

## Next

- [9. Demos](09-Demos.md): the PDDL scenarios of PickAndPlace and Painter.
- [7. API reference](07-API-Reference.md): the other classes a mission uses.
