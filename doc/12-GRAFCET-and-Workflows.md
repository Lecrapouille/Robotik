# 12. GRAFCET, GEMMA and workflows

What you will learn:

- drive a robot cell with a **GRAFCET** drawn in TimedPetriNetEditor, as
  automation engineers do, instead of a behavior tree;
- describe the start and stop modes of a machine with a **GEMMA**;
- describe the **workflow** of a cell (who works, on what, how long) and get
  its **cycle time** and its **bottleneck** without running the robots.

Both use [TimedPetriNetEditor](https://github.com/Lecrapouille/TimedPetriNetEditor)
(`external/TimedPetriNetEditor`): its GRAFCET engine and its (max,+) algorithms
are compiled into RobotIK, behind `Task/Grafcet` and `Task/Workflow`.

---

## 1. What is a GRAFCET?

A GRAFCET (IEC 60848, also called SFC, Sequential Function Chart) is a chart
of **steps** and **transitions**:

```text
   ┌────┐
   ║ X0 ║ Home           <- initial step (double square): active at start
   └─┬──┘   action: Home (requests the skill Home)
     ┼  done(Home)       <- transition with its receptivity (a condition)
   ┌─┴──┐
   │ X1 │ Detect         action: Detect(red_cube)
   └─┬──┘
     ┼  done(Detect(red_cube))
    ...
```

- A **step** is active or not. An active step does its **actions**.
- A **transition** fires when all the steps above it are active and its
  **receptivity** (a boolean condition) is true. Then the steps above become
  inactive and the steps below become active.
- All the transitions that can fire fire together. After a change, the
  graphs evolve until nothing can fire (they are **stable**), then the
  actions are computed.

In RobotIK an **action named like a skill** requests the skill: when the
action becomes true the skill is requested, when it becomes false while the
skill still runs, the skill is cancelled.

## 2. Tutorial: pick and place with a GRAFCET

Open `plugins/PickAndPlace/scenarios/pick_and_place.grafcet.json` in
TimedPetriNetEditor (`File > Open`). It has 12 steps, one per skill of
`pick_and_place.bt.yaml`, and a **Retry** step: when the camera is lost,
`failed(Detect(red_cube))` leads to Retry, which waits 500 ms (`500ms/X2`)
and goes back to Detect.

The scenario only names the file and the step that ends the run:

```yaml
# plugins/PickAndPlace/scenarios/pick_and_place_grafcet.scenario.yaml
  controller:
    grafcet:
      file: scenarios/pick_and_place.grafcet.json
      success: X11              # the End step

  faults:
    - { at: 0.0, resource: wrist_camera, action: disable }
    - { at: 1.0, resource: wrist_camera, action: restore }
```

Run it:

```bash
./build/Robotik-Headless build/plugins/PickAndPlace/scenarios/pick_and_place_grafcet.scenario.yaml
```

```text
  0 s  Home  SUCCEEDED  (0 s)
  0.01 s  Detect(red_cube)  FAILED (unavailable: wrist_camera)  (0 s)
  0.52 s  Detect(red_cube)  FAILED (unavailable: wrist_camera)  (0 s)
  1.03 s  Detect(red_cube)  SUCCEEDED  (0.01 s)
  1.05 s  Approach(red_cube)  SUCCEEDED  (2.37 s)
  3.43 s  Reach(red_cube)  SUCCEEDED  (0.5 s)
  3.94 s  Grasp(red_cube)  SUCCEEDED  (0.01 s)
  3.96 s  Approach(red_cube)  SUCCEEDED  (0.49 s)
  4.46 s  Transit(box)  SUCCEEDED  (1.07 s)
  5.54 s  Approach(box)  SUCCEEDED  (0.49 s)
  6.04 s  Release  SUCCEEDED  (0.01 s)
  6.06 s  Home  SUCCEEDED  (2.37 s)
[PASS] controller.success  (controller.success = 1)
[PASS] inside(red_cube, box)  (inside(red_cube, box) = 1)
[PASS] holding(gripper) == 0  (holding(gripper) = 0)
[PASS] collision_count(robot) == 0  (collision_count(robot) = 0)
[PASS] time < 30  (time = 8.45)
[PASS] default                  seed=7      controller succeeded at 8.45 s (perception truth)  time=8.45
pick_and_place_grafcet: 1/1 runs passed
```

Detect fails twice (camera off), the Retry step waits 500 ms each time, then
the cycle goes on. In the simulator, the **GRAFCET** panel of the Mission
workspace shows the graphs: active steps outlined in green, running actions
in green.

## 3. What a receptivity can read

A receptivity reads the **beliefs** only, never the truth of the simulator
(as skills and the planner):

| Name | Value |
|------|-------|
| `done(Skill)` | The run of the skill requested by this GRAFCET succeeded |
| `failed(Skill)` | It failed or was cancelled |
| `running(Skill)` | The skill waits or runs |
| `holding(red_cube)`, `in(red_cube, box)` ... | A predicate of the planning domain of the mission, computed by its observer ([chapter 11](11-PDDL-Planner.md)) |
| any other name (`Dcy`, `AU`) | An **operator input**: a button. Set by the scenario, the simulator panel, or the mission |

`done(S)` of a skill that is an action of the GRAFCET only tells about the run
the GRAFCET requested: when the same skill is used twice (Approach before and
after Grasp), the second step does not see the result of the first run.

The syntax (IEC 60848, written in ASCII):

| What | Syntax |
|------|--------|
| and, or, not | `a . b`, `a + b`, `/a` (also `*`, `!`, `and`, `or`, `not`) |
| constants | `1`, `0`, `true`, `false` |
| step 3 is active | `X3`, or `Safety:X3` for a step of another graph |
| rising, falling edge | `↑a` (`^a`, `rising(a)`), `↓a` (`falling(a)`) |
| time | `3s/X2` (true 3 s after X2 became active), `2s/a/5s` (and false 5 s after `a` became false); units `ms`, `s`, `min`, `h` |
| integer input | `[count >= 3]` |

Actions have a qualifier: `N` (while the step is active), `S`/`R` (set and
reset a stored action), `D` (delayed), `L` (limited), `SD`, `DS`, `SL`,
`P1`/`P0` (pulse when the step becomes active / inactive), and an optional
condition (conditional action).

## 4. Tutorial: a GEMMA

A GEMMA (Guide d'Étude des Modes de Marches et d'Arrêts) describes how a
machine starts, stops and handles an emergency. Here it is a GRAFCET of three
graphs (`pick_and_place_gemma.grafcet.json`):

```text
Safety      X0 Normal  --AU-->  X1 D1 (emergency stop): Stop skill,
                                   forcing Control:{init}, Production:{init}
            X1  --/AU . ↑Rearm-->  X0
Control     X0 A1 initial stop  --↑Dcy . Safety:X0-->  X1 F1 production
            X1  --Production:X12-->  X2 A2 end of cycle  --Production:X0-->  X0
Production  X0 Wait  --Control:X1-->  the pick and place  -->  X12 End
            X12  --Control:X2-->  X0
```

A **forcing order** (`Production:{init}`) is an action of a step: while the
step is active, the forced graph is held in its initial situation and does
not evolve. This is how the safety graph stops everything.

The scenario presses the buttons: `inputs:` at the start, `events:` later.

```yaml
# plugins/PickAndPlace/scenarios/pick_and_place_gemma_stop.scenario.yaml
  controller:
    grafcet:
      file: scenarios/pick_and_place_gemma.grafcet.json
      success: Safety:X1 . 2s/Safety:X1 . Control:X0 . Production:X0
      inputs: { Dcy: 1 }
      events:
        - { at: 3.0, input: AU, value: 1 }
```

```text
[PASS] controller.success  (controller.success = 1)
[PASS] collision_count(robot) == 0  (collision_count(robot) = 0)
[PASS] time < 10  (time = 5.02)
[PASS] default                  seed=42     controller succeeded at 5.02 s (perception truth)  time=5.02
pick_and_place_gemma_stop: 1/1 runs passed
```

In the simulator, the operator inputs are buttons at the top of the GRAFCET
panel: press **Dcy** to start a cycle, **AU** for the emergency stop.

## 5. Reference: `controller.grafcet`

| Key | Meaning |
|-----|---------|
| `file` | The `.grafcet.json` file (path from the package root). `grafcet: scenarios/x.grafcet.json` is the short form |
| `success` | Receptivity: the run succeeds when it is true (`X11`) |
| `failure` | Receptivity: the run fails when it is true |
| `inputs` | Operator inputs at the start (`{ Dcy: 1 }`) |
| `events` | Operator inputs later: `- { at: 3.0, input: AU, value: 1 }` |

Without `success` nor `failure`, the GRAFCET runs until the `timeout` of the
scenario, and the assertions decide. A scenario has one controller: a
behavior tree, a planner or a GRAFCET.

Errors found when the scenario is loaded: a syntax error (with its column),
an action that is not a skill, `done(S)` on an unknown skill, an unknown step.

## 6. Workflows: how fast can the cell work?

A **workflow** (`x.workflow.yaml`) describes the cell with the words of
[SynDEx](https://www.syndex.org/): the **operators** (robots, machines,
cameras) and, for each kind of **product**, its route: the operations, on
which operator, how long.

```yaml
# plugins/DualArm/scenarios/conveyor.workflow.yaml
workflow:
  id: conveyor_line
  operators:
    - { id: conveyor, entity: conveyor }
    - { id: left, entity: left }
    - { id: right, entity: right }
  products:
    - id: part_a
      count: 2                  # parts of this kind in the cell at once
      route:
        - { name: bring, on: conveyor, duration: 1.5 }
        - { name: sort to bin_left, on: left, duration: 5.0 }
    - id: part_b
      count: 2
      route:
        - { name: bring, on: conveyor, duration: 1.5 }
        - { name: sort to bin_right, on: right, duration: 6.0 }
```

RobotIK turns it into a **timed event graph** (a Petri net where every place
has one input and one output). In the (max,+) algebra the cell repeats with a
period, the **cycle time** λ. The longest circuit of the graph, the
**critical circuit**, is the bottleneck: shorten one of its operations and λ
goes down; shorten another one and nothing changes.

```bash
./build/Robotik-Headless build/plugins/DualArm/scenarios/conveyor.workflow.yaml
```

```text
workflow conveyor_line: 3 operators, 2 products
cycle time: 6 s
throughput: 20 products/min
critical circuit:
  part_b: sort to bin_right (right)
earliest schedule (2 cycles):
     0.00 s ..    1.50 s  conveyor   part_a     bring #1
     1.50 s ..    6.50 s  left       part_a     sort to bin_left #1
     1.50 s ..    3.00 s  conveyor   part_b     bring #1
     3.00 s ..    4.50 s  conveyor   part_a     bring #2
     3.00 s ..    9.00 s  right      part_b     sort to bin_right #1
     4.50 s ..    6.00 s  conveyor   part_b     bring #2
     6.50 s ..   11.50 s  left       part_a     sort to bin_left #2
     9.00 s ..   15.00 s  right      part_b     sort to bin_right #2
```

The right arm needs 6 s per part B: it sets the period. Make it faster, or
give it a second gripper (`capacity: 2`), and the cycle time goes down.

In the simulator, the **Workflow** panel of the Mission workspace shows the
same results with a Gantt chart and the event graph (critical circuit in
red). **Measured durations** replaces the declared duration of an operation
that names a `skill:` by the mean duration of this skill in the current run.

### Reference: `x.workflow.yaml`

| Key | Meaning |
|-----|---------|
| `id` | Name of the workflow |
| `operators[].id` | Name used by the routes |
| `operators[].entity` | World entity it stands for (information) |
| `operators[].capacity` | Products it works on at once (default 1) |
| `products[].id` | Product kind |
| `products[].count` | Products of this kind in the cell at once (default 1) |
| `products[].route[]` | `{ name, on, duration, skill }`: operation, operator, declared duration (s), skill measured in runs (optional) |

The routes must form a **flowshop**: every product visits the operators in the
order of the `operators` list, and skips the ones it does not need. Other
routes (a jobshop) are refused for now. Not done yet: transport media and
buffers between operators (the media of SynDEx), and the generation of
distributed code.

## 7. From C++

```cpp
#include "Robotik/Package/Scenario/Workflow.hpp"
#include "Robotik/Task/Workflow/WorkflowAnalysis.hpp"

robotik::WorkflowDescription workflow = robotik::loadWorkflow(file).take();
// Optional: the durations measured in a run (skills of the routes).
robotik::useMeasuredDurations(workflow,
                              robotik::measuredDurations(cell.skills()));
robotik::WorkflowAnalysis const analysis =
    robotik::analyseWorkflow(workflow).take();
std::cout << analysis.cycle_time.value() << " s\n";
```

The GRAFCET controller is `robotik::GrafcetController`
(`include/Robotik/Task/Grafcet/GrafcetController.hpp`), built by the `Cell`
from `controller.grafcet`; `Cell::grafcet()` gives it (null otherwise).

## Next

- [Chapter 5](05-Skills-and-Behavior-Trees.md): the skills a GRAFCET requests.
- [Chapter 11](11-PDDL-Planner.md): observers, also read by receptivities.
