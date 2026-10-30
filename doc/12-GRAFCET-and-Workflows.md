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

The scenario presses the buttons with `events:`. Dcy and Rearm are **push
buttons** (`pulse: true`: 1 during one cycle, so that their rising edge
counts once); AU is a **latched** button (`value: 1`, then `value: 0` to
release it).

```yaml
# plugins/PickAndPlace/scenarios/pick_and_place_gemma_stop.scenario.yaml
  controller:
    grafcet:
      file: scenarios/pick_and_place_gemma.grafcet.json
      success: Safety:X1 . 2s/Safety:X1 . Control:X0 . Production:X0
      events:
        - { at: 0.0, input: Dcy, pulse: true }   # push button
        - { at: 3.0, input: AU, value: 1 }
```

```text
[PASS] controller.success  (controller.success = 1)
[PASS] collision_count(robot) == 0  (collision_count(robot) = 0)
[PASS] time < 10  (time = 5.02)
[PASS] default                  seed=42     controller succeeded at 5.02 s (perception truth)  time=5.02
pick_and_place_gemma_stop: 1/1 runs passed
```

`pick_and_place_gemma_rearm.scenario.yaml` goes further: AU at 1 s, AU
released at 2 s, Rearm at 2.5 s, Dcy at 3 s, and the cycle ends at 11.55 s
with the cube in the box.

In the simulator, the operator inputs are buttons at the top of the GRAFCET
panel. A click is a push button (Dcy to start a cycle, Rearm), a right click
latches or releases a button (AU for the emergency stop).

## 5. Reference: `controller.grafcet`

| Key | Meaning |
|-----|---------|
| `file` | The `.grafcet.json` file (path from the package root). `grafcet: scenarios/x.grafcet.json` is the short form |
| `workflow` | Instead of `file`: the `.workflow.yaml` file the GRAFCET is generated from ([§7](#7-a-factory)) |
| `gemma` | With `workflow`: `default` adds the default GEMMA (over the `gemma` of the workflow) |
| `success` | Receptivity: the run succeeds when it is true (`X11`). A generated GRAFCET succeeds by default at its `End` step |
| `failure` | Receptivity: the run fails when it is true |
| `inputs` | Operator inputs at the start (`{ Manual: 1 }`); an edge needs a change, use a push button event for it |
| `events` | Operator inputs later: `- { at: 3.0, input: AU, value: 1 }` (latched), `- { at: 0.0, input: Dcy, pulse: true }` (push button: 1 during one cycle) |

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
red). **Bottleneck in 3D** draws the operators of the critical circuit in red
in the 3D view (the entity of each one: a robot with its tool, a belt), so
that you see which machine to make faster or to double. **Measured durations** replaces the declared duration of an operation
that names a `skill:` by the mean duration of this skill in the current run.

### Reference: `x.workflow.yaml`

| Key | Meaning |
|-----|---------|
| `id` | Name of the workflow |
| `operators[].id` | Name used by the routes |
| `operators[].entity` | World entity it stands for (information) |
| `operators[].capacity` | Products it works on at once (default 1) |
| `operators[].take` | Recipe to take a part: skill calls, one step each (§7) |
| `operators[].give` | Recipe to give the part to the next operator (§7) |
| `products[].id` | Product kind |
| `products[].count` | Products of this kind in the cell at once (default 1) |
| `products[].route[]` | `{ name, on, duration, skill }`: operation, operator, declared duration (s), skill measured in runs (optional). In a generated GRAFCET the skill is the action of the step of the operation: it runs `duration` (a belt runs 0.55 s), or until it is done when `duration` is 0 |
| `products[].parts` | World entities of this kind, one per lot (§7) |
| `buffers[]` | `{ id, after, places, zone }`: where parts wait after the operator `after`; `places` gives the entity of each product kind (one part per place); `zone: true` lets one operator at a time in it; `place` instead of `places`: one place for every product kind (the end of a belt) (§7) |
| `output` | World entity receiving the finished parts (the `{to}` of the last operation) |
| `gemma` | `default`: the generated GRAFCET has the default GEMMA (§7) |

The routes must form a **flowshop**: every product visits the operators in the
order of the `operators` list, and skips the ones it does not need. Other
routes (a jobshop) are refused for now.

**Capacity** is a number of tokens, not of machines. `capacity: 2` says that
two *interchangeable* units work at once (two grippers on the same arm, two
spindles); the analysis divides the time of the operator circuit by two. Two
units that are not the same (a fast arm and a slow one) are two operators.
The GRAFCET generator only takes `capacity: 1`, because a GRAFCET step holds
one token: write one operator per unit.

Not done yet: transport between two machines on a belt, and the generation of
distributed code.

## 7. A factory

`plugins/DualArm/scenarios/factory.scenario.yaml` puts both parts together:
the flowshop of `simple.flowshop` (TimedPetriNetEditor) is played by two arms
(machines M1 and M2) on three kinds of parts in two lots. Only the workflow
describes the cell; Robotik **generates** the GRAFCET from it:

```yaml
# plugins/DualArm/scenarios/factory.scenario.yaml
  controller:
    grafcet:
      workflow: scenarios/factory.workflow.yaml   # ends at its End step
```

The workflow gives, on top of the routes and the durations, how each arm
works (its **recipes**), where the parts wait between the arms (a
**buffer**), and the parts of each lot:

```yaml
# plugins/DualArm/scenarios/factory.workflow.yaml (part of it)
  operators:
    - id: M1
      entity: M1
      take: [ "M1.Hover({part})", "M1.Reach({part})",
              "M1.Grasp({part}) . holding(M1, {part})", "M1.Hover({part})" ]
      give: [ "M1.Over({to})", "M1.Release", "M1.Home" ]
    - id: M2
      ...
  buffers:
    - { id: station, after: M1, places: { P1: slot1, P2: slot2, P3: slot3 }, zone: true }
  output: output
  products:
    - id: P1
      parts: [p1a, p1b]
      route:
        - { name: machining, on: M1, duration: 2.0 }
        - { name: machining, on: M2, duration: 1.0 }
```

A recipe is a list of skill calls. Each one becomes a step whose action is
the skill; its receptivity is `done(skill)`, and what follows ` . ` is added
to it (`done(M1.Grasp(p1a)) . holding(M1, p1a)`). `{part}` is the part,
`{to}` its place in the next buffer (or `output` at the end), `{on}` the
operator. For each part on each operator the generator writes:

```text
idle . part done by the previous operator . its place in the next buffer free
  (wait for the zone of the buffer it takes from)  take recipe
  machining: a step with the time receptivity 2s/X10 (the arm holds the part)
  (wait for the zone of the buffer it gives to)    give recipe
  ──> idle (or done after its last part) . "part done by <operator>" . zone free
every operator done ──> End
```

The AND convergences say what the flowshop says: a machine works on one part
at a time, in the order of the products and lot after lot; a part goes to M2
once machined by M1; the place of a kind in the station is free again once M2
took the part. The station is a **zone**: one arm at a time in it, M2 first
(the operator taking parts goes first, so the line empties: the zone
receptivity of M1 is `/(X.. + X..)` on the waiting steps of M2). For the
factory this makes 127 steps; the run is the same as with the GRAFCET drawn
by hand before (50.23 s). `factory.workflow.yaml` also gives the cycle time of
the machining alone (6 s, M1 is the bottleneck); the run adds the moves of
the arms.

In the simulator, **Save .grafcet.json** in the GRAFCET panel writes the
generated GRAFCET next to the workflow (`factory.grafcet.json`), to open it
in TimedPetriNetEditor.

**Fed by a belt.** `factory_belt.scenario.yaml` puts the six parts on a
conveyor belt ([chapter 3](03-Worlds-and-Scenarios.md), `type: conveyor`),
one pitch apart. The belt is the first operator of the workflow: its
operation runs the skill of the belt for 0.55 s, when the pick place at its
end is free; M1 takes the part there.

```yaml
# plugins/DualArm/scenarios/factory_belt.workflow.yaml (part of it)
  operators:
    - id: feeder
      entity: feeder
    - id: M1
      ...
  buffers:
    - { id: pick, after: feeder, place: pick }   # the end of the belt, one part
    ...
  products:
    - id: P1
      parts: [p1a, p1b]
      route:
        - { name: bring, on: feeder, skill: feeder_belt.Run, duration: 0.55 }
        - { name: machining, on: M1, duration: 2.0 }
        - { name: machining, on: M2, duration: 1.0 }
```

The belt runs six times, each time just after M1 took the last part
(`0 s`, `2.57 s`, `9.7 s` ...), and the run ends at 50.66 s. 0.55 s at 0.2 m/s
is 11 cm, a little more than the pitch: the stop at the end of the belt holds
the part, the next ones come closer.

**With a GEMMA.** `gemma: default` (in the workflow, or in the `grafcet:`
block of the scenario) adds the graphs Safety and Control of the default
GEMMA of TimedPetriNetEditor before the Production: A1 initial stop, F1
normal production, A2 end of cycle, D1 emergency stop, A5 and A6 to restart.
A part starts only while Control is in F1. `Stop` lets the arms finish the
parts they hold; Control leaves A2 for A1 when every arm is between two
parts. The operator inputs are `Start`, `Stop`, `EStop`, `Rearm`, `Ack` and
`InitialPosition`: buttons of the GRAFCET panel, or `events:` of the
scenario.

```yaml
# plugins/DualArm/scenarios/factory_gemma.scenario.yaml
    grafcet:
      workflow: scenarios/factory.workflow.yaml
      gemma: default
      success: Control:X0 . 1s/Control:X0   # back to A1 (initial stop) for 1 s
      events:
        - { at: 0.0, input: Start, pulse: true }
        - { at: 20.0, input: Stop, pulse: true }
```

The run: after `Stop` at 20 s, M1 puts p3a in the station and M2 drops p2a;
Control is back in A1 at 22.09 s and the run succeeds at 23.09 s. p3a waits in
the station and lot b stays in the stock. After an emergency stop, Safety
puts Production back in its initial situation: the parts must be put back
too (A6, `InitialPosition`). For now the GEMMA is the one of the whole cell,
not one per machine: the machines share the station, so their sequences are
joined in one Production graph.

The generator refuses, with a message: a workflow without parts, an operator
of capacity above 1, a part going from an operator to the next one without a
buffer after the first, a buffer without a place for the product, `{to}` in
the last recipe without `output`, and an unknown placeholder.

**A behavior tree or a GRAFCET?** A plan, a behavior tree and a GRAFCET can
say the same order. A tree with only `Sequence` and `Parallel` says only
series-parallel orders. The planner of Robotik writes any partial order: a
`Parallel` of the steps, each one waiting for the events it needs
([chapter 11](11-PDDL-Planner.md)). The GRAFCET says it with AND convergences.
Choose by the job: the planner and the trees for robots that decide; the
GRAFCET and the GEMMA for a factory cell, the language of automation
engineers.

## 8. From C++

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

// The GRAFCET of the cell (needs recipes, buffers and parts).
#include "Robotik/Task/Workflow/WorkflowGrafcet.hpp"
std::vector<tpne::Net> graphs;
std::string const end = robotik::workflowGrafcet(workflow, graphs).take();
```

The GRAFCET controller is `robotik::GrafcetController`
(`include/Robotik/Task/Grafcet/GrafcetController.hpp`), built by the `Cell`
from `controller.grafcet`; `Cell::grafcet()` gives it (null otherwise).

## Next

- [Chapter 5](05-Skills-and-Behavior-Trees.md): the skills a GRAFCET requests.
- [Chapter 11](11-PDDL-Planner.md): observers, also read by receptivities.
