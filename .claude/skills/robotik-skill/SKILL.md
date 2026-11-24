---
name: robotik-skill
description: Write a RobotIK skill (robotik::Skill), register it on the SkillScheduler with resources/priority/preconditions, call it from a BlackThorn behavior tree (.bt.yaml), or make a tree react to faults. Use for skill code, *.bt.yaml files, preemption and fault-recovery questions.
---

# RobotIK skills and behavior trees

Tree decides *what*, scheduler decides *if now*, skill knows *how*.

## Skill

```cpp
class WaveSkill final: public robotik::Skill
{
public:
    void reset() override;                                                    // each new run
    robotik::Status tick(robotik::RobotContext& p_context, Seconds p_dt) override; // RUNNING/SUCCESS/FAILURE
    void cancel(robotik::RobotContext& p_context) override;                   // safe stop: joints().hold()
    robotik::Status stopping(robotik::RobotContext&, Seconds) override;       // optional: RUNNING keeps the resources until safe
    std::string why() const override;   // optional: why the last FAILURE ("cube not seen within 2 s")
};
```

`RobotContext { Robot& robot; WorldModel& world_model; Seconds time, dt; }`
— no simulator, no ground truth. Read objects with
`p_context.world_model.find("cube")` (`observed()`, `position`, `size`,
`confidence`). Reuse inside skills: `MoveTCPSkill(frame = robot.tool(),
target)` + `goal(pose)` (IK once per goal), `MoveJointSkill(joint, target)`,
`MoveJointsSkill({{"joint1", 0.5}})`, `HomeSkill(group)`.
Joints: `joints.find("j")` (id, doubles) or `joints.requireRevolute("j")` (typed,
Radians); `moveTo`, `spin`, `push`, `reached(j, tol[, speed])` (also
|velocity| <= SETTLED_SPEED), `hold()`. Path through waypoints:
`MoveTCPSkill::speed(inf)` on the inner points (Painter `follow`).
Writes are checked: commanding a joint/pump whose resource another skill
holds throws `std::logic_error` (WriterScope set by the scheduler). A free
resource is writable; a ScenarioLogic that must keep joints takes a lease
(`resources.acquire`, owner ANONYMOUS).
Resumable: a preempted skill (wait = true) starts again from `reset()`:
reset clears all run state, tick reads joints/beliefs now.
Example: `plugins/HelloWorld/src/WaveSkill.cpp`; bigger: `plugins/Painter/src/PainterSkills.cpp`.
Vacuum cup: `holding()` is measured (the vacuum), not what is held. The
grasp skill writes what it went to take with `cup->label("red_cube")` once
`holding()` is true; losing the vacuum clears it. Observers read
`holding()` for "hand empty" and `label()` for "holding X". The entity really
held is truth: `RoboticCell::held(cup)`, for metrics and the simulator only.

## Register (in ScenarioLogic::setup)

```cpp
robotik::SkillDescription d;
d.name = "Wave";                                       // = BT Action name, unique
d.resources = { p_sim.robot().resources().require("arm"),
                p_sim.robot().resources().require("cam", robotik::Access::Shared) };
d.priority = 100;          // higher admitted first, may preempt (Stop = highest)
d.wait = true;             // wait when busy/precondition false (else fail at once)
d.cancellable = true;      // false: cannot be preempted
d.urgent = false;          // true: victims end at once, no stopping phase (built-in Stop)
d.stop_timeout = Seconds(1.0); // longest stopping phase, then forced
d.preconditions = { { "gripper empty", [](robotik::RobotContext const& c) { return ...; } } };
p_sim.skills().add<WaveSkill>(std::move(d) /*, ctor args */);
```

Built-in per simulation: `Home` (needs a `joint_group`), `Stop` (all
actuators, top priority, never ends). Every skill becomes a BT `Action` of
the same name automatically (`registerSkills`). Custom nodes:
`p_cell.factory().registerNode("Dip", []{...})` (see Painter `registerNodes`);
include `RobotIK/Task/Skills/SkillNodes.hpp`, the only header that brings
BlackThorn (others name `bt::` types via `TreeTypes.hpp`; `RoboticCell::status()`
is a `robotik::Status`, `toStatus(bt::Status)` converts).

States: Idle, Waiting, Running, Stopping, Succeeded, Failed, Cancelled.
Reasons: Busy, Queued (another run of the same skill), Preempting (waits for
victims to stop), Unavailable (fault before start), Precondition,
ResourceLost (fault while running, ends at once), Preempted (wait = true:
back to Waiting; else Cancelled), Cancelled, Failed. `--trace` prints them.
Direct use: `RunId run = skills.request(skills.find("Home"))`, `state(run)`,
`cancel(run)` (own run only); `cancelAll(id)` every run of a skill;
`cancelAll()` everything at once. One run per skill, others queued.
Preemption is all or nothing (every holder of every conflict, or none).

## Behavior tree (BlackThorn YAML)

```yaml
BehaviorTree:
  Sequence:
    name: Main
    children:
      - Action: { name: Home }
      - UntilSuccess:            # retry until the fault clears
          attempts: 1000
          child:
            - Action: { name: Wave }
      - Selector:                # fallback: first child that succeeds
          children:
            - Action: { name: Grasp(red_cube) }
            - Action: { name: Home }
```

Nodes: `Sequence`, `Selector` (not "Fallback"), `Parallel`
(`success_threshold`, `failure_threshold`), `Inverter`, `ForceSuccess`,
`ForceFailure`, `UntilSuccess`/`UntilFailure` (`attempts`), `Repeater`
(`times`), `Action` (`name`). Composites: `children:`; decorators: `child:`
(one-element list). Action maps Waiting/Running/Stopping→RUNNING,
Succeeded→SUCCESS, Failed/Cancelled→FAILURE; leaving a running node cancels
its own ticket only. At the end of a run the node writes `<skill>.state`
(`FAILED`) and `<skill>.reason` (`resource lost`, `cancelled`...) on the
blackboard: a condition can tell a fault from a failure.

`Timeout`, `Delay`, `Wait`, `Cooldown` count simulated time: `RoboticCell`
and `PlanExecutor` give the tree their clock (`bt::Tree::clock`), so they
are reproducible. A tree you build yourself keeps the wall clock unless you
call `clock()`.

Faults: declare in scenario `faults:` or experiment `setup: inject_fault`.
No fault code in skills; the tree reacts (retry, fallback, abort).
Verified pattern: `plugins/HelloWorld/scenarios/skills_retry.bt.yaml`.

## PDDL planner instead of a tree

Scenario `controller.planner` (goal) + `scenarios/<x>.domain.pddl` + world
`kind:`. In `ScenarioLogic::setup`: `p_simulation.planning().observe("holding",
[](RobotContext const&, Args a){ return truth(...); })` per predicate
(beliefs only; `Truth::Unknown` when not observable), and
`bind("pick", {"Approach({0})", "Grasp({0})"}, {{"color", "{0}"}})` per
action (BT node names; parameters go to the step blackboard). Missing
observer/binding/kind → scenario refused with the list. Examples:
`plugins/PickAndPlace/src/PickPlacePlanning.cpp`, Painter `registerPlanning`.
Metrics `planner.plans|replans|steps|done`. Details: doc/concepts/05-pddl-planner.md.
`method: pabt` (PA-BT): no search, a reactive tree grows from the goal with
the same observers and bindings (`Task/Planning/Backchaining.hpp`); reads
only plain preconditions, reaches the goal conditions one after the other.

Independent steps run together (`planner.parallel: true`, default): order
from what each end of an action reads/changes + exclusive resources of the
bound skills; durative actions give required concurrency (`over all`).
Several robots: `SkillDescription::robot = "left"`, name skills `left.X(...)`,
bind `"{0}.X({1})"` with the arm as first PDDL argument; observers see the
other robots in `context.team`; beliefs are in the world frame (bring them
into the base: `context.robot.base().pose.inverse() * p`). Example:
`plugins/DualArm/src/ConveyorLogic.cpp`, `TrayLogic.cpp`.
