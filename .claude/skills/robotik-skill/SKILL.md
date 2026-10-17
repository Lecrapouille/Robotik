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
    std::string why() const override;   // optional: why the last FAILURE ("cube not seen within 2 s")
};
```

`RobotContext { Robot& robot; WorldModel& world_model; Seconds time, dt; }`
— no simulator, no ground truth. Read objects with
`p_context.world_model.find("cube")` (`observed()`, `position`, `size`,
`confidence`). Reuse inside skills: `MoveTCPSkill(frame = robot.tool(),
target)` + `goal(pose)` (IK once per goal), `MoveJointSkill(joint, target)`,
`MoveJointsSkill({{"joint1", 0.5}})`, `HomeSkill(group)`.
Joints: `joints.find("j")` (id, doubles) or `joints.revolute("j")` (typed,
Radians); `moveTo`, `spin`, `push`, `reached(j, tol)`, `hold()`.
Example: `plugins/HelloWorld/src/WaveSkill.cpp`; bigger: `plugins/Painter/src/PainterSkills.cpp`.

## Register (in Mission::setup)

```cpp
robotik::SkillDescription d;
d.name = "Wave";                                       // = BT Action name, unique
d.resources = { p_sim.robot().resources().require("arm"),
                p_sim.robot().resources().require("cam", robotik::Access::Shared) };
d.priority = 100;          // higher admitted first, may preempt (Stop = highest)
d.wait = true;             // wait when busy/precondition false (else fail at once)
d.cancellable = true;      // false: cannot be preempted
d.preconditions = { { "gripper empty", [](robotik::RobotContext const& c) { return ...; } } };
p_sim.skills().add<WaveSkill>(std::move(d) /*, ctor args */);
```

Built-in per simulation: `Home` (needs a `joint_group`), `Stop` (all
actuators, top priority, never ends). Every skill becomes a BT `Action` of
the same name automatically (`registerSkills`). Custom nodes:
`p_sim.factory().registerNode("Dip", []{...})` (see Painter `registerNodes`).

States: Idle, Waiting, Running, Succeeded, Failed, Cancelled. Reasons: Busy,
Unavailable (fault before start), Precondition, ResourceLost (fault while
running), Preempted, Cancelled, Failed. `--trace` prints them.
Direct use: `skills.request(skills.find("Home"))`, `state(id)`, `cancel(id)`.

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
(one-element list). Action maps Waiting/Running→RUNNING,
Succeeded→SUCCESS, Failed/Cancelled→FAILURE; leaving a running node cancels it.

**Do not use `Timeout`, `Delay`, `Wait`, `Cooldown`**: BlackThorn measures
them with `std::chrono::steady_clock` (wall time), so results differ between
headless and simulator. Put durations in skills (use `dt`).

Faults: declare in scenario `faults:` or experiment `setup: inject_fault`.
No fault code in skills; the tree reacts (retry, fallback, abort).
Verified pattern: `plugins/HelloWorld/scenarios/skills_retry.bt.yaml`.

## PDDL planner instead of a tree

Scenario `controller.planner` (goal) + `scenarios/<x>.domain.pddl` + world
`kind:`. In `Mission::setup`: `p_simulation.planning().observe("holding",
[](RobotContext const&, Args a){ return truth(...); })` per predicate
(beliefs only; `Truth::Unknown` when not observable), and
`bind("pick", {"Approach({0})", "Grasp({0})"}, {{"color", "{0}"}})` per
action (BT node names; parameters go to the step blackboard). Missing
observer/binding/kind → scenario refused with the list. Examples:
`plugins/PickAndPlace/src/PickPlacePlanning.cpp`, Painter `registerPlanning`.
Metrics `planner.plans|replans|steps|done`. Details: doc/11-PDDL-Planner.md.
