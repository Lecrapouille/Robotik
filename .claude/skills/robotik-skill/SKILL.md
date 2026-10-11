---
name: robotik-skill
description: Write a RobotIK skill (robotik::Skill), register it on the SkillScheduler with resources/priority/preconditions, call it from a BlackThorn behavior tree (.bt.yml), or make a tree react to faults. Use for skill code, *.bt.yml files, preemption and fault-recovery questions.
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
Verified pattern: `plugins/HelloWorld/scenarios/skills_retry.bt.yml`.
