# 5. Skills, the scheduler and behavior trees

Lesson 4 of [04-Your-First-Plugin.md](04-Your-First-Plugin.md) showed a skill
called by a behavior tree. This page explains the three pieces, how they talk,
and how to write your own.

The one sentence to remember:

> **The behavior tree decides *what* to do next.
> The scheduler decides *if it can run now*.
> The skill knows *how* to do it.**

```text
Behavior tree        "I want Grasp(red_cube) now."
      │ request
      ▼
SkillScheduler       "The gripper is free, the arm is free, the preconditions
      │               hold: you can start."   (or: "wait", or: "failed")
      │ tick
      ▼
Skill                "I move the arm a little toward the cube."
      │ commands
      ▼
Robot API            joints, actuators, sensors
```

---

## 1. What is a skill?

A skill is **one capability** of the robot: `Home`, `MoveJoint`, `Detect`,
`Grasp`, `FollowLine`. It can be very small (move one joint) or bigger
(`Pick` = detect, approach, reach, grasp, lift).

A skill is **not a thread**. It is an object with a `tick` method, called
once per simulation step until it returns `SUCCESS` or `FAILURE`. Hundreds of
skills can exist without one thread each.

```cpp
class Skill
{
public:
    virtual void reset();                                          // a new run starts
    virtual Status tick(RobotContext& p_context, Seconds p_dt) = 0; // one step of work
    virtual void cancel(RobotContext& p_context);                   // stop safely
};
```

| Method | Called | Your job |
|--------|--------|----------|
| `reset()` | Each time the skill starts | Clear the state of the last run |
| `tick(context, dt)` | Each step while running | Command the robot a little; return `RUNNING`, `SUCCESS` or `FAILURE` |
| `cancel(context)` | When cancelled, preempted, or a resource is lost | Put the hardware in a safe state (stop the joints, open the gripper...) |

A skill sees only the `RobotContext`:

```cpp
struct RobotContext
{
    Robot& robot;              // joints, sensors, actuators, resources, kinematics
    WorldModel& world_model;   // what the robot believes about the objects
    Seconds time{};            // robot clock at the start of the tick
    Seconds dt{};              // duration of the tick
};
```

There is **no simulator in there**. A skill cannot tell MuJoCo from a real
robot, and it cannot read the ground truth. This is on purpose: the same skill
must work on a real robot.

### Built-in skills

| Skill | Class | Notes |
|-------|-------|-------|
| `Home` | `HomeSkill` | Added by every simulation that has a `joint_group` actuator. Moves the joints to `home` |
| `Stop` | `StopSkill` | Added by every simulation. Reserves **all** actuators with the highest priority and holds the joints: an emergency stop. It never ends by itself (always `RUNNING`) |
| — | `MoveJointSkill(joint, target)` | One joint to an angle |
| — | `MoveJointsSkill(posture)` | Several joints, `{ "joint1": 0.5, ... }` |
| — | `MoveTCPSkill(frame)` + `goal(pose)` | A frame to a pose (IK once per goal). Useful **inside** your own skills |

Pick-and-place skills (`DetectSkill`, `ApproachSkill`, `GraspSkill`,
`ReleaseSkill`) are in `plugins/PickAndPlaceBT/src`.

---

## 2. Register a skill: `SkillDescription`

A skill is added to the scheduler with a description:

```cpp
robotik::ResourceManager const& resources = p_simulation.robot().resources();

robotik::SkillDescription grasp;
grasp.name = "Grasp(red_cube)";                       // unique; also the BT action name
grasp.resources = { resources.require("arm"),
                    resources.require("gripper"),
                    resources.require("wrist_camera", robotik::Access::Shared) };
grasp.priority = 100;                                 // higher runs first, may preempt
grasp.wait = true;                                    // wait while busy (default)
grasp.cancellable = true;                             // may be preempted (default)
grasp.preconditions = {
    { "gripper is empty",
      [](robotik::RobotContext const& c) { return !gripperHolds(c); } } };

p_simulation.skills().add<GraspSkill>(std::move(grasp), /* constructor args */ "red_cube");
```

Do this in `Mission::setup`. The simulation then makes every skill a behavior
tree action with the same name.

| Field | Meaning |
|-------|---------|
| `name` | Unique name. The behavior tree calls the skill by this name |
| `resources` | What the skill holds for its whole run. `Exclusive` (default) or `Shared` |
| `priority` | Higher first. A higher-priority request may **preempt** a running skill |
| `wait` | `true`: wait while a resource is busy or a precondition is false. `false`: fail at once |
| `cancellable` | `false` forbids preemption |
| `preconditions` | Conditions checked before the start. The text is shown when the skill is blocked |

---

## 3. Resources

A resource is a part of the robot that skills must share: an actuator or a
sensor. Each actuator and each sensor of the world file is a resource **with
the same name**: `arm`, `gripper`, `wrist_camera`.

```text
Grasp                    LookAround
 ├── arm      exclusive   └── head_camera  shared
 ├── gripper  exclusive
 └── camera   shared
```

- `Grasp` and `LookAround` can run **at the same time**: no conflict.
- `MoveArm` (arm) and `Grasp` (arm) **cannot**: the second one waits, or fails,
  or preempts the first (if its priority is higher).
- Two skills can share a camera stream (`Shared`).

In the simulator, the **Timeline** panel (Mission workspace) shows each
resource as a chip: grey when free, orange when a skill holds it, red when
it is in fault. Click a chip to inject a fault or to clear it.

---

## 4. The scheduler

`SkillScheduler` is the referee. It does not know how to move an arm. It knows
**if and when** a skill may run.

At each update it:

1. stops the runs that lost a resource (fault);
2. admits waiting requests by priority, if resources and preconditions allow;
3. preempts lower-priority skills when needed;
4. ticks the running skills.

```text
           request()
               │
               ▼
           Waiting ───── resource busy / precondition false (wait = true)
               │
     resources free and preconditions true
               │
               ▼
           Running ── tick() ── tick() ── ...
           │     │        │
      SUCCESS  FAILURE  cancel / preempted / resource lost
           │     │        │
           ▼     ▼        ▼
     Succeeded Failed  Cancelled / Failed
```

| `SkillState` | Meaning |
|--------------|---------|
| `Idle` | Never requested since the last reset |
| `Waiting` | Requested, not started (busy resource or false precondition) |
| `Running` | Holds its resources, ticked each update |
| `Succeeded` / `Failed` / `Cancelled` | End of the last run |

`SkillReason` says why: `Busy`, `Unavailable`, `Precondition`, `ResourceLost`,
`Preempted`, `Cancelled`, `Failed`. `--trace` prints them:

```text
  0.01 s  Wave  FAILED (resource lost: arm)  (0.99 s)
  1.02 s  Wave  FAILED (unavailable: arm)  (0 s)
```

### Use the scheduler without a tree

The tree is only one client of the scheduler. A test, a GUI button or an RL
policy can use it directly:

```cpp
robotik::SkillScheduler& skills = simulation.skills();
robotik::SkillId const home = skills.find("Home");
skills.request(home);
while (skills.state(home) != robotik::SkillState::Succeeded)
{
    simulation.step(Seconds(0.01));          // the simulation updates the scheduler
}
```

This is what the **Home** menu of HelloWorld does.

---

## 5. Behavior trees

A behavior tree is the **mission logic**. Robotik uses
[BlackThorn](https://github.com/Lecrapouille/BlackThorn). Trees are YAML
files, named in the scenario:

```yaml
scenario:
  controller:
    behavior_tree: skills.bt.yml
```

Each tick, the tree is evaluated from the root. Each node returns
`RUNNING`, `SUCCESS` or `FAILURE`.

### An `Action` is a skill

```yaml
- Action:
    name: Wave
```

A tick of this node **requests** the skill `Wave` and returns:

| Skill state | Node returns |
|-------------|--------------|
| `Waiting` or `Running` | `RUNNING` |
| `Succeeded` | `SUCCESS` |
| `Failed`, `Cancelled`, preempted | `FAILURE` |

When the tree leaves the node while it is running (for example a parent
gives up), the run is cancelled.

### The main nodes

| Node | Kind | Behaviour |
|------|------|-----------|
| `Sequence` | composite | Runs the children in order. Fails as soon as one fails. Succeeds when all succeed |
| `Selector` | composite | Tries the children in order. Succeeds as soon as one succeeds. Fails when all fail (a "fallback") |
| `Parallel` | composite | Ticks all children. Keys `success_threshold`, `failure_threshold` |
| `Inverter` | decorator | `SUCCESS` ↔ `FAILURE` |
| `ForceSuccess` / `ForceFailure` | decorator | Always succeeds / fails when the child ends |
| `UntilSuccess` | decorator | Runs the child again until it succeeds (`attempts`) |
| `UntilFailure` | decorator | Runs the child again until it fails (`attempts`) |
| `Repeater` | decorator | Runs the child `times` times |
| `Action` | leaf | A skill, by name |

Composites take `children:`, decorators take `child:` (a list with one node):

```yaml
BehaviorTree:
  Sequence:
    name: PickAndPlace
    children:
      - Action:
          name: Home
      - UntilSuccess:
          attempts: 1000
          child:
            - Action:
                name: Detect(red_cube)
      - Selector:
          children:
            - Action:
                name: Grasp(red_cube)
            - Action:
                name: Home          # grasp failed: go back home (and the Sequence fails)
```

> **Warning.** BlackThorn's `Timeout`, `Delay`, `Wait` and `Cooldown` count
> **real (wall-clock) time**, not simulated time. Headless runs faster than
> real time, so these nodes do not give reproducible results. Put durations in
> your skills (they receive `dt`) instead.

---

## 6. Tutorial: recover from a fault

Start from HelloWorld lesson 4. In `plugins/HelloWorld/scenarios/skills.yml`,
add a fault that takes the arm away between 1 s and 2 s:

```yaml
  faults:
    - { at: 1.0, resource: arm, action: disable }
    - { at: 2.0, resource: arm, action: restore }
```

**Step 1: no recovery.** Run it:

```bash
make -C plugins/HelloWorld
./build/Robotik-Headless build/plugins/HelloWorld/scenarios/skills.yml --trace
```

```text
  0 s  Home  SUCCEEDED  (0 s)
  0.01 s  Wave  FAILED (resource lost: arm)  (0.99 s)
[FAIL] controller.success  (controller.success = 0)
```

**Step 2: retry.** Change the tree to retry `Wave` until it works. This is
exactly `scenarios/skills_retry.bt.yml`:

```yaml
      - UntilSuccess:
          attempts: 1000
          child:
            - Action:
                name: Wave
```

```text
  0 s  Home  SUCCEEDED  (0 s)
  0.01 s  Wave  FAILED (resource lost: arm)  (0.99 s)
  1.02 s  Wave  FAILED (unavailable: arm)  (0 s)
  ...
  1.98 s  Wave  FAILED (unavailable: arm)  (0 s)
  2 s  Wave  SUCCEEDED  (3.77 s)
  5.78 s  Home  SUCCEEDED  (0.63 s)
[PASS] controller.success  (controller.success = 1)
```

Look at what you did **not** write: no `if (arm_broken)` in the skill, no
special code in the mission. The **fault** is a fact (the scheduler sees
it); the **reaction** is a decision (the tree makes it). Two separate jobs.

The HelloWorld experiment compares both trees on the same faulty scenario:
see [03-Worlds-and-Scenarios.md §9](03-Worlds-and-Scenarios.md#9-the-experiment-file).

---

## 7. Tutorial: write a skill that moves to a pose

Inside your own skill you can reuse the built-in ones. `MoveTCPSkill`
computes IK once per goal and drives the joints:

```cpp
class TouchSkill final: public robotik::Skill
{
public:

    explicit TouchSkill(robotik::Pose p_target) : m_target(p_target) {}

    void reset() override
    {
        m_move.reset();
        m_move.goal(m_target);           // in the robot base frame
    }

    robotik::Status tick(robotik::RobotContext& p_context, Seconds p_dt) override
    {
        return m_move.tick(p_context, p_dt);   // RUNNING until reached, or FAILURE
    }

    void cancel(robotik::RobotContext& p_context) override
    {
        m_move.cancel(p_context);
    }

private:

    robotik::Pose m_target{};
    robotik::MoveTCPSkill m_move;
};
```

To target an object, read it from the **belief**, not from the simulation:

```cpp
robotik::WorldObject const* cube = p_context.world_model.find("red_cube");
if (cube == nullptr || !cube->observed())
{
    return robotik::Status::FAILURE;     // the robot has not seen it
}
// cube->position, cube->size, cube->confidence ...
```

See `plugins/Painter/src/PainterSkills.cpp` (`DipSkill`, `MarkSkill`,
`RinseSkill`) for complete skills built on `MoveTCPSkill` and trajectories.

---

## 8. Custom tree nodes

By default, actions are named after the skills. A plugin can also register
its own BlackThorn nodes, for example to give arguments to a skill before
requesting it. The Painter does this (`painter::registerNodes`):

```cpp
p_factory.registerNode("Dip", [&p_scheduler]() {
    return std::make_unique<GestureNode>(p_scheduler, p_scheduler.find("dip"), Gesture::Dip);
});
```

Call it from `Mission::setup` with `p_simulation.factory()`.

---

## 9. Going further: planners, logic and LLMs

The split *tree / scheduler / skill* leaves room for higher layers. None of
them is implemented yet; the architecture is ready for them.

```text
"Put the red cube in the box"            ← an intent (LLM, user)
            │
            ▼
goal: inside(red_cube, box)              ← a structured goal
            │
            ▼
GOAP / PDDL planner                      ← "which actions are needed?"
            │ plan: Detect, Approach, Reach, Grasp, MoveTo(box), Release
            ▼
Behavior tree                            ← "how to run them and react?"
            │
            ▼
Skills + scheduler                       ← "can I run it now? how?"
            │
            ▼
Robot API
```

- **GOAP / PDDL** would build the sequence of skills from a goal and from the
  skill preconditions.
- **Prolog** could answer questions like `reachable(arm, red_cube)` to check
  preconditions or enrich the WorldModel.
- **An LLM** would stay at the level of intent: it never drives the motors.

Why keep the tree when a planner exists? A plan is ideal; the world is not.
`MoveTo(box)` can fail because something blocks the path, the grasp slipped,
or a sensor failed. The tree reacts: retry, fall back, re-plan, abort.

A skill is also a **slot**, not one implementation: `Reach(target)` can be
done with IK, with a motion planner, or with a learned policy. The scenario
and the tree do not change.
