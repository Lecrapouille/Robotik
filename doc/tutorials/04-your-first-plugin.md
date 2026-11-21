# 🔌 Your first plugin

[⏫ Tutorials](README.md)

A **plugin** is your program. It is a folder in `plugins/` with some YAML
files (world, scenarios) and a little C++ compiled into a shared library
(`.so`). The simulator and Headless load it when you choose one of its
scenarios.

This tutorial follows the **HelloWorld** plugin, lesson by lesson. The next
tutorial, [Your own plugin](05-your-own-plugin.md), creates a plugin from
zero. The plugin class, its lifecycle and `plugin.yaml` are listed in the
[plugin reference](../reference/plugin.md).

| Lesson | Scenario | You learn |
|--------|----------|-----------|
| 1 | `hello.scenario.yaml` | The smallest mission: count steps, publish a metric, end the run |
| 2 | `wave.scenario.yaml` | Command a joint, read the tool pose (forward kinematics) |
| 3 | `reach.scenario.yaml` | Reach a point with inverse kinematics |
| 4 | `skills.scenario.yaml` | Write a skill, call it from a behavior tree |
| — | all | The plugin class: menus, panels, keyboard |

Run them first, to know what you are reading:

```bash
./build/linux-gcc-debug/bin/Robotik-Simulator          # Demos → Hello world → ...
./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/HelloWorld/scenarios/hello.scenario.yaml
```

## 📦 What is in a plugin?

```text
plugins/HelloWorld/
├── plugin.yaml          the manifest: name, library, scenarios
├── Makefile             builds librobotik_hello_world.so
├── src/                 the C++ code
│   ├── Plugin.cpp           the plugin class: chooses a mission per scenario
│   ├── HelloMission.*       lesson 1
│   ├── WaveMission.*        lesson 2
│   ├── ReachMission.*       lesson 3
│   └── WaveSkill.*          lesson 4
├── worlds/arm.world.yaml      what exists: one 6-axis arm
├── scenarios/           what happens and what is checked
└── experiments/         variants to compare
```

**YAML** says *what*: the world, the task, the checks. **C++** says *how*:
the behaviours the YAML cannot express.

Who calls what:

```text
 Robotik-Simulator / Robotik-Headless   (the "host")
        │ 1. reads plugin.yaml (no code loaded yet)
        │ 2. loads librobotik_hello_world.so when you pick a scenario
        ▼
 HelloWorldPlugin::setup("wave")   ← your Plugin class
        │ creates a WaveMission, gives it to the host
        ▼
 Simulation (built from worlds/arm.world.yaml + scenarios/wave.scenario.yaml)
        │ at each step:
        ▼
 WaveMission::act()                ← your Mission class
```

So there are **two classes** to know:

| Class | Role | Talks to |
|-------|------|----------|
| `robotik::Plugin` | The entry point. Chooses the mission, adds menus and panels | The host (window, keyboard) |
| `robotik::Mission` | The behaviour during the simulation: skills, metrics, end | The simulation (robot, world) |

## 🐣 Lesson 1 — the smallest mission

`scenarios/hello.scenario.yaml`:

```yaml
scenario:
  id: hello
  description: Count the simulation steps, then stop after two seconds.
  world: worlds/arm.world.yaml      # relative to the package root
  seed: 1
  timeout: 30
  assertions:
    - controller.success      # the mission said SUCCESS
    - (> hello.ticks 100)       # a metric published by the mission
    - (< time 3)
```

`src/HelloMission.hpp`:

```cpp
class HelloMission final: public robotik::Mission
{
public:

    void reset(robotik::RoboticCell& p_cell, robotik::Seed p_seed) override;
    void act(robotik::RobotContext& p_context, robotik::Seconds p_dt) override;
    void measure(robotik::RoboticCell const& p_cell,
                 robotik::Metrics& p_metrics) const override;
    [[nodiscard]] robotik::Status
    status(robotik::RobotContext const& p_context) const override;

private:

    double m_ticks = 0.0;
};
```

`src/HelloMission.cpp`:

```cpp
void HelloMission::reset(robotik::RoboticCell& /*p_cell*/, robotik::Seed /*p_seed*/)
{
    m_ticks = 0.0;                         // a new episode starts
}

void HelloMission::act(robotik::RobotContext& /*p_context*/,
                       robotik::Seconds /*p_dt*/)
{
    m_ticks += 1.0;                        // called after each simulation step
}

void HelloMission::measure(robotik::RoboticCell const& /*p_cell*/,
                           robotik::Metrics& p_metrics) const
{
    p_metrics.set("hello.ticks", m_ticks); // the scenario can now use "hello.ticks"
}

robotik::Status
HelloMission::status(robotik::RobotContext const& p_context) const
{
    if (p_context.time >= robotik::Seconds(2.0))
    {
        return robotik::Status::SUCCESS;   // ends the run: controller.success = 1
    }
    return robotik::Status::RUNNING;
}
```

The methods of a `Mission`, all optional:

| Method | When | Use it to |
|--------|------|-----------|
| `setup(cell, view)` | Once, after the world is built | Add skills, detectors, decorations. `view` is null in headless |
| `reset(cell, seed)` | At each episode start | Clear your state. Derive random numbers from `seed` |
| `act(context, dt)` | After each simulation step | Command the robot. You see the robot and its beliefs, as a skill does |
| `simulate(cell, dt)` | Then, in simulation only | Make the world do something by itself (a conveyor, ink on a canvas) |
| `observe(cell, dt)` | Then | Read the truth for your metrics. The cell is `const`: no command |
| `measure(cell, metrics)` | When the assertions need values | Publish named numbers |
| `status(context)` | After each step, **only if the scenario has no behavior tree** | Return `SUCCESS` or `FAILURE` to end the run |

The split keeps the robot honest. `act` gets the same view as a skill, so
it cannot read where the cube really is, and the same code runs on a real
robot. `observe` can read the truth, because a metric must be exact, but
it cannot move anything.

> **Rule:** the plugin does not judge itself. It publishes metrics; the
> scenario file decides what is a success. Do not write `assert` or
> `is_finished()` in C++: write an assertion in YAML.

## 🦿 Lesson 2 — move a joint (forward kinematics)

The arm turns its base joint left and right. `src/WaveMission.cpp`:

```cpp
void WaveMission::act(robotik::RobotContext& p_context,
                      robotik::Seconds /*p_dt*/)
{
    robotik::Robot& robot = p_context.robot;
    robotik::JointSet& joints = robot.joints();

    // A typed handle: the angle is in radians, the compiler checks it.
    robotik::Revolute const base = joints.revolute("joint1");
    if (!base)
    {
        return;
    }

    // Command: a target angle. The joint controller moves toward it.
    Radians const home = joints.home(base);
    Radians const offset(0.6 * std::sin(p_context.time.value()));
    joints.moveTo(base, home + offset);

    // Measure: the angle reached, and where the tool is (forward kinematics,
    // in the robot base frame).
    m_swing = std::max(m_swing, std::abs((joints.position(base) - home).value()));
    m_tool_y = robot.framePose(robot.tool()).position.y;
}
```

What you see here:

- `p_context.robot` gives the robot. `robot.joints()` gives all joints.
- `joints.revolute("joint1")` gives a **typed handle**: its positions are
  `Radians`. A prismatic joint gives a `Prismatic` handle in metres. Mixing
  them does not compile.
- `moveTo` gives a **target**; the joint does not jump there. The physics
  (MuJoCo) moves it at a realistic speed. This is why we measure the
  *reached* angle with `joints.position(base)`, not the commanded one.
- `robot.tool()` is the name of the working frame: `tcp` when a tool is
  mounted, `tool0` otherwise. `robot.framePose(frame)` is **forward
  kinematics**: the pose of that frame in the robot base frame.

The scenario checks the result and the safety:

```yaml
  assertions:
    - controller.success
    - (> wave.swing 0.4)
    - { condition: (joint_limits_respected robot), phase: always }
```

**Try it:** change `0.6` into `0.2` in the code, rebuild
(`make robotik_hello_world`), run again. `(> wave.swing 0.4)` now fails, and
the report tells you the joint swung only 0.2 rad.

## 🎯 Lesson 3 — reach a point (inverse kinematics)

Now we give a **position** and ask which joint angles reach it.
`src/ReachMission.cpp`:

```cpp
void ReachMission::reset(robotik::RoboticCell& p_cell, robotik::Seed /*p_seed*/)
{
    robotik::Robot& robot = p_cell.robot();

    // The tool frame is "tcp" when a tool is mounted, "tool0" otherwise.
    robotik::Pose const start = robot.framePose(robot.tool());
    m_target = start;
    m_target.position.x += 0.03;                      // 3 cm forward

    // Inverse kinematics: no answer when the pose is out of reach.
    std::optional<std::vector<double>> const solution =
        robot.inverseKinematics(robot.tool(), m_target);
    m_solution = solution ? *solution : std::vector<double>{};
    m_error = distance(start, m_target);
}

void ReachMission::act(robotik::RobotContext& p_context,
                       robotik::Seconds /*p_dt*/)
{
    robotik::Robot& robot = p_context.robot;
    robotik::JointSet& joints = robot.joints();
    for (robotik::JointId id = 0; id < joints.size() && id < m_solution.size(); ++id)
    {
        joints.moveTo(id, m_solution[id]);            // one target per joint
    }
    m_error = distance(robot.framePose(robot.tool()), m_target);
}
```

- `robot.inverseKinematics(frame, pose)` returns one angle per joint, or
  nothing (`std::nullopt`) if the pose cannot be reached.
- Joints also have plain ids (`JointId`, `0 … size()-1`) with `double`
  values. Use them for loops; use typed handles when you name one joint.

The scenario uses a **deadline** assertion:

```yaml
    - { condition: (< reach.error 0.02), within: 5.0 }
```

In the simulator, this assertion is red at the start (the error is 3 cm), then
turns green when the tool arrives. A red assertion does not stop the run.

**Try it:** ask for 3 metres instead of 3 cm (`+= 3.0`). IK has no answer,
`status()` returns `FAILURE`, and the run ends at once:

```text
[FAIL] (< reach.error 0.02)  (reach.error = 3, not true within 5.000000 s)
[FAIL] default                  seed=1      controller failed at 0.01 s  time=0.01
```

## 🌳 Lesson 4 — a skill and a behavior tree

Lessons 1–3 drive the robot directly from `Mission::act`. This is simple,
but it does not scale: who decides the order? what if the arm is busy? what
if a sensor fails? Robotik answers with **skills** and **behavior trees**.

A skill is a small class with a `tick` method. `src/WaveSkill.cpp`:

```cpp
robotik::Status WaveSkill::tick(robotik::RobotContext& p_context, Seconds /*p_dt*/)
{
    robotik::JointSet& joints = p_context.robot.joints();
    robotik::JointId const base = joints.find("joint1");
    if (base == robotik::NO_JOINT)
    {
        return robotik::Status::FAILURE;
    }

    double const target = (m_waves % 2 == 0) ? 0.5 : -0.5;
    joints.moveTo(base, target);
    if (joints.reached(base, 0.02))
    {
        ++m_waves;
    }
    return m_waves >= 4 ? robotik::Status::SUCCESS : robotik::Status::RUNNING;
}

void WaveSkill::cancel(robotik::RobotContext& p_context)
{
    p_context.robot.joints().hold();   // cancelled or preempted: stop where we are
}
```

The mission registers it, with a **name** and the **resources** it needs:

```cpp
void SkillsMission::setup(robotik::RoboticCell& p_cell, robotik::SceneView* /*p_view*/)
{
    robotik::SkillDescription wave;
    wave.name = "Wave";
    wave.resources = { p_cell.robot().resources().require("arm") };
    wave.priority = 100;
    p_cell.skills().add<WaveSkill>(std::move(wave));
}
```

Every registered skill becomes a behavior tree **Action** with the same
name. `scenarios/skills.bt.yaml`:

```yaml
BehaviorTree:
  Sequence:
    name: HelloSkills
    children:
      - Action:
          name: Home        # built-in skill
      - Action:
          name: Wave        # our skill
      - Action:
          name: Home
```

And the scenario uses it as its controller:

```yaml
  controller:
    behavior_tree: skills.bt.yaml
  assertions:
    - controller.success
    - (skill_succeeded Wave)
    - name: home before waving
      sequence: [ (skill_succeeded Home), (skill_started Wave) ]
```

```bash
./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/HelloWorld/scenarios/skills.scenario.yaml --trace
```

```text
  0 s  Home  SUCCEEDED  (0 s)
  0.01 s  Wave  SUCCEEDED  (4 s)
  4.02 s  Home  SUCCEEDED  (0.63 s)
```

Skills, resources, priorities, faults and trees are explained in
[Skills, the scheduler and behavior trees](../concepts/skills-and-behavior-trees.md).

Next: [Your own plugin](05-your-own-plugin.md).
