# Three programs, one per layer

[⏫ Tutorials](README.md)

RobotIK has three layers. Each one works without the ones above it, so you
pick the smallest one that fits your project. The words are explained in
the [glossary](../reference/glossary.md).

| Layer | You have… | You write… |
|-------|-----------|------------|
| **Robot** | a URDF file | one robot, its joints and its physics |
| **Task** | robots, objects and a job | a robot cell in C++, with skills and a controller |
| **Package** | YAML files and a plugin | a scenario file that builds the cell and judges the run |

Three small programs show them. Each one is a test, so it always builds and
runs. They share one engine: a skill runs through the same scheduler whether
you request it, a behavior tree asks for it, or a plan needs it.

**1. Move a robot, without a planner** (Robot layer,
`tests/Robot/TestOpenRobot.cpp`).

```cpp
auto arm = robotik::openRobot("robotik.robots.robot_6axis");
if (!arm)
    return fail(arm.error());                  // a message telling what to fix
robotik::JointSet& joints = arm.value()->joints();
joints.moveTo(joints.revolute("joint2"), robotik::Radians(0.3));
for (int i = 0; i < 200; ++i)
    arm.value()->step(robotik::Seconds(0.01)); // MuJoCo moves the joint
```

**2. Run a skill, without a mission** (Task layer,
`tests/Examples/TestLayerExamples.cpp`). A robot and a cube described in
C++; `openRoboticCell` creates the world that holds them.

```cpp
robotik::EntitySpec arm;
arm.id = "arm";
arm.type = robotik::EntitySpec::Type::Robot;
arm.model = "robot_6axis.urdf";
robotik::EntitySpec cube;
cube.id = "cube";
cube.shape = robotik::ShapeSpec{};             // a 4 cm box
cube.pose.position = robotik::Vector3(0.4, 0.2, 0.02);
robotik::RoboticCellDescription description;
description.world.entities = { arm, cube };

auto opened = robotik::openRoboticCell(std::move(description));
robotik::RoboticCell& cell = *opened.value();
auto const raise = cell.skills().add<robotik::MoveJointSkill>(
    { .name = "Raise" }, "joint2", 0.5);
cell.run(raise, robotik::Seconds(10.0));       // SkillState::Succeeded
```

**3. Reach a goal, without writing a sequence of skills** (Task layer, same
test file). You give the goal; the planner finds the actions in the PDDL
domain, and the cell runs their skills. The mission gives the skills and the
observers that read the beliefs ([The PDDL planner](../concepts/pddl-planner.md)).

```cpp
PickPlaceMission mission(true, true);          // skills + observers
auto opened = robotik::openRoboticCell(description, &mission);
robotik::RoboticCell& cell = *opened.value();

robotik::PlannerSpec goal;
goal.domain = "pick_and_place.domain.pddl";
goal.goal = { robotik::parseLiteral("(in red_cube box)").take() };
cell.achieve(goal);
cell.run(robotik::Seconds(60.0));              // Status::SUCCESS: detect, pick, place
```

The `planner:` block of a scenario file calls the same `achieve`. The YAML
of the Package layer only reads files and calls the Task layer; it adds no
behaviour of its own.

**Package**: the scenario file builds the cell; the simulation judges the
run with the assertions of the file (same test file).

```cpp
auto scenario = robotik::loadScenario("hello.scenario.yaml", assets);
if (!scenario)
    return fail(scenario.error());
compages::world::World world;
robotik::Simulation simulation(world, scenario.take(), nullptr, &mission);
while (!simulation.finished())
    simulation.step(robotik::Seconds(0.01));
bool const ok = simulation.passed();
```

A plugin is the same Package layer, loaded by the simulator or by
Robotik-Headless: [Your first plugin](04-your-first-plugin.md).

## Where to go next

- Change the world and the scenario, without C++:
  [A cube in a box](03-a-cube-in-a-box.md).
- Write your own C++ code: [Your first plugin](04-your-first-plugin.md).
