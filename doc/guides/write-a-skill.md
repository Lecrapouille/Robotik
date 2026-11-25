# 🔧 Write a skill that moves to an object

[⏫ Guides](README.md)

A real mission moves the tool to an object: above a cube to grasp it, in front of a part to drill it. The robot does not know where the object is exactly. It knows where it believes the object is, from its camera. In this guide you write a skill `Touch` that reads this belief, and moves the tool 10 cm above the cube. It reuses the built-in skill `MoveTCPSkill`, which computes the joint angles (inverse kinematics) and drives the joints.

You need to know how a skill works and how a tree runs it ([A skill and a behavior tree](../tutorials/06-skills-and-a-behavior-tree.md)).

## 📖 The words

- A belief is what the robot thinks it knows about an object: its position, its size, the time when it saw it. The beliefs are in `world_model`, in the `RobotContext` that a skill receives. A skill never reads the true position: a real robot does not know it.
- `observed(object)` is true when the robot saw the object at least once. Before that, the belief is only a guess (a prior).
- `MoveTCPSkill` moves the work frame of the robot (`tcp` with a tool, `tool0` without) to a pose in the frame of the robot base.

## 🧠 1. Write the skill

```cpp
/// Moves the tool 10 cm above an object, at the position the robot believes.
class TouchSkill final: public robotik::Skill
{
public:

    explicit TouchSkill(std::string p_object)
        : m_object(std::move(p_object))
    {
    }

    void reset() override
    {
        m_move.reset();
        m_has_goal = false;
        m_why.clear();
    }

    robotik::Status tick(robotik::RobotContext& p_context,
                         robotik::Seconds p_dt) override
    {
        if (!m_has_goal)
        {
            // The belief, not the truth: where the robot thinks the object is.
            robotik::WorldObject const* object =
                p_context.world_model.find(m_object);
            if (object == nullptr || !observed(*object))
            {
                m_why = m_object + " not seen";
                return robotik::Status::FAILURE;
            }
            // Keep the orientation of the tool; go 10 cm above the object.
            robotik::Robot& robot = p_context.robot;
            robotik::Pose target = robot.requireFramePose(robot.tool());
            target.position = object->position + robotik::Vector3(0.0, 0.0, 0.10);
            m_move.goal(target);
            m_has_goal = true;
        }
        return m_move.tick(p_context, p_dt);
    }

    void cancel(robotik::RobotContext& p_context) override
    {
        m_move.cancel(p_context);
    }

    [[nodiscard]] std::string why() const override
    {
        return m_why;
    }

private:

    std::string m_object;
    robotik::MoveTCPSkill m_move;
    bool m_has_goal = false;
    std::string m_why;
};
```

The first `tick` reads the belief and gives a goal to `m_move`. The next ticks only call `m_move.tick`, which returns `RUNNING` until the tool arrives, then `SUCCESS`. It returns `FAILURE` if no joint angles reach the pose. `reset` and `cancel` pass the call to `m_move`, so that the inner skill starts again and stops with the outer one.

With one robot, the beliefs are in the frame of the robot base, like the goal of `MoveTCPSkill`. With several robots, the beliefs are in the world frame ([Several robots](../reference/world-file.md#-several-robots)).

`why` explains the last failure in words. The simulator shows it in the **Why** column of the execution graph, and the planner writes it when it plans again. The `--trace` of RobotIK-Headless gives only `FAILED (failed)`.

The includes are `RobotIK/Task/Skills/MotionSkills.hpp`, `RobotIK/Task/Perception/WorldModel.hpp` and `RobotIK/Task/RobotContext.hpp`.

## 📝 2. Register it with its resource

In the `setup` of your scenario logic, add the skill with the resource that it moves, and the name of the object:

```cpp
void setup(robotik::RoboticCell& p_cell, robotik::SceneView* /*p_view*/) override
{
    p_cell.skills().add<TouchSkill>(
        { .name = "Touch",
          .resources = { p_cell.robot().resources().require("arm") } },
        "red_cube");
}
```

## 🌳 3. Call it from a tree, and give the robot a belief

The tree goes home, then touches:

```yaml
BehaviorTree:
  Sequence:
    name: TouchTheCube
    children:
      - Action:
          name: Home
      - Action:
          name: Touch
```

The world has the arm of HelloWorld and a red cube at (0.40, 0.20, 0.02). The scenario runs the tree, and checks that the tool ends near the cube:

```yaml
scenario:
  id: touch
  description: Move the tool 10 cm above the cube.
  world: worlds/cell.world.yaml
  seed: 1
  controller:
    behavior_tree: scenarios/touch.bt.yaml
  perception: { fidelity: truth }
  timeout: 20
  assertions:
    - controller.success
    - (skill_succeeded Touch)
    - (< (distance robot.tool0 red_cube) 0.12)
```

The world has no camera, so nothing fills the beliefs. The line `perception: { fidelity: truth }` copies the true position of each object into the beliefs: it replaces perception, and the run report says it (`perception truth`). With a camera and a detector, remove this line.

## ✅ 4. Run it

```text
  0 s  Home  SUCCEEDED  (0 s)
  0.01 s  Touch  SUCCEEDED  (1.13 s)
[PASS] controller.success  (controller.success = 1)
[PASS] (skill_succeeded Touch)  ((skill_succeeded Touch) = 1)
[PASS] (< (distance robot.tool0 red_cube) 0.12)  ((distance robot.tool0 red_cube) = 0.0955508)
[PASS] default                  seed=1      controller succeeded at 1.16 s (perception truth)  time=1.16
touch: 1/1 runs passed
```

The tool ends 9.6 cm from the centre of the cube: 10 cm, within the tolerance of `MoveTCPSkill`.

Without the `perception` line, the robot has no belief about the cube, and the skill fails at once:

```text
  0 s  Home  SUCCEEDED  (0 s)
  0.01 s  Touch  FAILED (failed)  (0 s)
[FAIL] controller.success  (controller.success = 0)
```

This is the correct behaviour: a skill that does not know where the object is must not move toward a guess.

## 🔗 See also

- `demos/Painter/src/PainterSkills.cpp` (`DipSkill`, `MarkSkill`, `RinseSkill`): skills built on `MoveTCPSkill` and trajectories.
- `demos/PickAndPlace/src/PickPlaceSkills.cpp`: the skills `Approach`, `Reach`, `Grasp` and `Release`, with a real camera.
- [Truth, description and belief](../concepts/02-worlds-scenarios-experiments.md#-truth-description-and-belief): the three sources of a position, and the levels of `perception`.
