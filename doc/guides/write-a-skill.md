# Write a skill that moves to a pose

[⏫ Guides](README.md)

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
