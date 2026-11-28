# 🔧 Skills and the `SkillScheduler`

[⏫ API reference](README.md)

Headers: `RobotIK/Task/Skills/Skill.hpp`, `RobotIK/Task/Skills/MotionSkills.hpp`, `RobotIK/Task/RobotContext.hpp`.

```cpp
class Skill
{
    virtual void reset();
    virtual Status tick(RobotContext& p_context, Seconds p_dt) = 0;
    virtual void cancel(RobotContext& p_context);
    SkillFailure const& failure() const;   // why the current run failed: { code, why }; empty until it fails
protected:
    Status fail(std::string code, std::string why);  // records the failure, returns FAILURE
    Status fail(SkillFailure child);                 // passes on the failure of a child skill
};

struct SkillFailure { std::string code; std::string why; };  // code for the tree, why for a person

struct RobotContext { Robot& robot; WorldModel& world_model; Seconds time; Seconds dt;
                      std::span<Robot const* const> team; };   // team: every robot, read only; empty when alone
```

`Status`: `IDLE`, `RUNNING`, `SUCCESS`, `FAILURE`.

`SkillDescription`: `name`, `resources`, `priority`, `wait`, `cancellable`, `urgent`, `stop_timeout`, `preconditions` (`{ text, std::function<bool(RobotContext const&)> }`), `robot` (id of the robot it drives; empty: the first one).

Built-in: `HomeSkill(group)`, `MoveJointSkill(joint, target)`, `MoveJointsSkill(posture)` (a [`JointPosture`](joints.md#-jointposture)), `MoveTCPSkill(frame, target)` + `goal(pose)`, `StopSkill`. See [Skills, the scheduler and behavior trees](../../concepts/04-skills-and-behavior-trees.md).

## 📅 `SkillScheduler`

Header: `RobotIK/Task/Scheduler.hpp`.

| Method | Does |
|--------|------|
| `add<T>(description, args...)` | Registers a skill; returns its `SkillId`. Throws on a duplicate name |
| `find(name)` | `SkillId`, or `NO_SKILL` |
| `request(id)` | Ask for a run (no effect if already waiting or running) |
| `cancel(id)`, `cancelAll()` | Cancel at the next update |
| `update(context)` | Admit, preempt, tick (the simulation calls it) |
| `state(id)`, `reason(id)`, `blocker(id)` | `SkillState`, `SkillReason`, the resource behind it |
| `precondition(id)` | The false precondition, if blocked by one |
| `name(id)`, `description(id)`, `skill(id)` | Introspection |
| `trace()` | Every run since the last reset (`SkillRun`: state, reason, start, end) |
