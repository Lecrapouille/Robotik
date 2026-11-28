# 🕹 `TeachPendant`

[⏫ API reference](README.md)

Header: `RobotIK/Robot/TeachPendant.hpp`. A virtual teach pendant: move the robot by small steps (jog), save positions (waypoints), replay them with smooth moves. The words are explained in [The teach pendant](../../concepts/07-teach-pendant.md). The pendant writes joint targets only; the arm moves when the runtime steps.

```cpp
robotik::TeachPendant pendant;
pendant.jogTool(robot, { 0.0_m, 0.0_m, 0.01_m }, robotik::Vector3rad::ZERO);
pendant.record(robot, "above", 2.0_s);
pendant.play(robot, true);
while (pendant.isPlaying())
{
    pendant.update(robot, dt);   // next targets of the move
    runtime.step(dt);            // the physics moves the arm
}
```

| Method | Does |
|--------|------|
| `jogJoint(robot, Revolute, Radians)`, `jogJoint(robot, Prismatic, Length)` | One joint by a small step, inside its limits; stops a replay |
| `jogTool(robot, translation, rotation)` | The chosen link by a small step along the axes of the base (inverse kinematics); stops a replay |
| `record(robot, label, duration)` | Saves the measured joints and their names; returns the index |
| `goTo(robot, index)`, `play(robot, loop)` | Starts a move to one waypoint, or to all of them in order |
| `update(robot, dt)` | Gives the motors the targets of this step; call it at each step while `isPlaying()` |
| `stop(robot)` | Stops a replay where the arm is |
| `erase(index)`, `clear()` | Removes waypoints (stops a replay) |
| `setFrame(name)`, `getFrame(robot)` | The link that `jogTool` moves; empty is the tool |
| `getJogAxes(robot)`, `getJogGoal(robot, rotation)` | Which of the six directions can move now; what the IK must keep |
| `getWaypoints()` | The saved positions (`Waypoint { label, joints, joint_names, pose, duration }`) |
| `isPlaying()`, `getTargetIndex()`, `getError()` | State of the replay, and why the last command failed |

A command that cannot be done returns `false`, keeps the targets and says why in `getError()`. A waypoint saved on a robot with other joints (other names, or the same count with other names) is refused by `goTo` and `play`. Pause the mission of the scenario before you jog: it writes its own targets at each step.
