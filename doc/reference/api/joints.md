# 🧮 Joints: `JointSet` and `JointPosture`

[⏫ API reference](README.md)

Header: `RobotIK/Robot/Joints.hpp`. The joints of one robot, one array per quantity, indexed by `JointId`. A skill gets them with `robot.getJoints()`.

Two ways to name a joint:

```cpp
robotik::JointId const id = joints.find("joint1");              // index, doubles; NO_JOINT if absent
robotik::Revolute const j = joints.requireRevolute("joint1");  // typed handle, Radians; throws if absent
robotik::Prismatic const k = joints.requirePrismatic("rail");  // typed handle, Length
```

| Method | Does |
|--------|------|
| `size()` | Number of joints; ids are `0 … size()-1` |
| `find(name)` / `require(name)` | Id, or `NO_JOINT` / throws |
| `requireRevolute(name or id)`, `requirePrismatic(name or id)` | Typed handle; throws if absent or of the other kind |
| `type(id)`, `isPrismatic(id)` | Kind of joint (`JointType`: `Revolute`, `Continuous`, `Prismatic`) |
| `limits(handle)` | `lower`, `upper`, `velocity`, `effort`; `bounded(limits)` |
| `position(j)`, `velocity(j)`, `effort(j)` | Measured state |
| `home(j)` | Home position (written by `RobotRuntime::hold`) |
| `moveTo(j, position)` | **Position command** (a target; the controller moves toward it) |
| `spin(j, velocity)` | Velocity command |
| `push(j, effort)` | Effort (torque or force) command |
| `reached(j, tolerance)` | In position mode and close to the target |
| `hold(j)`, `hold()` | Stay where you are |
| `release(j)` | Stop commanding (no effort) |
| `target(id)`, `mode(id)` | Current command |

A skill writes commands only. The measures are written by the runtime (`RobotRuntime::measureJoint`).

## 🧷 `JointPosture`

A position of the robot, joint by joint: a home, the targets of a move. Two tables, because the unit depends on the kind of the joint:

```cpp
robotik::JointPosture home;
home.angles["joint2"] = 0.3_rad;    // a joint that turns
home.lengths["finger"] = 0.02_m;    // a joint that slides
```

A value of `angles` is always `Radians`, a value of `lengths` always `Length`. That a name is in the right table is checked when the robot is known: `RobotRuntime::hold` throws, `MoveJointsSkill` fails with `wrong_unit`, and the world file reader classes each number of `home:` by the kind of its joint in the URDF (`readJointTypes(urdf)`, a `Result`). `isEmpty(posture)` tells if it names no joint.
