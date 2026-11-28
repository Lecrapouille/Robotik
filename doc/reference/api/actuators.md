# 🦾 Actuators

[⏫ API reference](README.md)

Header: `RobotIK/Robot/Actuators.hpp`. Declared in the world file; each one is also a resource.

```cpp
auto* group = p_cell.robot().getActuators().first<robotik::JointGroup>();
```

| Class | YAML `type` | Main methods |
|-------|-------------|--------------|
| `JointGroup` | `joint_group` | `joints()`, `contains(id)`, `moveTo(span<double>)`, `hold()`, `reached(tol)` |
| `Motor` | `motor` | `moveTo(Radians)`, `spin(AngularVelocity)`, `push(Torque)`, `stop()`, `position()` |
| `VacuumGripper` | `vacuum` | grasp / release what touches the suction face (see `demos/PickAndPlace`) |

A program adds one with `RobotRuntime::addActuator<T>(name, …)`; a skill only finds and commands them.
