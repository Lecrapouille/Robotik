# 🔒 Resources and faults

[⏫ API reference](README.md)

Headers: `RobotIK/Robot/Resources.hpp`, `RobotIK/Task/Faults.hpp`.

| `ResourceManager` | Does |
|-------------------|------|
| `require(name, Access)` | A `ResourceRequirement` for a `SkillDescription`. `Access::Exclusive` (default) or `Access::Shared` |
| `find(name)`, `name(id)`, `size()` | Lookup |
| `available(name or id)` | False while in fault |
| `fail(name)`, `restore(name)`, `restoreAll()` | Inject / clear a fault by hand |

`FaultInjector` applies the `faults:` of the scenario (`at`, `rate`) at each step. You rarely need it directly.

A backend that is not healthy (`RobotBackend::getHealth`) fails the resources of the actuators of its robot until it is healthy again (see [`RobotRuntime`](robot.md#-robotruntime-the-methods-you-use-most)).
