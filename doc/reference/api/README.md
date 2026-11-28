# 📘 API reference

[⏫ Reference](../README.md)

The classes a plugin author uses, one page per group. For each class: the header, what it is for, how it fits with the others (diagrams), and only the methods you use most. Every public method, with its contract, is in the Doxygen pages: run `make doc`, then open `doc/doxygen/html/index.html`. The comments of the headers are the same text.

Conventions in the code:

- namespace `robotik`; units from `RobotIK/Common/Math.hpp` (`Seconds`, `Radians`, …), the same types as Compages (`compages::si`);
- headers are in three layers (see [Architecture](../../concepts/architecture/README.md) and the [glossary](../glossary.md)): `RobotIK/Robot/` (one robot), `RobotIK/Task/` (a job in C++), `RobotIK/Package/` (files and plugins), plus `RobotIK/Common/`;
- reading a file returns a `Result` (`if (!r) … r.error()`, then `r.value()` or `r.take()`); a mistake in the code throws;
- parameters start with `p_`, members with `m_`;
- SI units everywhere: metres, radians, seconds, newtons;
- z up, like URDF.

| I want to… | Use | Page |
|------------|-----|------|
| Write the entry point of my plugin | `Plugin`, `Host`, `Canvas` | [Plugin API](plugin.md) |
| Add behaviour to a simulation | `ScenarioLogic` | [`ScenarioLogic`](scenario-logic.md) |
| Read the world, the time, the truth | `RoboticCell` (and `Simulation` for the verdict) | [`RoboticCell`, `Simulation`, `StandaloneCell`, `StandaloneSimulation`](robotic-cell.md) |
| Run a scenario of a plugin without a window, in a few lines | `StandaloneSimulation::openPlugin` | [`StandaloneSimulation`](robotic-cell.md#-standalonesimulation) |
| Know which robot class to use: what a skill sees, what runs the robot, a robot alone | `Robot`, `RobotRuntime`, `StandaloneRobot` | [The robot](robot.md#-three-classes-one-robot) |
| Read the joints and sensors, compute the kinematics, from a skill | `Robot` | [The robot](robot.md#-robot-the-methods-you-use-most) |
| Step a robot, plug a backend, read the truth, write a driver | `RobotRuntime`, `RobotBackend`, `PhysicsWorld` | [The robot](robot.md#-robotruntime-the-methods-you-use-most) |
| Get one robot in one call, without a cell | `StandaloneRobot::open` | [The robot](robot.md#-get-a-robot-in-one-call) |
| Command joints, give a posture | `JointSet`, `JointPosture` | [Joints](joints.md) |
| Jog the robot, save and replay positions | `TeachPendant` | [`TeachPendant`](teach-pendant.md) |
| Use the gripper, a motor | actuators | [Actuators](actuators.md) |
| Write a skill, run, cancel, inspect skills | `Skill`, `RobotContext`, `SkillScheduler` | [Skills](skills.md) |
| Reserve parts of the robot, inject faults | `ResourceManager`, `FaultInjector` | [Resources and faults](resources.md) |
| Process images, update beliefs | `Detector`, `PerceptionPipeline`, `WorldModel` | [Perception and beliefs](perception.md) |
| Publish numbers for assertions | `Metrics` | [Metrics](metrics.md) |
| Find a 3D model | `AssetResolver` | [Assets](assets.md) |
| Make an RL environment | `Environment`, `SimulationEnvironment` | [Learning](learning.md) |
| Load a scenario, an experiment, a trajectory in C++ | `loadScenario`, `Experiment` | [Scenario objects](scenario-objects.md) |

One umbrella header includes everything: `#include "RobotIK/RobotIK.hpp"`.
