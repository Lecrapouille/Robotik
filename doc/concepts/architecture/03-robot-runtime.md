# 🔐 What a skill sees, what runs the robot

[⏫ Architecture](README.md)

A skill must behave the same in a test, in the simulator and on the real robot. If it could read the true position of the robot, or write the measure of a joint, it would work in simulation and fail on the hardware. RobotIK prevents this with the types: a skill gets a `Robot`, and the program that runs the robot holds a `RobotRuntime`.

## 🧩 Two classes and a model

| Class | Header | What it holds | Who has it |
|-------|--------|---------------|------------|
| `Robot` | `Robot/Robot.hpp` | measured joints and their commands, sensors (read only), actuators, kinematics on the measured joints, estimated base, resources (read only), id | a skill, through `RobotContext::robot` |
| `Robot::Model` | `Robot/Robot.hpp` | what does not change after the load: name, URDF, work frame (`tool`), joint names in the order of their `JointId` | read with `Robot::getModel()` |
| `RobotRuntime` | `Robot/RobotRuntime.hpp` | the `Robot`, plus the clock, the ECS world and the entities of the links, the backend, the `PhysicsWorld`, the true base, the start pose, the health of the backend | `RoboticCell`, `StandaloneRobot`, the backends, the sensors, the grasp models, the tests |

The runtime **owns** the `Robot` and lends it with `runtime.robot()`. It does not derive from it. An earlier version did (`RobotSession : public Robot`): the `Robot` then stored the world, the backend and the true base as `protected` members, and the derived class only added the right to reach them. The access was split, the storage was not, and nothing in the types stopped a cast from `Robot&` to the session. Now the `Robot` does not store these things at all, so there is nothing to reach.

```cpp
robotik::RobotRuntime runtime(world, urdf);       // loads the URDF, the joints, the entities
runtime.addActuator<robotik::JointGroup>("arm");  // only the runtime adds a device
runtime.connect(std::move(physics));              // the backend
runtime.step(0.01_s);                             // time passes
robotik::Robot& robot = runtime.robot();          // what a skill gets
robot.getJoints().moveTo(elbow, 0.5_rad);         // a command
```

`Robot::Model` follows the model and state split of Pinocchio. A `TeachPendant` waypoint keeps the joint names of the robot that recorded it: a replay on an arm with six joints of other names is refused, even if the count is the same.

## 👀 What a skill can do

With its `Robot&`, a skill:

- reads the joints and sends them targets (`getJoints()`);
- finds the sensors and reads them, and finds the actuators and commands them (`getSensors()`, `getActuators()`);
- computes the kinematics on the measured joints, read only (`requireFramePose`, `inverseKinematics`, `getKinematics()`);
- reads the estimated base (`getBase()`, `getWorldPose`), the resources, the name, the id and the work frame.

The time comes with each tick, in `RobotContext::time` and `dt`: the clock belongs to what runs the robot (`RobotRuntime::getTime`).

`RobotContext::team` gives the other robots of the cell as `Robot const*`: a skill reads what they measure, as a real cell shares it on its network. It cannot command them, so the scheduler sees every command of a skill.

## 🚫 What only the runtime can do

| Role | Methods | Why not in a skill |
|------|---------|--------------------|
| Make time pass | `connect`, `step`, `prepare`/`advance`/`sense`, `reset`, `hold`, the clock `getTime` | only the program decides when the physics moves |
| Write the measures | `measureJoint`, `placeJoint`, `measureHolding`, `measureBase` | a measure comes from a sensor; a skill could fake it |
| Read the truth of the simulator | `getTrueBase`, `getTrueWorldPose` | a real robot knows only its estimate |
| Localization | `estimateBase`, `setLocalization` | a plugin algorithm (or a copy of the truth) writes the estimate |
| Change a sensor | the writable `getSensors()`: image source, noise seed, forget a reading | a skill that reseeds the noise makes an experiment lie |
| Add a device | `addSensor<T>`, `addActuator<T>` | a real robot does not grow a camera because a program asks |
| The 3D world | `getWorld`, `getRoot`, `getLink`, `getJointLink`, `getUrdfFrame` | it is the simulator; it does not exist on hardware |
| The backend | `getBackend`, `getPhysics`, `getHealth` | see below |
| Change the robot | writable `getResources()` (faults) and `getKinematics()`, `setId`, `setStartPose` | the setup is done at start or by the scenario |

A command to a sensor (start a capture, turn a camera) is not a change of the sensor: it is an actuator, so it has a resource and goes through the scheduler.

## 🛡 How it is checked

Three checks, from the strongest:

1. The storage. The `Robot` has no member for the world, the entities, the backend nor the true base. `JointSet` knows no entity either: the runtime keeps the entity of each joint.
2. The compiler. `tests/Robot/TestSkillAccess.cpp` asks the compiler whether code that holds a `Robot&` can change a camera, add an actuator, reach the world, or command a robot of the team. The answer must be no, or the tests do not build. The same file checks that `Robot` is not a base of `RobotRuntime`.
3. The layers. `make check-layers` compiles `Task/Skills` and `Task/Planning` without `Robot/RobotRuntime.hpp`, so a skill cannot even name the runtime.

## 🔌 The backend: hardware abstraction and simulated world

`RobotBackend` is the hardware abstraction layer of one robot. It does what is the same in simulation and on hardware: `attach`, `reset`, `step` (commands to the motors, encoders to `measureJoint`), `time()`, `getHealth()`. A hardware driver implements it and nothing else.

`PhysicsWorld` is what only a simulator knows: cast a ray, read a contact, the pose of a free body, the force on a link, hold or move a body. The simulated sensors, the truth of the cell and the grasp models read it with `runtime.getPhysics()`, which is null on hardware. `MujocoBackend` implements both.

The contract of `RobotBackend`, written for a first driver:

- **Time.** In simulation, `step(dt)` moves the world by `dt`. On hardware, time moves alone: `step` writes and reads at the period of the bus, and `time()` gives the stamp of the reading. A late step sends the last commands once; it does not run twice to catch up.
- **Errors.** A lost link, a drive fault or an emergency stop is reported by `getHealth()`, never thrown inside `step`. While the backend is not healthy, the runtime marks every actuator resource failed: the actuators go to their safe state, and the skills and the tree see a fault (`resource lost`). When the backend is healthy again, the runtime restores only what it failed; a fault of the scenario on the same actuator stays.
- **Safety.** The emergency stop and the hardware limits are in the driver and in the robot controller, not in the skills.

## 🚧 Limits

- A tool is given when the robot is built: there is no tool change while it runs, so the `Robot::Model` is the same for the whole life of the robot.
- The Pinocchio model is not shared between two robots of the same URDF: each `Robot` owns its own `Kinematics` (model and data).
- No hardware driver is written yet. The contract above is what the first one must keep.

## 🔗 See also

- [The robot: model, state, backend](02-robot.md)
- [Resources and arbitration](04-resources.md), for what a failed resource does to a skill
- [API reference: the robot](../../reference/api/robot.md), with the three classes side by side
