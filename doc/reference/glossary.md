# 📖 Glossary

[⏫ Reference](README.md)

The words of RobotIK, in simple English. The comments of the headers use them; each one is defined here once. Words are in alphabetical order inside each group.

## 🧅 The three layers

| Layer | In one sentence | Folder |
|-------|-----------------|--------|
| **Robot** | One robot: its joints, sensors, motors and physics. | `include/RobotIK/Robot/` |
| **Task** | Robots and objects that do a job together, with a controller. | `include/RobotIK/Task/` |
| **Package** | The files that describe a job (YAML, plugins) and the verdict of a run. | `include/RobotIK/Package/` |
| **Common** | Small tools every layer uses: units, errors, signals, random. | `include/RobotIK/Common/` |

A layer only uses the layers under it; `make check-layers` checks it.

## 🤖 Robot

- **Actuator**: a device that acts: a motor, a group of joints, a suction gripper. Each one is also a *resource* of the same name.
- **Backend**: what really moves the joints. In simulation it is MuJoCo; on a real robot it would be a driver. The code above does not change.
- **Device**: a sensor or an actuator.
- **Flange**: the plate at the end of an arm, where a tool is screwed.
- **Frame**: a point and three axes, to say where something is and how it is turned. RobotIK uses Z up, metres and radians.
- **IK (inverse kinematics)**: finding the joint angles that put the tool at a given pose.
- **Configuration**: the values of all the joints of a robot together: it says the shape of the robot. A *waypoint* saves one.
- **Joint**: an axis between two links. *Revolute* turns (radians); *prismatic* slides (metres).
- **Jog**: move the robot by one small step each time a button is pushed: one joint (joint jog) or the tool along a line or about an axis (tool jog, which needs IK). See [The teach pendant](../concepts/07-teach-pendant.md).
- **Kinematics**: the geometry of the robot: where each link is for given joint angles. RobotIK uses Pinocchio for it.
- **Capture stamp**: the time a sensor measured, kept by its reading even when the reading arrives later (latency). A belief takes the stamp of the reading it comes from.
- **Latency**: the delay between the moment a sensor measures and the moment the reading is available.
- **Link**: a rigid part of the robot (an arm segment, the wrist).
- **MuJoCo**: the physics engine: gravity, contacts, motor forces.
- **Noise model**: how a sensor spoils the true value: bias, random noise, rounding, limits.
- **Pinocchio**: the library that computes kinematics.
- **Pose**: a position plus a rotation.
- **Lease**: the right to use a set of resources, given all at once or not at all. It gives them back when it is released. See [Resources and arbitration](../concepts/architecture/04-resources.md).
- **Resource**: something a skill reserves to run (the arm, the gripper, the camera). Two skills cannot hold the same resource. A failure makes a resource unavailable.
- **Sensor**: a device that measures: camera, IMU (motion), lidar (distance), force-torque.
- **Robot clock**: the time of a robot, given by its backend after each step: the simulated time, or the stamp of the last reading on hardware.
- **SI units**: metres, seconds, radians, newtons. The types carry them (`Seconds`, `Radians`...), so a length cannot be given for an angle.
- **Smooth move (quintic, rest-to-rest)**: a move of a joint that starts and ends at zero speed, with no jerk; its curve is a polynomial of degree 5. The teach pendant replays waypoints this way.
- **Teach pendant**: the hand-held box an operator uses to jog a robot, save waypoints and replay them. RobotIK has one in the simulator and in C++ (`TeachPendant`). See [The teach pendant](../concepts/07-teach-pendant.md).
- **TCP (tool centre point)**: the working point of the tool (the tip of the brush, the centre of the suction cup).
- **URDF**: an XML file that describes a robot: its links, joints and shapes.
- **Waypoint**: a position of the robot saved to come back to it: the value of every joint, a name and the time to reach it.

## 📦 Assets and packages

- **Asset**: a file a robot project reuses: a robot model, a tool, an object.
- **Asset pack**: a folder of assets with a `pack.yaml` file that lists them, with a version and a license. A model is named `pack_id.key`, for example `robotik.tools.vacuum`.
- **Manifest**: a small file that describes a folder: what it contains, its version, what it needs. `pack.yaml` is the manifest of an asset pack, `plugin.yaml` the manifest of a plugin package.
- **Package (plugin package)**: a folder with a `plugin.yaml`, a shared library, and its `worlds/`, `scenarios/`, `experiments/`, `data/`. Paths in its files start at this folder.
- **Plugin**: a shared library (`.so`) that adds a demo to RobotIK: its skills, its scenario logic, its panels. It talks to the host through a C border (ABI) only.
- **ABI**: the C functions and structures between a plugin and its host. No C++ type crosses it, so a plugin built apart still works.
- **Host**: the program that loads plugins: `RobotIK-Simulator` (window) or `RobotIK-Headless` (command line, no window).
- **Headless**: without a window or a GPU, as in continuous integration.

## 🎯 Task

- **Agent**: something that maps an observation to an action: a learned policy, a fly brain, a scripted player.
- **Behavior tree**: a tree of small actions and conditions, ticked again and again, that decides what the robot does next. RobotIK uses BlackThorn.
- **Belief**: what the robot thinks is true about an object (its pose, if it is held). Beliefs live in the `WorldModel`. They can be wrong or old.
- **Robotic cell (cell)**: the place where robots work, as in a factory: robots, objects, one physics, a controller. The `RoboticCell` class runs it.
- **Controller**: what decides in a cell: a behavior tree, a PDDL planner, or the scenario logic.
- **Detector**: an algorithm that finds objects in an image (color blobs, AprilTags, a neural network).
- **ECS (entity, component, system)**: a way to store data. An *entity* is an id, a *component* is plain data put on it, a *system* is a loop over the entities that have some components. Fast, because data of one kind sits side by side. Compages stores the world this way.
- **Fault**: a failure, scripted or random, of a resource (a camera that stops, a gripper that drops).
- **Fidelity**: how real a part of the simulation is, on three axes (`Robot/Fidelity.hpp`). *Physics* (`PhysicsFidelity`): `abstract` (an object follows the tool), `rigid`, `contact`. *Perception* (`PerceptionFidelity`): `truth`, `noisy_truth`, `sensed` (see *perception fidelity*). *Localization* (`LocalizationFidelity`): `truth`, `plugin`.
- **Ground truth**: the real state of the simulated world. Metrics, assertions and RL rewards may read it. Skills, observers and scenario logics read beliefs instead, unless the scenario asks for a truth fidelity.
- **Grasp model**: how the simulation makes a gripper hold an object, for a given physics fidelity.
- **Scenario logic** (`ScenarioLogic`): the C++ part of a scenario: its skills, its metrics, what it does at each step.
- **Perception fidelity**: where beliefs come from. `truth`: the simulator writes the true pose (a mock, as when a car simulator gives the lane marks instead of an image). `noisy_truth`: the true pose with noise, delay and misses. `sensed`: real detectors on the images. The run report says which one was used.
- **Localization**: where the base pose a skill reads comes from. `truth`: the true base, copied (a mock). `plugin`: an algorithm of the plugin estimates it. Sensors and truth queries always use the true base. See [Localization](scenario-file.md#-localization).
- **Belief error**: the gap between a belief and the truth, measured by the conditions `believed`, `belief_error` and `base_error`. See [Check what the robot believes](scenario-file.md#-check-what-the-robot-believes).
- **Preemption**: a request of higher priority takes the resources of a running skill. The scheduler preempts all the holders of a request or none. A preempted skill that may wait starts again later.
- **Run (of a skill)**: one execution of a skill, from its request to its end. Its ticket is a `RunId`.
- **RunId (ticket)**: what `request` returns: the skill and the number of the request. A caller reads and cancels its own run with it.
- **Scheduler**: decides which requested skills can run now (resources, priorities) and runs them.
- **Skill**: one behaviour of the robot (move to a pose, grasp, dip a brush). A skill reads only its `RobotContext`: the robot and the beliefs.
- **Stopping**: the state of a cancelled or preempted run that keeps its resources until the skill says the robot is safe.
- **Writer**: who sends the commands now: the skill that the scheduler ticks, or a command outside the skills. An actuator refuses a command from a writer that does not hold its resource.

## 🧠 Planning

- **Action (PDDL)**: a step the planner may choose, with its parameters, preconditions and effects.
- **Atom / literal**: a fact such as `(holding gripper red_cube)`; a literal is an atom or its negation.
- **Binding**: links a PDDL action to the skills that do it.
- **Domain**: the PDDL file that lists types, predicates and actions.
- **Effect**: what an action makes true or false. RobotIK checks it on the beliefs after the step; with `verify: observed`, only a belief seen again after the start of the step counts.
- **GOAP** (Goal-Oriented Action Planning): a planner of video games. From the goal it builds a graph of states linked by actions, and an A* search finds the cheapest plan. RobotIK has no GOAP; `planner.method: search` does the same job with Kairos ([Without search: a tree grown from the goal (PA-BT)](../concepts/05-pddl-planner.md#-without-search-a-tree-grown-from-the-goal-pa-bt)).
- **Kairos**: the PDDL planner RobotIK uses.
- **Observer**: a function that tells whether a predicate holds, from the beliefs, never from the ground truth.
- **PA-BT** (Planning and Acting with Behavior Trees): `planner.method: pabt`. No search, unlike GOAP: a reactive behavior tree grows backwards from the goal during the run, each false condition getting the actions that make it true ([Without search: a tree grown from the goal (PA-BT)](../concepts/05-pddl-planner.md#-without-search-a-tree-grown-from-the-goal-pa-bt)).
- **PDDL**: a language to describe a planning problem: what can be done and what is wanted.
- **Plan**: the list of actions the planner found. Steps that do not depend on each other run together.
- **Predicate**: the name of a kind of fact (`holding`, `in`).
- **Replan**: plan again when a step fails or an effect is not seen.

## 🎬 Scenarios and runs

- **Actor**: an entity of the world that moves by itself, a person or a vehicle. The story moves it; no skill commands it and no physics acts on it ([actors](world-file.md#-actors)).
- **Assertion**: a condition checked during or at the end of a run (`(inside red_cube box)`), with a verdict: passed, failed or pending.
- **Case**: one variant of an experiment (a scenario plus overrides).
- **Condition**: a question about the run, in the notation of PDDL: `(< time 30)`, `(inside red_cube box)`, `(and ... )`. The operator comes first, inside parentheses. RobotIK reads it once when the scenario loads and refuses an unknown name ([conditions](scenario-file.md#-conditions)).
- **Event (of a story)**: a list of actions that one actor plays when a condition becomes true ([events](story-file.md#-events)). Not a host event of a plugin.
- **Experiment**: several cases, each run with several seeds, compared.
- **Metric**: a named number a run publishes (`time`, `laps`).
- **Outcome**: why a run ended: success, failure, timeout, stop condition.
- **Place**: a named point of the floor in a story (`counter: [1.6, 0.0]`), a target of the moves.
- **Run report**: what a run leaves: outcome, verdicts, metrics, seed, and how real it was (physics and perception fidelity).
- **Scenario**: a world plus what happens in it (controller, faults) and how it is judged (timeout, assertions).
- **Seed**: the number that fixes all random draws. Same seed, same run.
- **Story**: what the actors of the world do during a scenario: places, variables and events, in the `story:` section of the scenario file ([the story](story-file.md)).
- **Utterance**: a sentence an actor said, with its id, its text, whom it was said to and when: `(said woman hello)`.
- **World**: the file that lists what exists at the start: robots, tools, objects and their poses.

## 🎲 Learning

- **Environment**: a world an agent learns in: reset, step, reward.
- **Episode**: one try, from a reset to the end.
- **Observation / action**: the numbers an agent receives / sends.
- **Policy**: a learned agent, often a neural network (ONNX file).
- **Privileged observation**: a value taken from the ground truth; a real robot could not measure it.
- **Reward**: the score of a step.
