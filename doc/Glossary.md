# Glossary

The words of RobotIK, in simple English. The comments of the headers use
them; each one is defined here once. Words are in alphabetical order inside
each group.

## The three layers

| Layer | In one sentence | Folder |
|-------|-----------------|--------|
| **Robot** | One robot: its joints, sensors, motors and physics. | `include/Robotik/Robot/` |
| **Task** | Robots and objects that do a job together, with a controller. | `include/Robotik/Task/` |
| **Package** | The files that describe a job (YAML, plugins) and the verdict of a run. | `include/Robotik/Package/` |
| **Common** | Small tools every layer uses: units, errors, signals, random. | `include/Robotik/Common/` |

A layer only uses the layers under it; `make check-layers` checks it.

## Robot

- **Actuator**: a device that acts: a motor, a group of joints, a suction
  gripper. Each one is also a *resource* of the same name.
- **Backend**: what really moves the joints. In simulation it is MuJoCo; on a
  real robot it would be a driver. The code above does not change.
- **Device**: a sensor or an actuator.
- **Flange**: the plate at the end of an arm, where a tool is screwed.
- **Frame**: a point and three axes, to say where something is and how it is
  turned. RobotIK uses Z up, metres and radians.
- **IK (inverse kinematics)**: finding the joint angles that put the tool at
  a given pose.
- **Joint**: an axis between two links. *Revolute* turns (radians);
  *prismatic* slides (metres).
- **Kinematics**: the geometry of the robot: where each link is for given
  joint angles. RobotIK uses Pinocchio for it.
- **Latency**: the delay between the moment a sensor measures and the moment
  the reading is available.
- **Link**: a rigid part of the robot (an arm segment, the wrist).
- **MuJoCo**: the physics engine: gravity, contacts, motor forces.
- **Noise model**: how a sensor spoils the true value: bias, random noise,
  rounding, limits.
- **Pinocchio**: the library that computes kinematics.
- **Pose**: a position plus a rotation.
- **Resource**: something a skill reserves to run (the arm, the gripper, the
  camera). Two skills cannot hold the same resource. A failure makes a
  resource unavailable.
- **Sensor**: a device that measures: camera, IMU (motion), lidar (distance),
  force-torque.
- **SI units**: metres, seconds, radians, newtons. The types carry them
  (`Seconds`, `Radians`...), so a length cannot be given for an angle.
- **TCP (tool centre point)**: the working point of the tool (the tip of the
  brush, the centre of the suction cup).
- **URDF**: an XML file that describes a robot: its links, joints and shapes.

## Assets and packages

- **Asset**: a file a robot project reuses: a robot model, a tool, an object.
- **Asset pack**: a folder of assets with a `pack.yaml` file that lists them,
  with a version and a license. A model is named `pack_id.key`, for example
  `robotik.tools.vacuum`.
- **Manifest**: a small file that describes a folder: what it contains, its
  version, what it needs. `pack.yaml` is the manifest of an asset pack,
  `plugin.yaml` the manifest of a plugin package.
- **Package (plugin package)**: a folder with a `plugin.yaml`, a shared
  library, and its `worlds/`, `scenarios/`, `experiments/`, `data/`. Paths in
  its files start at this folder.
- **Plugin**: a shared library (`.so`) that adds a demo to RobotIK: its
  skills, its mission, its panels. It talks to the host through a C border
  (ABI) only.
- **ABI**: the C functions and structures between a plugin and its host. No
  C++ type crosses it, so a plugin built apart still works.
- **Host**: the program that loads plugins: `Robotik-Simulator` (window) or
  `Robotik-Headless` (command line, no window).
- **Headless**: without a window or a GPU, as in continuous integration.

## Task

- **Agent**: something that maps an observation to an action: a learned
  policy, a fly brain, a scripted player.
- **Behavior tree**: a tree of small actions and conditions, ticked again and
  again, that decides what the robot does next. RobotIK uses BlackThorn.
- **Belief**: what the robot thinks is true about an object (its pose, if it
  is held). Beliefs live in the `WorldModel`. They can be wrong or old.
- **Cell (robot cell)**: the place where robots work, as in a factory:
  robots, objects, one physics, a controller. The `Cell` class runs it.
- **Controller**: what decides in a cell: a behavior tree, a PDDL planner, or
  the mission code.
- **Detector**: an algorithm that finds objects in an image (color blobs,
  AprilTags, a neural network).
- **ECS (entity, component, system)**: a way to store data. An *entity* is an
  id, a *component* is plain data put on it, a *system* is a loop over the
  entities that have some components. Fast, because data of one kind sits
  side by side. Compages stores the world this way.
- **Fault**: a failure, scripted or random, of a resource (a camera that stops,
  a gripper that drops).
- **Fidelity**: how real a part of the simulation is. *Physics*: `abstract`
  (an object follows the tool), `rigid`, `contact`. *Perception*: `truth`,
  `noisy_truth`, `sensed` (see *perception fidelity*).
- **Ground truth**: the real state of the simulated world. Metrics,
  assertions and RL rewards may read it. Skills, observers and missions read
  beliefs instead, unless the scenario asks for a truth fidelity.
- **Grasp model**: how the simulation makes a gripper hold an object, for a
  given physics fidelity.
- **Mission**: the C++ part of a demo: its skills, its metrics, what it does
  at each step.
- **Perception fidelity**: where beliefs come from. `truth`: the simulator
  writes the true pose (a mock, as when a car simulator gives the lane marks
  instead of an image). `noisy_truth`: the true pose with noise, delay and
  misses. `sensed`: real detectors on the images. The run report says which
  one was used.
- **Scheduler**: decides which requested skills can run now (resources,
  priorities) and runs them.
- **Skill**: one behaviour of the robot (move to a pose, grasp, dip a brush).
  A skill reads only its `RobotContext`: the robot and the beliefs.

## Planning

- **Action (PDDL)**: a step the planner may choose, with its parameters,
  preconditions and effects.
- **Atom / literal**: a fact such as `holding(gripper, red_cube)`; a literal
  is an atom or its negation.
- **Binding**: links a PDDL action to the skills that do it.
- **Domain**: the PDDL file that lists types, predicates and actions.
- **Effect**: what an action makes true or false. RobotIK checks it on the
  beliefs after the step; with `verify: observed`, only a belief seen again
  after the start of the step counts.
- **Kairos**: the PDDL planner RobotIK uses.
- **Observer**: a function that tells whether a predicate holds, from the
  beliefs, never from the ground truth.
- **PDDL**: a language to describe a planning problem: what can be done and
  what is wanted.
- **Plan**: the list of actions the planner found. Steps that do not depend
  on each other run together.
- **Predicate**: the name of a kind of fact (`holding`, `in`).
- **Replan**: plan again when a step fails or an effect is not seen.

## GRAFCET and workflows

- **Buffer**: where parts wait between an operator and the next one of
  their route; one part per place (`buffers:` of a workflow).
- **Cell device**: a device of the cell that belongs to no robot (a conveyor
  belt); its name is a resource of the cell (`Cell::devices()`).
- **Conveyor belt**: the top face of a fixed box body; the parts resting on
  it go at its speed while it runs (`type: conveyor`, skill `<belt>.Run`).
- **Critical circuit**: the circuit of a timed event graph that sets the
  cycle time: the bottleneck of the cell.
- **Cycle time (λ)**: the period of a cell in steady state; the throughput is
  the number of products per cycle divided by λ.
- **Deadlock**: a situation where nothing can go on: each part waits for
  what another one holds (a step holds the belt, another step waits for it).
- **Flowshop**: a cell where every product visits the operators in the same
  order.
- **Forcing order**: an action of a GRAFCET step that holds another graph in a
  given situation (`Production:{init}`); the forced graph does not evolve.
- **GEMMA**: a guide to describe the start, stop and emergency modes of a
  machine; in RobotIK, a GRAFCET of three graphs (safety, control,
  production).
- **GRAFCET**: a chart of steps and transitions (IEC 60848). An active step
  does its actions; a transition fires when the steps above it are active and
  its receptivity is true.
- **Operator (workflow)**: who works in a cell: a robot, a machine, a camera
  (word of SynDEx).
- **Operator input**: an input of a GRAFCET that no skill nor observer gives:
  a button (`Dcy`, `AU`).
- **Recipe**: the skills an operator runs to take a part or to give it
  (`take:`, `give:` of a workflow); the generated GRAFCET has one step per
  skill.
- **Receptivity**: the condition of a GRAFCET transition (`done(Home) . /AU`).
- **Timed event graph**: a Petri net where each place has one input and one
  output transition; its behavior is linear in the (max,+) algebra.
- **Workflow**: the operators, products and routes of a cell
  (`x.workflow.yaml`); with recipes, buffers and parts, the GRAFCET of the
  cell is generated from it.
- **Zone**: a buffer where one operator at a time may work; the one taking
  parts goes first.

## Scenarios and runs

- **Assertion**: a condition checked during or at the end of a run
  (`inside(red_cube, box)`), with a verdict: passed, failed or pending.
- **Case**: one variant of an experiment (a scenario plus overrides).
- **Experiment**: several cases, each run with several seeds, compared.
- **Metric**: a named number a run publishes (`time`, `laps`).
- **Outcome**: why a run ended: success, failure, timeout, stop condition.
- **Run report**: what a run leaves: outcome, verdicts, metrics, seed, and
  how real it was (physics and perception fidelity).
- **Scenario**: a world plus what happens in it (controller, faults) and how
  it is judged (timeout, assertions).
- **Seed**: the number that fixes all random draws. Same seed, same run.
- **World**: the file that lists what exists at the start: robots, tools,
  objects and their poses.

## Learning

- **Environment**: a world an agent learns in: reset, step, reward.
- **Episode**: one try, from a reset to the end.
- **Observation / action**: the numbers an agent receives / sends.
- **Policy**: a learned agent, often a neural network (ONNX file).
- **Privileged observation**: a value taken from the ground truth; a real
  robot could not measure it.
- **Reward**: the score of a step.
