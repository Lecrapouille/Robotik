# 9. The demos

Each demo is a plugin in `plugins/`. Each one shows **one idea** of robotics,
and each one is also a test: its scenarios have assertions, and they run in
Headless.

| Plugin | Idea | Scenarios | Experiment | Extra program |
|--------|------|-----------|------------|---------------|
| [HelloWorld](#1-helloworld) | Learn the API, one lesson per scenario | hello, wave, reach, skills | `hello_experiment` | — |
| [PickAndPlace](#2-pick-and-place) | Perception + skills + behavior tree or PDDL planner + faults | nominal, faults, pddl, pddl_faults | `pick_and_place_experiment` | — |
| [Painter](#3-painter) | Free bodies, trajectories in files, "nothing moved", planned layers | painter, painter_pddl | `painter_experiment` | — |
| [LineFollower](#4-line-follower) | Mobile robot, localisation, control loop | line_follower | — | `Robotik-LineFollower` |
| [PickAndPlace](#5-pick-and-place-rl) | RL environment, parallel pool | pick_and_place_rl | — | `Robotik-PickAndPlaceRL` |
| [FlyBrain](#6-fly-brain) | A biological brain as an agent | fly_obstacle_avoidance | — | `Robotik-Fly` |
| [HumanoidWalk](#7-humanoid-walk) | A walking policy learned elsewhere (ONNX), on a floating base | walk, walk_circle | — | — |
| [DualArm](#8-dual-arm) | Two robots, one plan: steps in parallel, required concurrency | conveyor, conveyor_sequential, tray | — | — |
| [MicroDuck](#9-micro-duck) | A small walking robot in a room: a learned policy, real contacts | walk_forward, tour, drive | — | — |

All demos open from the simulator **Demos** menu, and run with:

```bash
./build/Robotik-Headless build/plugins/<Plugin>/scenarios/<scenario>.scenario.yaml
```

---

## 1. HelloWorld

The tutorial plugin. Read it with [04-Your-First-Plugin.md](04-Your-First-Plugin.md).

| Scenario | Shows |
|----------|-------|
| `hello` | The smallest mission: a metric and an end |
| `wave` | Joint commands, forward kinematics |
| `reach` | Inverse kinematics, a `within` assertion that turns green |
| `skills` | A skill (`Wave`) called by a behavior tree |

The plugin also has a panel, a menu entry and a key binding (**H** = Home).
It is used by the plugin unit tests (`tests/Plugin/TestPlugin.cpp`).

---

## 2. Pick and place

**The reference demo**: look at it first to understand the architecture end
to end. [Video](https://youtu.be/MuWdpaLvGoU?si=qjJWl1kZFv4DRHOo).

A 6-axis arm with a wrist camera and a vacuum cup takes a red cube and drops
it in a box:

```text
Home → Detect → Approach → Reach → Grasp → Approach → Transit(box) → Approach(box) → Release → Home
```

Files: `plugins/PickAndPlace/`
- `worlds/pick_place_cell.world.yaml`: the arm, its camera, the vacuum cup and four
  stored tools, the cube (randomized), the box;
- `scenarios/pick_and_place.scenario.yaml`: nominal run;
- `scenarios/pick_and_place_faults.scenario.yaml`: the camera fails during the run;
- `scenarios/pick_and_place.bt.yaml`: the tree (`UntilSuccess` around `Detect`
  waits for the camera to come back);
- `src/`: `ColorDetector` (perception), the skills, `PickPlaceMission`.

The assertions judge the result, not the trajectory:

```yaml
assertions:
  - controller.success
  - inside(red_cube, box)
  - holding(gripper) == 0
  - { condition: "collision_count(robot) == 0", phase: always }
  - time < 30
```

In headless, no image is rendered: the scenario says
`perception: { blind: truth }`, so the true cube position replaces the
camera, and reports say `perception truth`. The simulator renders the image
and uses the colour detector.

The fault scenario does not only check that things fail: it checks that the
architecture **reacts** (retry, fall back, abort).

The same job runs with the other controllers: the PDDL planner
(`pick_and_place_pddl*.scenario.yaml`, [chapter 11](11-PDDL-Planner.md)),
and a GRAFCET (`pick_and_place_grafcet.scenario.yaml`, with a Retry step
when the camera is lost) or a GEMMA of three graphs with its operator buttons
Dcy, AU, Rearm (`pick_and_place_gemma*.scenario.yaml`,
[chapter 12](12-GRAFCET-and-Workflows.md)). `pick_and_place.workflow.yaml`
gives the cycle time of the cell.

```bash
./build/Robotik-Headless build/plugins/PickAndPlace/scenarios/pick_and_place.scenario.yaml --seed 11
./build/Robotik-Headless build/plugins/PickAndPlace/experiments/pick_and_place.experiment.yaml \
    --report pick.json
```

The experiment runs three seeds, a wider cube layout, and a camera down at
start.

**With the PDDL planner** (`scenarios/pick_and_place_pddl.scenario.yaml` and
`pick_and_place_pddl_faults.scenario.yaml`, menu **PDDL planner**): no tree, only the
goal `in(red_cube, box)`. The domain is
`scenarios/pick_and_place.domain.pddl`; `src/PickPlacePlanning.cpp` reads
the state on the beliefs and binds each action to the same skills. With the
camera down, the executor plans again until it can go on. See
[11. The PDDL planner](11-PDDL-Planner.md#2-tutorial-run-a-planned-pick-and-place).

---

## 3. Painter

An arm dips a brush in four paint pots, paints one stroke per colour on a
canvas standing on an easel, and rinses the brush between colours.

- The easel is a **free body**: pushed, it slides or tips over. The canvas is
  a free box resting on it. Both are entities of
  `plugins/Painter/worlds/painting_lab.world.yaml`; the mission finds them by id and
  only brings the gestures.
- The strokes are **trajectories** in `scenarios/trajectories/`, written in
  the easel frame.
- Custom tree nodes (`Dip`, `Mark`, `Rinse`) give the colour to the skills.
- **Straight strokes.** `Mark` cuts each stroke into 2 mm Cartesian steps,
  each one an IK target, and the arm joints are stiff (`kp` 1500, `kd` 80).
  Gains alone are not enough: between two far targets each joint moves on
  its own, and the brush draws an arc (2 to 3 cm off the line). The plugin
  publishes `painter.stroke_deviation`, the largest distance between the
  inked points and the straight line (about 1.5 mm), and the scenario
  asserts it. All of it is in `parameters:` of the scenario:

  ```yaml
  parameters:
    stroke_step: 0.002
    stroke_tolerance: 0.02
    arm_gains: { kp: 1500, kd: 80 }
  ```

Zero collision does not prove the easel stayed in place, so the scenario
measures it:

```yaml
  - { condition: "pose_error(easel) < 0.002", phase: always }
  - { condition: "pose_error(canvas) < 0.005", phase: always }
  - { condition: "on(canvas, easel)", phase: end }
  - name: brush loaded before the first stroke
    sequence: [ "skill_succeeded(dip)", "skill_started(mark)" ]
```

The experiment `painter_experiment` paints again on a slippery floor and with
a canvas four times heavier.

**With the PDDL planner** (`scenarios/painter_pddl.scenario.yaml`): the goal is the
four layers painted and a clean brush. The facts `before(red, black)`… give
the order of the layers; the planner finds the twelve gestures (dip, mark,
rinse per colour). The robot cannot see the paint: the observers read what
the gestures did (`Stroke::brush`, `Stroke::painted` in
`src/PainterScene.hpp`).

---

## 4. Line follower

A mobile robot with two wheels, a camera, AprilTags on the floor, and a line
to follow for one lap. [Video](https://youtu.be/T0Oop_cmlPw?si=UKg83e5xunciYAZ1).

```text
camera → perception (AprilTag, line) → localisation → controller → differential robot
```

The simulated **odometry is wrong on purpose**: it drifts. A perfect
odometry would prove nothing about localisation. The tags correct the
estimate, and you can draw the truth against the estimate:

```bash
./build/Robotik-LineFollower --seed 4 --laps 1 --save map.png   # green: truth, red: estimate
./build/Robotik-LineFollower --view                              # camera on screen
./build/Robotik-Headless build/plugins/LineFollower/scenarios/line_follower.scenario.yaml --seed 4
```

Tree: `Localize`, then `UntilSuccess` (3 attempts) of `ReachLine` +
`FollowLine`. Assertions: `laps >= 1`, `cross_track.max < 0.08`, `time < 150`.

Needs OpenCV and AprilTag at build time.

---

## 5. Pick and place RL

The classic learning loop on the pick-and-place cell: `Environment`, parallel
pool, reproducibility. Training a real PPO or SAC policy is **not** the goal
here: it validates the infrastructure.

```bash
./build/Robotik-PickAndPlaceRL --policy converged --envs 16 --threads 8 --episodes 32 \
    --log episodes.json
```

This scenario lives in the PickAndPlace plugin, beside the behavior tree
ones (submenus of the **Demos** menu). Its entry in `plugin.yaml` has
`clock: plugin`: the plugin `update` advances the simulation. The pool and
the plugin share `PickPlaceControl.cpp`. Details in
[08-Reinforcement-Learning.md](08-Reinforcement-Learning.md).

---

## 6. Fly brain

A fly-like agent reaches food while avoiding obstacles.
[Video](https://youtu.be/ao2lA3Rlmrc?si=DECtcn7l44Q-iMCo).

```text
sensors → observation → brain → action → controller → body
```

The brain knows neither MuJoCo, nor Compages, nor the URDF. Two brains share
this surface: a reflex rule, and the integrate-and-fire network of Shiu et
al. 2024, which can load the FlyWire connectome (`--edges`, `--binding`);
without those files it runs a small circuit with the same equations.

The body is the ×100 model (`drosophila_x100.urdf`, about 28 cm). Flight is
plain kinematics: wings, legs and neck are a posture, not a wind tunnel. The
flight model is the robot backend (`FlyBackend`), the brain is the agent the
mission runs. The eye beams and the trail are decorations: ignored by
grasping, range sensors and ground truth.

```bash
./build/Robotik-Fly --brain reflex
./build/Robotik-Fly --view
./build/Robotik-Fly --headless --envs 8 --episodes 8 --seed 123456
./build/Robotik-Headless build/plugins/FlyBrain/scenarios/fly_obstacle_avoidance.scenario.yaml
```

In the simulator, the **Fly brain** panel shows the state and chooses the
brain: the small circuit, or the FlyWire connectome (loaded in the
background, a few seconds). The **Cameras** panel shows both eyes.

---

## 7. Humanoid walk

A LimX HU_D04_01 humanoid (31 joints) walks in a circle with the walking
policy trained by LimX. No training here: the policy (`walk.onnx`) and the
robot come from the LimX repositories (Apache-2.0), and ONNX Runtime runs the
policy. It shows that a **learned controller** is just another skill.

```bash
make -C plugins/HumanoidWalk download-external-libs   # LimX repositories + ONNX Runtime release
make -C plugins/HumanoidWalk -j"$(nproc)"             # prepares data/hu_d04/ once, then builds
./build/Robotik-Headless build/plugins/HumanoidWalk/scenarios/walk_circle.scenario.yaml --trace
```

```text
  0 s  Stand  SUCCEEDED  (0.999 s)
  1 s  Walk  SUCCEEDED  (24.999 s)
[PASS] controller.success  (controller.success = 1)
[PASS] humanoid.height_min > 0.7  (humanoid.height_min = 0.882378)
[PASS] humanoid.turns > 1  (humanoid.turns = 1.30405)
[PASS] humanoid.radius > 0.6  (humanoid.radius = 0.918705)
[PASS] humanoid.radius < 1.4  (humanoid.radius = 0.918705)
```

| Scenario | What happens |
|----------|--------------|
| `walk` | (simulator) the robot walks until you stop the run; the **Humanoid** panel changes the command (forward, left, turn, Stop) |
| `walk_circle` | (test) stand 1 s, then walk 25 s on a circle of `vx / yaw_rate` = 1 m, judged by assertions |

How it works:

- The tree is two skills, `Stand` then `Walk` (`walk_circle.bt.yaml`).
- `WalkMission` plugs a **floating-base** MuJoCo backend, without gravity
  compensation, with the PD gains of the policy on each joint.
- The plugin owns the clock (`clock: plugin` in `plugin.yaml`): it steps the
  physics every 1 ms, and `Walk` runs the policy every 10 steps (100 Hz) on
  the **IMU and the joint encoders only**. Ground truth only feeds the
  metrics (`humanoid.height_min`, `humanoid.turns`, `humanoid.radius`, …).
- When the IMU is lost (fault) or the robot lies down, `Walk` fails and puts
  the joints in damping mode, as the real robot does: it falls.

More details in [plugins/HumanoidWalk/README.md](../plugins/HumanoidWalk/README.md).

---

## 8. Dual arm

**Two robots in one world, planned together.** Two 6-axis arms share one
physics, one scheduler and their beliefs ([03 §4](03-Worlds-and-Scenarios.md#several-robots)).
The PDDL planner plans for both, and the executor runs at the same time the
steps that do not depend on each other ([11 §4](11-PDDL-Planner.md#steps-that-run-together)).
No behavior tree is written.

```bash
make -C plugins/DualArm -j"$(nproc)"
./build/Robotik-Headless build/plugins/DualArm/scenarios/conveyor.scenario.yaml --trace
```

| Scenario | What happens |
|----------|--------------|
| `conveyor` | A conveyor of two lanes brings four parts from its infeed table (out of reach) to the end stop of each lane; each arm takes the parts of its lane and drops them in its bin, then waits over its pick zone for the next part. The belt brings the next part while an arm carries the last one |
| `conveyor_sequential` | The same plan, step by step (`planner.parallel: false`), to compare |
| `tray` | The left arm holds a tray with four glasses **from below**, its cup pointing up, carries it to another stand and puts it down; the right arm, on a pedestal, feels the tray with a drill and its force sensor, and draws a target between the glasses **while the tray moves**. A temporal domain (durative actions) |

**Conveyor.** One plan of 12 steps; the times show the overlap:

```text
  0 s  Convey(p4, feeder, zone_right)  SUCCEEDED  (2.56 s)
  2.57 s  right.Hover(p4)  SUCCEEDED  (0.95 s)
  2.57 s  Convey(p3, feeder, zone_left)  SUCCEEDED  (2.91 s)
  3.53 s  right.Reach(p4)  SUCCEEDED  (0.4 s)
  3.94 s  right.Grasp(p4)  SUCCEEDED  (0.01 s)
  3.96 s  right.Hover(p4)  SUCCEEDED  (0.41 s)
  4.38 s  right.Over(bin_right)  SUCCEEDED  (2.09 s)
  5.49 s  left.Hover(p3)  SUCCEEDED  (0.95 s)
  5.49 s  Convey(p2, feeder, zone_right)  SUCCEEDED  (2.91 s)
  6.45 s  left.Reach(p3)  SUCCEEDED  (0.4 s)
  6.48 s  right.Release  SUCCEEDED  (0.01 s)
  6.5 s  right.Ready(zone_right)  SUCCEEDED  (2.08 s)
  ...
  8.59 s  right.Hover(p2)  SUCCEEDED  (0.28 s)
  ...
[PASS] dual_arm.overlap > 2  (dual_arm.overlap = 8.22)
[PASS] time < 20  (time = 16.85)
```

Run step by step, the same plan ends at 33.27 s instead of 16.85 s.

- **The belt.** The conveyor is a fixed body (`robotik.dual_arm.conveyor`,
  a model of the plugin pack in `data/`). The belt is a device of the
  plugin with its resource `conveyor`: `Convey(part, feeder, zone)` pushes
  the part off the infeed table, across to the lane of the zone, and down
  the lane to its end stop. It holds the resource alone, so two conveys
  never overlap, and a fault on `conveyor` stops the belt.
- **Waiting at the right place.** `place` ends with `Ready(zone)`: the arm
  comes back over its pick zone, at the height it takes the next part from.
  The part arrives under the cup (`right.Ready(zone_right)` runs during
  `Convey(p2, …)`), so the next `Hover` takes 0.28 s instead of 0.95 s. The
  zone is a parameter of `place` in the domain, bound by the static fact
  `reaches(arm, zone)`.

**Tray.** The plan holds the tray during the drilling: `drill` needs the
tray `in_motion` over all its duration, a token `carry` adds at its start
and removes at its end. The carry motion ends at 25.52 s; the executor keeps
the tray held until the drill is done (30.09 s), then lowers it.

```text
  0 s  left.Approach(tray)  SUCCEEDED  (2.5 s)
  2.51 s  left.Reach(tray)  SUCCEEDED  (0.99 s)
  3.51 s  left.Grasp(tray)  SUCCEEDED  (0.01 s)
  3.53 s  left.Raise  SUCCEEDED  (1.99 s)
  5.53 s  left.Carry(pad_b)  SUCCEEDED  (19.99 s)
  5.53 s  right.Trace(tray)  SUCCEEDED  (24.56 s)
  30.11 s  left.Lower  SUCCEEDED  (1.99 s)
  32.11 s  left.Release  SUCCEEDED  (0.01 s)
  32.13 s  left.Retreat  SUCCEEDED  (0.99 s)
  plan 0 at 0 s (start): solved (lift left tray pad_a) (carry left tray pad_a pad_b) (drill right tray) (set_down left tray pad_b)
[PASS] on(tray, stand_b)  (on(tray, stand_b) = 1)
[PASS] on(g1, tray)  (on(g1, tray) = 1)
...
[PASS] collision_count(left) == 0  (collision_count(left) = 0)
[PASS] collision_count(right) == 0  (collision_count(right) = 0)
[PASS] dual_arm.contact > 15  (dual_arm.contact = 18.44)
[PASS] dual_arm.overlap > 4  (dual_arm.overlap = 19.99)
```

How it works:

- **One arm under, one arm above.** The stands (`robotik.dual_arm.stand`)
  have two rails, open toward the middle of the cell. The cup of the left
  arm points up: it slides between the rails under the tray
  (`Approach`), rises to 5 mm under its bottom face (`Reach`) and holds it
  there. The drill comes from above. The arms never meet.
- **Why so high.** With the tool pointing up, the wrist of the 6-axis arm
  bends at most 100°: the forearm must stay about horizontal, so the tray
  is at 0.80 m, near the full reach of the arm. With the tool pointing
  down, the forearm may rise at most 10° above horizontal: the drilling arm
  stands on a pedestal of 0.55 m.
- **Synchronized arms.** `Trace(tray)` draws its path **in the frame of the
  tool of the carrying arm**, read through `RobotContext::team` (what that
  arm measures of itself), not through the camera. It comes 12 cm above the
  start of the path and goes down.
- **The contact is felt.** A force sensor measures the force on the spindle
  (`right_ft: { type: force_torque, link: drill_spindle }` on the drill).
  Read above the tray, it gives the weight of the tool (the tare). The drill
  goes down slowly until it feels 1 N more: the tray is there, wherever the
  beliefs put it. Then it follows the path at 6 cm/s and moves up or down to
  keep 5 N (an admittance). Over a path of about 19 s, it touches the tray
  18.4 s.
- **The path is a file.** `scenarios/trajectories/drill_target.trajectory.yaml`
  draws a target in the frame of the tray: a line through the corridor
  between the glasses along x, a line through the other corridor along y,
  and a circle in the free middle of the 2 × 2 glasses. The body of the
  drill passes 3.3 cm from the nearest glass. Edit it to draw another path.
- **A still drill.** The spindle does not turn: a turning bit drags the
  tray, and its glasses, by friction. The arm joint group lists the six arm
  joints, so the spindle is not one of them.
- **Real glasses.** The cup is at the `rigid` physics level, so the tray and
  the glasses are free bodies of MuJoCo; `on(g1, tray)` with `phase: always`
  is a real contact at every step. `dual_arm.contact` counts the seconds the
  drill really touched the tray (`touching(right_drill, tray)`).
- **Touching is the work.** `touches: [tray]` on the drill
  ([03 §4](03-Worlds-and-Scenarios.md#tools)): its contacts with the tray
  are not collisions; a contact with a glass, a stand or the other arm is.
- **Gentle moves.** The tray moves along straight lines with a smooth
  profile (`PathSkill`). The arms have stiffer joint gains than the
  default: the gravity compensation of the drives does not know the 0.7 kg
  of the tray.
- **What is real, what is not.** Real (MuJoCo): the tray and the glasses
  (free bodies: gravity, contacts, friction), the arms (joints driven by
  their servos), the contact of the drill on the tray, the force it feels.
  Simplified: the cup holds by a weld that never slips (the `contact`
  fidelity does not exist yet), the drill removes no material and its
  spindle does not turn, and the beliefs are the true poses
  (`perception: { fidelity: truth }`, no camera detection).

Files: `plugins/DualArm/`
- `worlds/conveyor_cell.world.yaml`, `worlds/tray_cell.world.yaml`;
- `data/`: the pack `robotik.dual_arm` (conveyor, stand, pedestal);
- `scenarios/trajectories/drill_target.trajectory.yaml` (the path of the drill);
- `scenarios/conveyor.domain.pddl` (classical), `scenarios/tray.domain.pddl` (temporal);
- `src/ConveyorMission.cpp`, `src/TrayMission.cpp`: skills of each robot,
  observers, bindings; `src/Conveyor.cpp`: the belt; `src/ArmSkills.cpp`,
  `src/TraySkills.cpp`: the skills.

Limits: the parts and glasses are boxes (the only free shape); the parts
are given by the simulator (`perception: { fidelity: truth }`), as the
plugin has no detector yet; the arms are the generic `robot_6axis`.

---

## 9. Micro Duck

**A small legged robot in a room.** The Pollen Robotics Microduck (14
servos, about 25 cm) walks with the official `velstand` policy (ONNX),
like the humanoid of [§7](#7-humanoid-walk), but in a room: the walls and
the furniture are one fixed body of MuJoCo, and a light toy box is a free
body the duck pushes. No training is needed.

```bash
make -C plugins/MicroDuck download-external-libs   # microduck_rl + ONNX Runtime release
make -C plugins/MicroDuck -j"$(nproc)"             # prepares data/microduck/ once, then builds
./build/Robotik-Headless build/plugins/MicroDuck/scenarios/tour.scenario.yaml --trace
```

| Scenario | What happens |
|----------|--------------|
| `walk_forward` | Walk straight for 8 s on an empty floor (a test) |
| `tour` | Push the toy box, turn left, walk, turn left, walk back (a behavior tree) |
| `drive` | (simulator) the **Micro Duck** panel gives the command (forward, left, turn, Stop) |

```text
  0 s  Stand  SUCCEEDED  (0.015 s)
  0.02 s  Forward  SUCCEEDED  (5 s)
  5.025 s  TurnLeft  SUCCEEDED  (2 s)
  7.03 s  Forward  SUCCEEDED  (5 s)
  12.035 s  TurnLeft  SUCCEEDED  (2 s)
  14.04 s  Forward  SUCCEEDED  (5 s)
[PASS] controller.success  (controller.success = 1)
[PASS] duck.height_min > 0.09  (duck.height_min = 0.112537)
[PASS] pose_error(toy) > 0.15  (pose_error(toy) = 0.330871)
[PASS] duck.distance > 2.0  (duck.distance = 3.10567)
[PASS] default                  seed=1      controller succeeded at 19.05 s (physics abstract, toy: rigid)  time=19.05
tour: 1/1 runs passed
```

How it works:

- **The physics of the core.** The mission does not build its own
  physics. In `setup` it tunes `Cell::physicsOptions()` (a floating
  base, no gravity compensation, the step of the policy, the exact rotor
  inertia of the small servos) and the core builds the physics with the
  room and the box of the world.
- **Real contacts.** The box is a prop made rigid by the scenario
  (`physics.overrides: { toy: rigid }`): `pose_error(toy)` measures how far
  the duck pushed it.
- **Slower than asked.** With PD actuators instead of the motor model of
  the training, 0.4 m/s gives about 0.14 m/s. `duck.displacement` measures
  the real progress; `duck.distance` also counts the sway of the trunk.

More details in [plugins/MicroDuck/README.md](../plugins/MicroDuck/README.md).

---

## 10. Write a good demo

A good demo answers **one yes/no question**:

```text
Can I detect an object with a camera?
Can I localise a robot with AprilTags?
Can a skill fail cleanly?
Can I re-plan after a fault?
Can I compare two control strategies on identical runs?
```

And it is:

- **small**: one idea per demo;
- **reproducible**: a seed, assertions, a headless run;
- **readable**: YAML first, little C++;
- **observable**: in the simulator, or with `--trace` and reports;
- **testable**: explicit success criteria.

When a demo is stable, it runs both ways —
`Robotik-Simulator x.scenario.yaml` and `Robotik-Headless x.scenario.yaml` — and it
becomes a regression test.

### Ideas for new demos

- **A real SO-101 arm**: the same scenario, only the backend changes.
- **RGB-D perception**: camera → depth → Open3D / OpenCV → WorldModel → pick.
- **Planning**: goal → PDDL / GOAP → behavior tree → skills.
- **Reasoning**: WorldModel → Prolog facts and rules → decision.
- **Strategy comparison**: the same scenario solved by a BT, a planner and a
  policy, compared on time, collisions, failures and robustness to faults.
