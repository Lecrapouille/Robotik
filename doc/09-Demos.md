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
`perception: { oracle: when_blind }`, so the true cube position replaces the
camera, and reports say `oracle perception`. The simulator renders the image
and uses the colour detector.

The fault scenario does not only check that things fail: it checks that the
architecture **reacts** (retry, fall back, abort).

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
| `conveyor` | A conveyor brings four parts from a feeder (out of reach) to two pick zones; each arm takes the parts of its zone and drops them in its bin. The belt brings the next part while an arm carries the last one |
| `conveyor_sequential` | The same plan, step by step (`planner.parallel: false`), to compare |
| `tray` | The left arm lifts a tray with four glasses, carries it to another stand and puts it down; the right arm brings its tool above each glass **while the tray moves**. A temporal domain (durative actions) |

**Conveyor.** One plan of 12 steps; the times show the overlap:

```text
  0 s  Convey(p4, zone_right)  SUCCEEDED  (2.69 s)
  2.7 s  right.Hover(p4)  SUCCEEDED  (1.08 s)
  2.7 s  Convey(p3, zone_left)  SUCCEEDED  (2.38 s)
  3.79 s  right.Reach(p4)  SUCCEEDED  (0.43 s)
  4.23 s  right.Grasp(p4)  SUCCEEDED  (0.01 s)
  4.25 s  right.Hover(p4)  SUCCEEDED  (0.45 s)
  4.71 s  right.Over(bin_right)  SUCCEEDED  (1.94 s)
  5.09 s  left.Hover(p3)  SUCCEEDED  (1.08 s)
  5.09 s  Convey(p2, zone_right)  SUCCEEDED  (2.08 s)
  ...
[PASS] dual_arm.overlap > 2  (dual_arm.overlap = 6.25)
[PASS] time < 20  (time = 13.91)
```

Run step by step, the same plan ends at 26.61 s instead of 13.91 s. The
conveyor is a device of the plugin with its resource `conveyor`: the
`Convey(part, zone)` skill holds it alone, so two conveys never overlap, and
a fault on `conveyor` stops the belt.

**Tray.** The plan holds the tray during the visits: `visit` needs the tray
`in_motion` over all its duration, a token `carry` adds at its start and
removes at its end.

```text
  plan 0 at 0 s (start): solved (lift left tray pad_a) (carry left tray pad_a pad_b) (visit right g1 tray) (visit right g2 tray) (visit right g3 tray) (visit right g4 tray) (set_down left tray pad_b)
  4.53 s  left.Carry(pad_b)  SUCCEEDED  (10 s)
  4.53 s  right.Visit(g1, tray)  SUCCEEDED  (3.66 s)
  8.2 s  right.Visit(g2, tray)  SUCCEEDED  (1.85 s)
  10.06 s  right.Visit(g3, tray)  SUCCEEDED  (1.72 s)
  11.79 s  right.Visit(g4, tray)  SUCCEEDED  (1.81 s)
[PASS] on(g1, tray)  (on(g1, tray) = 1)
...
[PASS] collision_count(left) == 0  (collision_count(left) = 0)
[PASS] collision_count(right) == 0  (collision_count(right) = 0)
```

How it works:

- **Synchronized arms.** `Visit(glass, tray)` keeps its tool above the glass
  **in the frame of the tool of the carrying arm**, read through
  `RobotContext::team` (what that arm measures of itself), not through the
  camera.
- **Real glasses.** The cup is at the `rigid` physics level, so the tray and
  the glasses are free bodies of MuJoCo; `on(g1, tray)` with `phase: always`
  is a real contact at every step. Dropped instead of put down, the glasses
  fall and these assertions fail.
- **Gentle moves.** The tray moves along straight lines with a smooth
  profile (`PathSkill`), and the speed limits of the joints keep it slow.
  The arms have stiffer joint gains than the default: the gravity
  compensation of the drives does not know the 0.7 kg of the tray.
- **No crossing.** The visiting tool comes from 20 cm above the glass, then
  goes down: from far, the joints would cut through the other arm.

Files: `plugins/DualArm/`
- `worlds/conveyor_cell.world.yaml`, `worlds/tray_cell.world.yaml`;
- `scenarios/conveyor.domain.pddl` (classical), `scenarios/tray.domain.pddl` (temporal);
- `src/ConveyorMission.cpp`, `src/TrayMission.cpp`: skills of each robot,
  observers, bindings; `src/Conveyor.cpp`: the belt; `src/ArmSkills.cpp`,
  `src/TraySkills.cpp`: the skills.

Limits: the parts and glasses are boxes (the only free shape); the parts
are found by the oracle (`perception: { oracle: always }`), as the plugin
has no detector yet; the arms are the generic `robot_6axis`.

---

## 9. Write a good demo

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
