# 9. The demos

Each demo is a plugin in `plugins/`. Each one shows **one idea** of robotics,
and each one is also a test: its scenarios have assertions, and they run in
Headless.

| Plugin | Idea | Scenarios | Experiment | Extra program |
|--------|------|-----------|------------|---------------|
| [HelloWorld](#1-helloworld) | Learn the API, one lesson per scenario | hello, wave, reach, skills | `hello_experiment` | — |
| [PickAndPlaceBT](#2-pick-and-place) | Perception + skills + behavior tree + faults | nominal, faults | `pick_and_place_benchmark` | — |
| [Painter](#3-painter) | Free bodies, trajectories in files, "nothing moved" | painter | `painting_benchmark` | — |
| [LineFollower](#4-line-follower) | Mobile robot, localisation, control loop | line_follower | — | `Robotik-LineFollower` |
| [PickAndPlaceRL](#5-pick-and-place-rl) | RL environment, parallel pool | pick_and_place_rl | — | `Robotik-PickAndPlaceRL` |
| [FlyBrain](#6-fly-brain) | A biological brain as an agent | fly_obstacle_avoidance | — | `Robotik-Fly` |

All demos open from the simulator **Demos** menu, and run with:

```bash
./build/Robotik-Headless build/plugins/<Plugin>/scenarios/<scenario>.yml
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

Files: `plugins/PickAndPlaceBT/`
- `worlds/pick_place_cell.yaml`: the arm, its camera, the vacuum cup and four
  stored tools, the cube (randomized), the box;
- `scenarios/pick_and_place.yml`: nominal run;
- `scenarios/pick_and_place_faults.yml`: the camera fails during the run;
- `scenarios/pick_and_place.bt.yml`: the tree (`UntilSuccess` around `Detect`
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
./build/Robotik-Headless build/plugins/PickAndPlaceBT/scenarios/pick_and_place.yml --seed 11
./build/Robotik-Headless build/plugins/PickAndPlaceBT/experiments/pick_and_place_benchmark.yaml \
    --report pick.json
```

The experiment runs three seeds, a wider cube layout, and a camera down at
start.

---

## 3. Painter

An arm dips a brush in four paint pots, paints one stroke per colour on a
canvas standing on an easel, and rinses the brush between colours.

- The easel is a **free body**: pushed, it slides or tips over. The canvas is
  a free box resting on it. Both are entities of
  `plugins/Painter/worlds/painting_lab.yaml`; the mission finds them by id and
  only brings the gestures.
- The strokes are **trajectories** in `scenarios/trajectories/`, written in
  the easel frame.
- Custom tree nodes (`Dip`, `Mark`, `Rinse`) give the colour to the skills.

Zero collision does not prove the easel stayed in place, so the scenario
measures it:

```yaml
  - { condition: "pose_error(easel) < 0.002", phase: always }
  - { condition: "pose_error(canvas) < 0.005", phase: always }
  - { condition: "on(canvas, easel)", phase: end }
  - name: brush loaded before the first stroke
    sequence: [ "skill_succeeded(dip)", "skill_started(mark)" ]
```

The experiment `painting_benchmark` paints again on a slippery floor and with
a canvas four times heavier.

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
./build/Robotik-Headless build/plugins/LineFollower/scenarios/line_follower.yml --seed 4
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

The plugin uses `host.clock: plugin` (its `update` advances the simulation)
and reuses `PickPlaceControl.cpp` of the BT plugin. Details in
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
./build/Robotik-Headless build/plugins/FlyBrain/scenarios/fly_obstacle_avoidance.yml
```

In the simulator, the **Fly brain** panel shows the state and switches to the
FlyWire connectome.

---

## 7. Write a good demo

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
`Robotik-Simulator scenario.yml` and `Robotik-Headless scenario.yml` — and it
becomes a regression test.

### Ideas for new demos

- **A real SO-101 arm**: the same scenario, only the backend changes.
- **RGB-D perception**: camera → depth → Open3D / OpenCV → WorldModel → pick.
- **Planning**: goal → PDDL / GOAP → behavior tree → skills.
- **Reasoning**: WorldModel → Prolog facts and rules → decision.
- **Strategy comparison**: the same scenario solved by a BT, a planner and a
  policy, compared on time, collisions, failures and robustness to faults.
