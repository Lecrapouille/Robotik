# MicroDuck

A Pollen Robotics Microduck (14 servos, about 25 cm) walks in a small room
with the official `velstand` policy: no training is needed. It pushes a toy
box, tours the room, and follows the panel. The body comes from
[microduck_rl](https://github.com/pollen-robotics/microduck_rl) (branch
`develop`, Apache-2.0), the policy from
[microduck-policies](https://huggingface.co/pollen-robotics/microduck-policies)
(Apache-2.0). The STL meshes are **CC BY-NC-SA 4.0**: they are not
committed, `plugin.yaml` credits them.

## Build and run

```bash
make -C plugins/MicroDuck download-external-libs   # microduck_rl + ONNX Runtime release
make -C plugins/MicroDuck -j"$(nproc)"             # runs prepare_assets.py once, then builds
./build/Robotik-Headless build/plugins/MicroDuck/scenarios/tour.scenario.yaml --trace
./build/Robotik-Simulator build/plugins/MicroDuck/scenarios/drive.scenario.yaml   # Space: play
```

Without ONNX Runtime or the clone, `make` skips the plugin.

| Scenario | World | What happens |
|----------|-------|--------------|
| `walk_forward` | `flat` | Walk straight for 8 s; the trunk moves more than 0.8 m |
| `tour` | `playground` | Push the toy box, turn left, walk, turn left, walk back (behavior tree) |
| `drive` | `playground` | The panel **Micro Duck** gives the command (forward, left, turn, Stop) until you stop the run |

## The files

```text
data/microduck/     the duck: URDF, meshes, walk.onnx, walk.yaml, pack.yaml
                    (made by prepare_assets.py, gitignored)
data/playground/    the room: one URDF (walls, sofa, pouf, shelf, rug), pack.yaml
worlds/             flat.world.yaml, playground.world.yaml (written by hand)
scenarios/          walk_forward, tour, drive and their behavior trees
src/                DuckPolicy (ONNX), DuckSkills (Stand, Walk), DuckMission, Plugin
```

The room is one fixed body of MuJoCo: the duck bumps into its walls and
furniture for real. The toy box is a prop that the scenario makes rigid
(`physics.overrides: { toy: rigid }`): a free body of 80 g that the duck
pushes. The rug is drawn only.

## How it works

- **Physics.** `DuckMission::setup` tunes `Simulation::physicsOptions()`: a
  floating base, no gravity compensation, the 5 ms step of the training,
  and `exact_inertia` to keep the rotor armature (0.0018) and the damping
  (0.053) of the small servos; the default floors (0.1 and 0.5) would make
  the legs unusable. The core then builds the physics with the room and
  the box, as for any other world.
- **Clock.** The plugin owns the clock: it steps the simulation every 5 ms,
  and a walk skill runs the policy every 4 steps (50 Hz) on the IMU and the
  joint encoders only. Ground truth only feeds the metrics.
- **Policy.** Observation 61, action 14, in the order of the official
  policies: gyro, projected gravity, joint position minus home, joint
  velocity, previous action, then the command (forward, left, yaw, and
  zeros for the head and the trunk pose). Target of a joint: action + home.
- **Skills.** `Stand` holds the home posture for one IMU sample (longer, the
  duck sits down). `Walk` runs the policy with the command of the panel;
  `Forward` and `TurnLeft` with a fixed command (scenario parameters
  `forward_speed`, `forward_duration`, `turn_rate`, `turn_duration`). The
  policy keeps its last action from one walk to the next.
- **A fall.** A tilt does not fail a walk: this policy is trained to stand
  back up. A walk fails only when the IMU is lost, and the joints then go
  to damping mode.

## PD actuators

The policy was trained with the BAM motor model (XL330, voltage, friction).
This plugin uses the PD gains of the MuJoCo file (`kp` 0.55, `kd` 0, joint
damping 0.053, torque limit 0.96 N·m), the same fallback as
`infer_policy.py --no-bam`. The duck then walks slower than asked: 0.4 m/s
gives about 0.14 m/s, 0.25 m/s about 0.09 m/s, and 0.15 m/s only holds the
stand. A BAM actuator would be the closer match to training.

Metrics: `duck.height_min` (m), `duck.distance` (m, path length in the
plane, sway included), `duck.displacement` (m, from the start to now).

The official policy set can also sit, pick an object from the floor, roll,
kick a ball and skate. This plugin only loads the walk (`velstand`).
