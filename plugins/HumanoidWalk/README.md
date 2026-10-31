# HumanoidWalk

A LimX HU_D04_01 humanoid (31 joints) walks in a circle with the walking
policy trained by LimX. No training is needed: the policy (`policy.onnx`)
comes with [humanoid-rl-deploy-cpp](https://github.com/limxdynamics/humanoid-rl-deploy-cpp),
the robot with [humanoid-description](https://github.com/limxdynamics/humanoid-description)
(both Apache-2.0).

## Build

```bash
make PLUGINS+=HumanoidWalk             # fetches its repositories and ONNX Runtime, prepares its data once
./build/linux-gcc-debug/bin/Robotik-Simulator build/linux-gcc-debug/lib/robotik/plugins/HumanoidWalk/scenarios/walk.scenario.yaml   # Space: play
./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/HumanoidWalk/scenarios/walk_circle.scenario.yaml --trace
```

Two scenarios:

- `walk` (simulator): the robot walks until you stop the run. The panel
  **Humanoid** changes the command while it walks (forward, left, turn,
  Stop) and shows the path radius, the turns and the pelvis height.
- `walk_circle` (test): 25 s on a circle of 1 m, judged by assertions.

`prepare_assets.py` writes `data/hu_d04/` (gitignored): the serial URDF
without its floating joint, the meshes, `walk.onnx` and its contract
`walk.yaml` (joint order, PD gains, default posture, scales, armature).
Without ONNX Runtime or this folder, `make` skips the plugin.

## How it works

- The robot uses the **serial** URDF: the ankles and the waist are plain
  joints, as in the training. The MuJoCo file of LimX has the real closed
  linkages and needs their closed-source projection tool.
- `WalkMission` plugs a floating-base MuJoCo backend **without gravity
  compensation**, with the rotor armature, and gives each joint the PD gains
  of the policy with no slew (`JointGains::speed_ratio` = infinity).
- The plugin owns the clock: it steps the simulation every 1 ms, and the
  `Walk` skill runs the policy every 10 steps (100 Hz) on the IMU and the
  joint encoders only. Ground truth only feeds the metrics.
- The physics is MuJoCo: masses and inertias of the URDF, torque limits,
  contacts. The URDF of LimX only collides by the feet; `prepare_assets.py`
  adds the collision primitives of their MuJoCo file, so a falling robot
  lands on its body.
- When the IMU is lost (fault) or the robot lies down, `Walk` fails and
  leaves the joints in damping mode, as the LimX robot does: the robot
  falls. The run ends there, but in the simulator the physics goes on, so
  you see the fall.
- The command (`vx`, `vy`, `yaw_rate`) is in the scenario parameters. The
  circle radius is `vx / yaw_rate`.

Metrics: `humanoid.height_min`, `humanoid.distance`, `humanoid.turns`,
`humanoid.radius`, `humanoid.commanded_radius`.
