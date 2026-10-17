---
name: robotik-scenario
description: Write or fix RobotIK world (worlds/*.yaml), scenario (scenarios/*.yml) and experiment (experiments/*.yaml) files — entities, sensors, actuators, faults, assertions, trajectories, seeds, overrides. Use for any YAML under plugins/*/worlds|scenarios|experiments.
---

# RobotIK YAML

World = what exists. Scenario = what happens, when it stops, what is
checked. Experiment = variants × seeds. Paths are relative to the package root (folder of plugin.yaml); `..` and symlinks are allowed but the canonical path must stay in the package; absolute paths are refused.
Scenario/experiment files must be listed in the plugin `plugin.yaml`.
Edit `plugins/...`, then `make -C plugins/<Name>` (it overwrites `build/`).

## World

```yaml
world:
  id: cell
  floor: { friction: 0.8 }            # needed for free bodies (else they fall forever)
  entities:
    - id: robot                        # exactly one robot
      type: robot
      model: robotik.robots.robot_6axis    # <pack>.<key> from data/packs/*/pack.yaml
      pose: { position: [0, 0, 0], rpy: [0, 0, 0] }
      home: { joint1: 0.0, joint2: 0.3 }
      sensors:
        cam: { type: camera, parent: link6, position: [0.05, 0, 0], fov: 70,
               resolution: [320, 240], frequency: 15, noise: 0.01 }
        imu: { type: imu, parent: base_link, frequency: 200, latency: 0.002,
               accelerometer: { sigma: 0.02, bias: 0.05, saturation: [-160, 160] } }
        lidar: { type: range, parent: base_link, beams: 180, fov: 360, max_range: 10 }
        ft: { type: force_torque, link: link6, force: 0.5, torque: 0.02 }
      actuators:
        arm: { type: joint_group }                  # all joints if none listed
        wheel: { type: motor, joint: left_wheel_joint }
    - id: vacuum                       # mounted tool (one at a time)
      type: tool
      model: robotik.tools.vacuum
      attach: { parent: robot.flange, frame: tool_mount }
      actuators: { gripper: { type: vacuum } }
    - { id: drill, type: tool, model: robotik.tools.drill }   # stored (no attach)
    - id: cube                         # prop: kinematic, exact, moves only when grasped
      type: prop
      shape: { type: box, size: [0.04, 0.04, 0.04] }   # pose = centre
      color: [0.9, 0.1, 0.1]
      pose: { position: [0.40, 0.20, 0.02] }
      randomize: { x: [-0.02, 0.02], y: [-0.02, 0.02] }   # true pose shifted per seed
    - { id: box, type: prop, shape: { type: container, size: [0.16, 0.16, 0.08] },
        pose: { position: [0.40, -0.20, 0.04] } }        # wall/floor 5 mm
    - id: easel                        # body: physics (free|fixed); URDF model or box+mass
      type: body
      model: robotik.painter.easel
      physics: free
      friction: 0.6
    - { id: canvas, type: body, shape: { type: box, size: [0.02, 0.4, 0.5] }, mass: 0.5,
        physics: free, attach: { parent: easel.canvas_mount, transform: { position: [0.011, 0, 0.251] } } }
```

Each sensor/actuator name is also a resource (skills reserve it, faults
disable it). Noise: a sigma, or `{sigma, bias, resolution, saturation}`.
A free `body` settles at start (initial pose taken after settling) and may
drift ~1 cm: prefer `prop` for exact tutorials/tests.

## Scenario

```yaml
scenario:
  id: main                             # = plugin.yaml entry id = Plugin::setup string
  description: ...
  world: worlds/cell.yaml
  seed: 42
  controller: { behavior_tree: scenarios/main.bt.yml }   # omit → Mission::status ends the run
  # or instead of a tree (doc/11-PDDL-Planner.md); entities need `kind: <pddl type>`:
  # controller:
  #   planner:
  #     domain: scenarios/x.domain.pddl
  #     facts: [...]                  # block list! [a(x, y)] splits at the comma
  #     goal:
  #       - in(red_cube, box)
  #     replan: { max: 3, delay: 0.5 }   # verify: true, search: lama, timeout: 10
  trajectories: { red: scenarios/trajectories/red.yaml }
  perception: { oracle: when_blind }   # never (default) | when_blind | always
  faults:
    - { at: 1.0, resource: arm, action: disable }
    - { at: 2.0, resource: arm, action: restore }
    - { resource: gripper, rate: 0.01 }          # random, per second
  timeout: 60
  stop_when: inside(cube, box)
  assertions: [...]
```

Run ends on: controller success/failure, `stop_when`, `timeout`, failed
assertion with `stop: true`.

## Assertions

```yaml
  - controller.success                                   # phase end (default)
  - { condition: "collision_count(robot) == 0", phase: always }
  - { condition: "reach.error < 0.02", within: 5.0 }
  - { condition: "holding(gripper)", eventually: true }
  - { condition: "time > 1", at: 2.0 }
  - { name: order, sequence: [ "skill_succeeded(Home)", "skill_started(Wave)" ] }
  - { condition: "joint_limits_respected(robot)", phase: always, stop: true }
```

Grammar: `<value> <op> <number>` (`== != < <= > >=`) or a value alone (≠ 0).
Values: `time`, `controller.success`, mission metrics, `pose_error(e)` (m),
`orientation_error(e)` (rad), `distance(a, b)` (entities or `robot.tool0`,
`robot.tcp` only with a tool, `entity.link`), `inside(a, b)` (rests on
container floor ±1 mm), `on(a, b)`, `touching(a, b)`, `collision_count(e)`,
`tool_is_attached(t, robot.flange)`, `joint_limits_respected(robot)`,
`holding(g[, obj])`, `skill_started|succeeded|failed(name)`.
Unknown value/entity → the assertion fails with the reason.

## Trajectory file

```yaml
trajectory:
  type: cartesian            # or joint (+ joints: [...])
  frame: easel.canvas_mount  # world if empty
  orientation: { rpy: [1.5708, 0, -1.5708] }
  points: [ [0.18, 0.12, 0.16], { position: [0.06, 0.08, 0.16], rpy: [1.5708, 0, -1.5708] } ]
```

## Experiment

```yaml
experiment:
  id: bench
  seeds: [1, 2, 3]
  world: worlds/other.yaml          # optional: replaces every world
  cases:
    - { id: nominal, scenario: scenarios/main.yml }
    - id: variant
      scenario: scenarios/main.yml
      controller: { behavior_tree: scenarios/retry.bt.yml }
      overrides:                       # dotted paths, list segment = element id
        world.entities.cube.randomize: { x: [-0.05, 0.05] }
        scenario.timeout: 90
      setup:
        - inject_fault: { resource: cam, at: 0.0, action: disable }
  evaluation:
    assertions: [ { condition: "joint_limits_respected(robot)", phase: always } ]
    metrics: [ time, collision_count(robot) ]
```

## Verify

```bash
./build/Robotik-Headless build/plugins/<Name>/scenarios/<s>.yml --trace
./build/Robotik-Headless build/plugins/<Name>/experiments/<e>.yaml --report r.json
```
