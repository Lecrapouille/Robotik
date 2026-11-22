# 🌍 The world file

[⏫ Reference](README.md)

## 🧊 Entities

The world is a list of **entities**. Each has an `id`, a `type`, a 3D model or a primitive shape, and a pose.

```yaml
world:
  id: painting_lab
  floor: { friction: 0.8 }             # a physical floor for free bodies

  entities:
    - id: robot
      type: robot
      model: robotik.robots.robot_6axis
      home: { joint1: 0.0, joint2: 0.3, joint3: 1.3, joint4: 0.0, joint5: 1.54, joint6: 0.0 }
      actuators:
        arm: { type: joint_group }

    - id: brush
      type: tool
      model: robotik.tools.default
      attach: { parent: robot.flange, frame: tool_mount }

    - id: easel
      type: body
      model: robotik.painter.easel
      physics: free
      friction: 0.6
      pose: { position: [0.65, 0.0, 0.0], rpy: [0.0, 0.0, 3.141592653589793] }

    - id: canvas
      type: body
      shape: { type: box, size: [0.02, 0.40, 0.50] }
      mass: 0.5
      physics: free
      attach:
        parent: easel.canvas_mount
        transform: { position: [0.011, 0.0, 0.251] }
```

| Type | What it is | Physics |
|------|------------|---------|
| `robot` | A robot you control (one or more, see [Several robots](#-several-robots)). Keys: `home`, `sensors`, `actuators` | Its chain, fixed at its `pose` |
| `tool` | A tool. It keeps its identity once mounted and brings its own actuators and sensors | Hangs on the flange |
| `body` | A simulated rigid body: a URDF model (`physics: free` or `fixed`) or a box (`free`, with `mass`) | Yes: gravity, contacts, friction |
| `prop` | A kinematic object: it does not move, unless the robot grasps it. Shape `box` or `container` (open box) | No |

Use a `prop` when you want a simple, exact scene. Use a `body` when physics matters: "does the easel slide when the brush pushes it?".

Any entity may also have a `kind`: its type for the PDDL planner (`kind: cube`). Only entities with a `kind` are objects of the planning problem ([The PDDL planner](../concepts/05-pddl-planner.md#-what-goes-where)).

## 🏷 Models are asset ids

`model: robotik.robots.robot_6axis` is not a path. It is `<pack>.<model>`: the file is found in an **asset pack** (`data/packs/robotik.robots/pack.yaml`). See [Share 3D models in an asset pack](../guides/share-3d-models.md).

## 📎 Attach an entity to another

`attach.parent` names an entity, or one of its links: `easel.canvas_mount` is the link `canvas_mount` of the easel URDF. `transform` places the entity in that frame. For a free body, this is only the start pose: physics decides after that.

The URDF describes a *model*. The world describes an *instance*: where it is, and on what it depends.

## 🔧 Tools

A tool is always mounted by its link `tool_mount` on the robot flange (`robot.flange`). Only one tool is mounted on a robot at a time. Tools declared without `attach` are **in storage**: not built, but the simulator **Teach Pendant** panel can mount one instead. Details in [Tools and frames](../concepts/03-tools-and-frames.md).

```yaml
    - id: vacuum
      type: tool
      model: robotik.tools.vacuum
      attach: { parent: robot.flange, frame: tool_mount }
      actuators:
        gripper: { type: vacuum }
    - { id: drill, type: tool, model: robotik.tools.drill }    # in storage
```

A tool that works **by contact** (a drill on its workpiece, a probe) lists what it may touch. Its contacts with these entities are its work, not collisions (`collision_count` does not count them); any other contact still counts. From the DualArm tray cell:

```yaml
    - id: right_drill
      type: tool
      model: robotik.tools.drill
      attach: { parent: right.flange, frame: tool_mount }
      # The drill works on the tray: touching it is not a collision.
      touches: [tray]
```

Only a tool has `touches`, and each name must be another entity of the world; otherwise the world is refused.

## 👥 Several robots

A world may have several entities of type `robot`: two arms of a cell, an arm and a mobile base. They share **one** physics (MuJoCo), **one** resource manager, **one** skill scheduler and **the beliefs**. From the DualArm tray cell (`plugins/DualArm/worlds/tray_cell.world.yaml`):

```yaml
    - id: left
      type: robot
      kind: arm
      model: robotik.robots.robot_6axis
      ...
      actuators:
        left_arm: { type: joint_group, joints: [joint1, joint2, joint3, joint4, joint5, joint6] }
    - id: left_cup
      type: tool
      model: robotik.tools.vacuum
      attach: { parent: left.flange, frame: tool_mount }
      actuators:
        left_gripper: { type: vacuum }

    - id: right
      type: robot
      kind: arm
      model: robotik.robots.robot_6axis
      pose: { position: [1.02, 0.0, 0.55], rpy: [0.0, 0.0, 3.141592653589793] }
      ...
```

Rules:

- Each robot has its own tool: `attach: { parent: <robot>.flange, frame: tool_mount }`.
- The names of the sensors and actuators are **unique in the world** (`left_arm`, `right_arm`): they are the names of the resources, and the robots share the resource manager. A second `gripper` is refused when the world is loaded.
- Each robot keeps the names of its own URDF: both arms have a `joint1`. In the same way, two `body` entities may use the same model (the two stands of the tray cell).
- A `joint_group` without `joints` takes every joint of the robot, the joints of its tool too (the spindle of a drill). List the arm joints when the tool has joints of its own.
- The first robot of the file is `RoboticCell::robot()`. The others are `RoboticCell::robot("right")`; all of them are `RoboticCell::robots()`.
- The skills of the core are named after the robot: `left.Home`, `right.Stop`. With one robot they stay `Home` and `Stop`.
- The queries take the robot id: `pose(right.tcp)`, `(collision_count left)`, `(joint_limits_respected right)`. A contact with the other robot is a collision.
- The shared beliefs are in the **world frame** (with one robot, they stay in its base frame: see [Truth, description and belief](../concepts/02-worlds-scenarios-experiments.md#-truth-description-and-belief)).

How a skill drives one robot of the cell: [Several robots](../concepts/04-skills-and-behavior-trees.md#-several-robots).

## 📡 Sensors and actuators

Each sensor and each actuator is also a **resource** with the same name: skills reserve it, faults make it unavailable.

```yaml
      sensors:
        wrist_camera: { type: camera, parent: link6, position: [0.05, 0.0, 0.0],
                        fov: 70, resolution: [320, 240], frequency: 15, noise: 0.01 }
        imu:
          type: imu
          parent: base_link
          frequency: 200
          latency: 0.002
          accelerometer: { sigma: 0.02, bias: 0.05, saturation: [-160, 160] }
          gyroscope: 0.001
        lidar: { type: range, parent: base_link, beams: 180, fov: 360, max_range: 10,
                 noise: { sigma: 0.01, resolution: 0.001 } }
        wrist_ft: { type: force_torque, link: link6, force: 0.5, torque: 0.02 }
      actuators:
        arm: { type: joint_group }                        # no joints listed: all of them
        left_wheel: { type: motor, joint: left_wheel_joint }
        gripper: { type: vacuum }
```

| Sensor | Measures | Frame of the data |
|--------|----------|-------------------|
| `camera`, `rgbd` | an image (and depth) | optical: z forward, x right, y down |
| `imu` | specific force (+9.81 up at rest), angular velocity, attitude | the sensor |
| `range` | one distance per beam; `max_range` when nothing is hit | XY plane of the sensor |
| `force_torque` | force and torque received by the link from its parent | the measured link |

| Actuator | Commands |
|----------|----------|
| `joint_group` | a set of joints (all of them by default) |
| `motor` | one joint |
| `vacuum` | a vacuum gripper: grasps what touches it |

**Noise.** A noise channel is a standard deviation (`noise: 0.01`) or a full model `{ sigma, bias, resolution, saturation: [min, max] }`, applied in this order. `latency` delays the delivery of a measure; the measure keeps its capture time. Every reading has a common header: time stamp, frame, sequence number, validity (false while the resource is in fault).
