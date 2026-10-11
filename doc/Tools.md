# Tools

A robot URDF and a tool URDF stay separate files. Nothing is concatenated into a third URDF. Each file is one kinematic chain. `MujocoBackend::load` reads a chain, `attach(robot, link, tool, "tool_mount")` hangs the tool on the robot's `flange` link, and `detach` puts the tool back in the world as its own chain. A link name or a joint name both work: a joint names the link it moves.

## Frames

| Name | What it is | Where it lives |
|------|------------|----------------|
| `flange` | Mounting face of the robot, fixed by the manufacturer. ISO 9787: origin at the centre of the face, z pointing outwards. | Robot file |
| `tool0` | ROS-Industrial name of the flange frame. It coincides with `flange`, which means no tool is mounted. | Robot file |
| `tool_mount` | Root link of the tool. Its frame is placed on `flange` when the tool is attached. | Tool file |
| `tcp` | Tool Center Point: drill tip, midpoint of the gripper fingers, centre of the suction face. | Tool file |
| End effector, tool | Roles, not frames. The tool is the set of links. The end effector is the TCP. | Nowhere as a frame name |

`robot_6axis.urdf` publishes `flange` and `tool0`. Inverse kinematics uses `tcp` when a tool is mounted, and `tool0` otherwise. The TCP is the working point of the tool. It is not the mount: the mount is `flange` against `tool_mount`.

## World file

A tool is an entity of the world, with its own identity. The mounted one hangs from `robot.flange` by its `tool_mount` link; it brings its own actuators (and sensors). Tools declared without `attach` are in storage: they are not built, but the simulator can mount one instead.

```yaml
world:
  id: pick_place_cell
  entities:
    - id: robot
      type: robot
      model: robotik.robots.robot_6axis
      actuators:
        arm: { type: joint_group }
    - id: vacuum
      type: tool
      model: robotik.tools.vacuum
      attach: { parent: robot.flange, frame: tool_mount }
      actuators:
        gripper: { type: vacuum }
    - { id: plain, type: tool, model: robotik.tools.default }
    - { id: drill, type: tool, model: robotik.tools.drill }
    - { id: fingers, type: tool, model: robotik.tools.gripper }
    - { id: welder, type: tool, model: robotik.tools.welding }
```

Models are asset ids: the tools live in the pack `data/packs/robotik.tools`. The fixed joint between `flange` and `tool_mount` is the identity. Mounting the drill removes the vacuum and its `gripper` actuator: nothing sticks to the flange any more. A scenario checks the assembly with `tool_is_attached(vacuum, robot.flange)`.

Every tool uses the same interface:

- The root link is `tool_mount`.
- The work point is `tcp`.
- Internal joints and links keep a prefix, for example `gripper_finger_left_joint` and `drill_spindle_joint`.

`tool_mount` and `tcp` are not prefixed. One tool is mounted at a time, so those names do not collide.

The simulator **Teach** panel lists the tool entities of the world and reloads the scenario with the chosen one. **None** leaves `tool0` as the work frame.

## Checks at zero joints

With every axis at zero, the TCP height is 1.27 m for the flat pad (`tool_default`), 1.342 m for the gripper and 1.422 m for the drill. Those values match the stacked link lengths.

Tool masses and inertias are plausible estimates. They are enough for dynamics and should be replaced when a real tool is measured.
