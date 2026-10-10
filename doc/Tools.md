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

## Scenario

`robot.tools` names the URDF of each tool. An actuator whose `type` is one of those names mounts that tool. `type: vacuum` is the suction gripper and the key `vacuum`.

```yaml
robot:
  model: robot_6axis.urdf
  tools:
    default: tool_default.urdf
    drill: tool_drill.urdf
    gripper: tool_gripper.urdf
    vacuum: tool_vacuum.urdf
    welding: tool_welding.urdf
  actuators:
    arm: { type: joint_group }
    gripper: { type: vacuum }
```

Bare names are resolved from `data/`, like robot models. The fixed joint between `flange` and `tool_mount` is the identity.

Every tool uses the same interface:

- The root link is `tool_mount`.
- The work point is `tcp`.
- Internal joints and links keep a prefix, for example `gripper_finger_left_joint` and `drill_spindle_joint`.

`tool_mount` and `tcp` are not prefixed. One tool is mounted at a time, so those names do not collide.

The simulator **Teach** panel lists the keys of `robot.tools` and reloads the scenario with that tool. **None** leaves `tool0` as the work frame.

## Checks at zero joints

With every axis at zero, the TCP height is 1.27 m for the flat pad (`tool_default`), 1.342 m for the gripper and 1.422 m for the drill. Those values match the stacked link lengths.

Tool masses and inertias are plausible estimates. They are enough for dynamics and should be replaced when a real tool is measured.
