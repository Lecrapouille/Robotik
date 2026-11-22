# 📐 Tools and frames

[⏫ Concepts](README.md)

This page explains how a tool (vacuum cup, gripper, drill, brush) is mounted on a robot, and the names of the frames involved. Read [The tool and the TCP](01-robotics-in-ten-minutes.md#-the-tool-and-the-tcp) first if *flange* and *TCP* are new words.

## 📄 Two files, never merged

A robot URDF and a tool URDF stay **separate files**. Nothing is concatenated into a third URDF. Each file is one kinematic chain:

- `MujocoBackend::load` reads a chain;
- `attach(robot, link, tool, "tool_mount")` hangs the tool on the robot `flange` link;
- `detach` puts the tool back in the world as its own chain.

A link name or a joint name both work: a joint names the link it moves.

## 🧭 Frames

| Name | What it is | Where it lives |
|------|------------|----------------|
| `flange` | Mounting face of the robot, fixed by the manufacturer. ISO 9787: origin at the centre of the face, z pointing outwards. | Robot file |
| `tool0` | ROS-Industrial name of the flange frame. It coincides with `flange`, which means no tool is mounted. | Robot file |
| `tool_mount` | Root link of the tool. Its frame is placed on `flange` when the tool is attached. | Tool file |
| `tcp` | Tool Center Point: drill tip, midpoint of the gripper fingers, centre of the suction face. | Tool file |
| End effector, tool | Roles, not frames. The tool is the set of links. The end effector is the TCP. | Nowhere as a frame name |

`robot_6axis.urdf` publishes `flange` and `tool0`. Inverse kinematics uses `tcp` when a tool is mounted, and `tool0` otherwise. The TCP is the working point of the tool. It is not the mount: the mount is `flange` against `tool_mount`.
