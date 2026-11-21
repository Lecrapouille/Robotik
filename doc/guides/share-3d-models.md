# 📦 Share 3D models in an asset pack

[⏫ Guides](README.md)

A robot, tool or object model is not copied into each plugin. It lives in a
**pack**: a folder with a `pack.yaml`, with a version.

```yaml
# data/packs/robotik.tools/pack.yaml
id: robotik.tools
version: 1.0.0
license: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
source: https://github.com/Lecrapouille/RobotIK
models:
  vacuum: tool_vacuum.urdf
  gripper: tool_gripper.urdf
```

A YAML file names a model `<pack id>.<key>`: `robotik.tools.vacuum`,
`robotik.robots.robot_6axis`. `AssetResolver` finds the real file; a plugin
never writes a path. The longest pack id wins: `robotik.robots.abb.irb2400`
is in the pack `robotik.robots.abb`.

| Level | Where | Example |
|-------|-------|---------|
| Shared pack | `data/packs/<id>/` | `robotik.robots`, `robotik.tools`, `robotik.robots.abb` |
| Pack of a plugin | `plugins/<Name>/data/` (`resources.directory: data`) | `robotik.painter` (the easel) |

Search order: `$ROBOTIK_DATA`, `data/packs` next to the executable folder,
`share/robotik/packs` of the install prefix, then the pack of each plugin.
The first one wins; another pack with the same id and another version is
reported as a conflict.

`license` is an SPDX expression. `NOASSERTION` means nobody checked the
license yet: this is the case of the ABB meshes. Check before you
redistribute them. The Assets panel of the simulator shows every pack with
its license.
