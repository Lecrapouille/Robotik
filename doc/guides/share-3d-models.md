# 📦 Share 3D models in an asset pack

[⏫ Guides](README.md)

A robot, a tool or an object is described by a model file (a URDF file). Several plugins often use the same model: the 6-axis arm is in almost all the demos. If each plugin keeps its own copy, the copies become different, and nobody knows which one is right. RobotIK keeps each model once, in an asset pack: a folder with a list of its models, a version and a license. In this guide you make a pack for your plugin, use one of its models in a world, and read the errors.

## 📖 The words

- An asset is a file that a project reuses: a robot model, a tool, a mesh.
- An asset pack is a folder with a `pack.yaml` file. This file gives the id of the pack, its version, its license, and the list of its models.
- A model id is `<pack id>.<model key>`, for example `robotik.robots.robot_6axis`. A world file names a model by its id, never by a path. The asset resolver (`AssetResolver`) finds the file.

## 📁 1. Choose where the pack goes

| Where | When | Example |
|-------|------|---------|
| `data/packs/<id>/` | several plugins use the models | `robotik.robots`, `robotik.tools`, `robotik.robots.abb` |
| `plugins/<Name>/data/` | only this plugin uses them | `robotik.painter` (the easel of the Painter) |

This guide makes a pack in the folder of a plugin `MyRobot`. The steps are the same for a shared pack, without step 3.

## 📄 2. Write the pack

Put the URDF file in `plugins/MyRobot/data/`, then write `plugins/MyRobot/data/pack.yaml`:

```yaml
id: my.robots
version: 1.0.0
license: GPL-3.0-or-later
source: https://example.org/my-robots
description: The arm of my plugin.
models:
  arm: my_arm.urdf
```

The keys are `id`, `version`, `license`, `source`, `description` and `models`. Each entry of `models` gives a key and a file, relative to the folder of the pack. The `license` is an SPDX expression: it tells who can copy the models. Write `NOASSERTION` when nobody checked it yet (the ABB meshes are in this case). Check the license before you give the files to other people.

## 🔌 3. Declare the pack in the plugin

In `plugins/MyRobot/plugin.yaml`, `resources.directory` gives the folder of the pack of the plugin. `resources.packs` lists the shared packs that the plugin needs, with a version:

```yaml
resources:
  directory: data
  packs:
    - { id: robotik.robots, version: "1.0" }
```

`make` copies `data/` with the other folders of the plugin.

## 🌍 4. Use the model

In the world file, name the model by its id:

```yaml
    - id: robot
      type: robot
      model: my.robots.arm
```

Run a scenario of this world. If the run starts, the resolver found the model.

## 🐞 5. Read the errors

If the key of the model is wrong (`my.robots.arms`), the message names the pack and the key:

```text
.../cell.world.yaml: world 'cell', entity 'robot': Pack 'my.robots' has no model 'arms'
```

If `pack.yaml` has an error, the pack is not loaded. The message gives the model that is missing, then the error of each pack that could not be read:

```text
.../cell.world.yaml: world 'cell', entity 'robot': No asset pack provides 'my.robots.arm'; .../data/pack.yaml: unknown key 'licence' (did you mean 'license'?)
```

In the simulator, the **Assets** panel lists every pack with its version, its license and its errors.

## 🔎 How the resolver chooses

The resolver looks for packs in this order:

1. the folder in `$ROBOTIK_DATA`;
2. `data/packs` next to the folder of the executable;
3. `share/robotik/packs` of the install prefix;
4. the pack of each plugin (`resources.directory`).

The first pack with an id wins. If another folder has a pack with the same id and another version, the resolver reports a conflict.

When two pack ids fit a model id, the longest one wins: `robotik.robots.abb.irb2400` is in the pack `robotik.robots.abb`, not in `robotik.robots`.

## 🔗 See also

- [Mount a tool on a robot](mount-a-tool.md): the tools of the pack `robotik.tools`.
- [Plugin reference](../reference/plugin.md): all the keys of `plugin.yaml`.
