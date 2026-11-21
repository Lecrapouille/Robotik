# 🧩 Your own plugin

[⏫ Tutorials](README.md)

Let us create `MyRobot`, from HelloWorld.

**Step 1. Copy.**

```bash
cp -r plugins/HelloWorld plugins/MyRobot
mv plugins/MyRobot/HelloWorld.mk plugins/MyRobot/MyRobot.mk
rm -rf plugins/MyRobot/experiments
```

**Step 2. Rename in four places.** The names must match, or the host refuses
the package.

| File | Change |
|------|--------|
| `plugins/MyRobot/MyRobot.mk` | the target `robotik_my_robot` (in `TARGETS`, its variables, and `$(call ROBOTIK_PLUGIN,robotik_my_robot,MyRobot)`), its `DESCRIPTION` |
| `plugins/MyRobot/plugin.yaml` | `plugin.id: robotik.my_robot`, `plugin.name`, `library.entrypoint: librobotik_my_robot.so`, the scenario list |
| `plugins/MyRobot/src/Plugin.cpp` | `describe()` returns `{ .id = "robotik.my_robot" }`; rename the class |
| `plugins/MyRobot/scenarios/*.scenario.yaml` | Keep only what you need; the `id` of each file must match an entry of `plugin.yaml` |

The rules:

```text
MyRobot.mk  TARGETS += robotik_my_robot
                 │
                 └──► produces librobotik_my_robot.so
                                    │
plugin.yaml  library.entrypoint: librobotik_my_robot.so      (same file name)
plugin.yaml  plugin.id: robotik.my_robot
                                    │
Plugin.cpp   describe() { .id = "robotik.my_robot" }          (same id)
plugin.yaml  scenarios.entries[].id: patrol
                                    │
Plugin.cpp   { "patrol", robotik::mission<PatrolMission>() }  (same scenario id)
```

**Step 3. Keep one scenario and one mission.** For example, delete the
lessons you do not need from `src/`. `MyRobot.mk` compiles every `src/*.cpp`; to list them instead:

```make
robotik_my_robot.SOURCES := src/Plugin.cpp src/PatrolMission.cpp
```

**Step 4. Build and run.**

```bash
make robotik_my_robot
./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/MyRobot/scenarios/patrol.scenario.yaml
./build/linux-gcc-debug/bin/Robotik-Simulator          # Demos → My robot → ...
```

Add `MyRobot` to `PLUGINS` in the root `Makefile` (or `make PLUGINS+=MyRobot`),
so that its `MyRobot.mk` is read. `make` compiles the `.so` and copies `plugin.yaml`, the
library, and the folders `worlds/`, `scenarios/`, `experiments/` and `data/`
into `build/linux-gcc-debug/lib/robotik/plugins/MyRobot/`.

**Step 5. Use another robot.** Change the `model` of the robot in your world
file. Available models are listed in the `pack.yaml` files of `data/packs/`:

```bash
cat data/packs/*/pack.yaml
```

## 🐞 Common errors

| Message | Cause |
|---------|-------|
| `scenario is not in a plugin package` | The file is not listed in `plugin.yaml`, or you ran a file outside `build/linux-gcc-debug/lib/robotik/plugins/` and `plugins/` |
| `unknown scenario 'X' (this plugin runs: ...)` | The mission table of `Plugin.cpp` has no entry `X` |
| `plugin id does not match plugin.yaml` | `describe().id` differs from `plugin.id` |
| `built for plugin API 99 ...` | `api_version` in `plugin.yaml` differs from the host ABI (`1`) |
| `plugin library not found: ...` | `library.entrypoint` differs from the `.so` built by the target (`lib` + its name + `.so`), or the plugin was not built |
| a pack `robotik.xxx` is named in the error | A pack of `resources.packs` is not found, or has an incompatible version |
| `Unknown Pinocchio frame: tcp` | An assertion uses `robot.tcp` but no tool is mounted: use `robot.tool0` |
| a `body` falls forever | The world has no `floor:` |

Next: [A planned pick and place](06-a-planned-pick-and-place.md).
