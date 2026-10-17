# 2. Getting started

In this page you build Robotik, watch a robot in the simulator, run the same
scenario without a window, and break a test on purpose to see what a failure
looks like. It takes about 20 minutes (most of it is compilation).

---

## 1. Build

Robotik builds on **Linux** with `make` (no CMake). It does not build on
macOS: the display needs OpenGL 4.3 or more.

```bash
git clone https://github.com/Lecrapouille/RobotIK --recurse-submodules
cd RobotIK

make download-external-libs      # once: fetch MuJoCo, Pinocchio, ... and the
                                 # libraries of each plugin
make -j"$(nproc --all)"          # the libraries, Robotik, the two hosts and all plugins
```

After the build, the `build/` folder contains:

```text
build/
├── Robotik-Simulator      the 3D window
├── Robotik-Headless       the same, without window
├── librobotik-core.so     the library
└── plugins/               one folder per plugin (demo), ready to run
    ├── HelloWorld/
    ├── PickAndPlace/
    └── ...
```

Unit tests, for developers:

```bash
make tests -j"$(nproc --all)"
```

---

## 2. Watch a robot: the simulator

```bash
./build/Robotik-Simulator
```

Open the **Demos** menu. Each entry is a plugin; each sub-entry is one of its
scenarios. Choose **Hello world → 2. Move a joint**: the arm turns left and
right for six seconds.

The window has four parts:

```text
┌ File  Demos  View  <plugin menus>  Help      [Plugin] [Robot] [Mission] [Autonomy] ┐  menu + workspaces
├──────────────┬───────────────────────────────────────────────┬─────────────────────┤
│              │ [Play] [Step] [Replay] seed [Next seed] speed │                     │
│  panels of   │               3D scene        [EMERGENCY STOP]│  panels of the      │
│  the         │                                               │  workspace, and the │
│  workspace   ├───────────────────────────────────────────────┤  plugin panel       │
│              │  panels of the workspace                      │                     │
├──────────────┴───────────────────────────────────────────────┴─────────────────────┤
│ RUNNING  Pick and place  seed 42  Assertions 3/5            t 4.2 s  x1.0 real time │  status bar
└─────────────────────────────────────────────────────────────────────────────────────┘
```

The run controls are on the 3D scene. The **status bar** always says what is
happening: the state of the run, the scenario, the seed, how many assertions
pass (click it to see them), the simulated time and the real-time factor.

A **workspace** is a set of panels for one task. Switch with the buttons at
the top, or with **Ctrl+1** to **Ctrl+4**:

| Workspace | Panels | Use it to |
|-----------|--------|-----------|
| **Plugin** | Plugin (description, files of the package, dependencies with their licenses), Files (read-only, one tab per kind: Scenario, World, Behavior tree, Planner (domain, problem, output, generated tree), Experiment, and one for any other file), one view beside it that follows the tab read: Scenario (its settings and assertions), World (entity tree), Behavior tree, Planner (goal, plan, beliefs) or Experiment (cases, seeds, evaluation, command to run it); Assets (asset packs: models, version, license, source). No 3D view. A click in the files of the package loads a scenario, or shows the file in its tab | Read what is loaded, where it comes from, and whether it passes |
| **Robot** | Joints (limits, velocity, effort; drag a position, or Ctrl+click to type it, to move a joint), Joint plots, Teach Pendant (tool, Cartesian moves of the tool or of any link, waypoints), Cameras | Look at the body, move it by hand. In Manual mode the mission is suspended and the robot moves even when the run is paused |
| **Mission** | Behavior tree, Timeline (with the resources), Resources (who holds what, faults by hand), Assertions, Skills (cards or table), Planner (the plans of the PDDL planner, see [chapter 11](11-PDDL-Planner.md#debugging)) | Debug the execution |
| **Autonomy** | World model (belief against truth), Cameras, Perception (detections) | Check what the robot believes and sees |

The panel of the plugin (here **Hello world**) opens by itself next to the
panels of the workspace. You can move and dock every panel; the layout of
each workspace is saved in `~/.config/robotik/simulator.ini`. **View → Reset
layout** puts it back.

Main shortcuts (all of them in **Help → Keyboard shortcuts**, F1):

| Key | Does |
|-----|------|
| Space | Play / pause |
| `.` | One step, when paused |
| Ctrl+R / Ctrl+N | Replay with the same seed / the next seed |
| Ctrl+E | Emergency stop, or release it |
| F5 | Reload the scenario file (after you edit it) |

You can also give a scenario file directly:

```bash
./build/Robotik-Simulator build/plugins/HelloWorld/scenarios/wave.scenario.yaml
```

Now try the reference demo, **Demos → Pick and place**: a 6-axis arm finds
a red cube with its camera, takes it with a vacuum cup and drops it in a box.

---

## 3. Run without a window: headless

The same scenario, in a terminal:

```bash
./build/Robotik-Headless build/plugins/HelloWorld/scenarios/wave.scenario.yaml
```

Output:

```text
[PASS] controller.success  (controller.success = 1)
[PASS] wave.swing > 0.4  (wave.swing = 0.598172)
[PASS] joint_limits_respected(robot)  (joint_limits_respected(robot) = 1)
[PASS] default                  seed=1      controller succeeded at 6.01 s  time=6.01
wave: 1/1 runs passed
```

Read it like this:

- one line per **assertion**, with the **measured value** in parentheses;
- one summary line per **run**: the seed, why the run stopped
  (`controller succeeded at 6.01 s`) and the simulated time;
- the exit code: `0` all passed, `2` a run failed, `1` a file or a plugin
  could not load. This is what CI uses.

Options:

| Option | Effect |
|--------|--------|
| `--seed N` | Replaces the seed of the scenario (random layout, sensor noise, faults) |
| `--report file.json` | Writes a full JSON report: outcome, metrics, assertions, and the plans of the PDDL planner |
| `--trace` | Prints the history of the skills |

```bash
./build/Robotik-Headless build/plugins/HelloWorld/scenarios/skills.scenario.yaml --trace
```

```text
  0 s  Home  SUCCEEDED  (0 s)
  0.01 s  Wave  SUCCEEDED  (4 s)
  4.02 s  Home  SUCCEEDED  (0.63 s)
[PASS] controller.success  (controller.success = 1)
...
```

---

## 4. Tutorial: break a test on purpose

The goal: see that a scenario *is* a test.

1. Open `build/plugins/HelloWorld/scenarios/wave.scenario.yaml` in an editor (this is a
   copy made by `make`; the source is in `plugins/HelloWorld/scenarios/`).
   Do not create a new file there yet: a scenario must be listed in
   `plugin.yaml`, otherwise Headless says `scenario is not in a plugin package`.
2. Change the assertion `wave.swing > 0.4` into `wave.swing > 0.9`.
3. Run it again:

   ```bash
   ./build/Robotik-Headless build/plugins/HelloWorld/scenarios/wave.scenario.yaml; echo "exit code: $?"
   ```

   ```text
   [PASS] controller.success  (controller.success = 1)
   [FAIL] wave.swing > 0.9  (wave.swing = 0.598172)
   [PASS] joint_limits_respected(robot)  (joint_limits_respected(robot) = 1)
   [FAIL] default                  seed=1      controller succeeded at 6.01 s  time=6.01
            FAIL wave.swing > 0.9  (wave.swing = 0.598172, expected > 0.9, at 6.01 s)
   wave: 0/1 runs passed
   exit code: 2
   ```

   The failure keeps the **measured value**, the **expected bound** and the
   **time**: you know immediately the joint turned 0.6 rad, not 0.9.
4. Open the same file in the simulator. The assertion is red in the
   **Assertions** panel (Plugin workspace) and the status bar says
   `Assertions 2/3`, but the simulation **does not stop**: a failed
   assertion is a verdict, not an emergency stop.
5. Put back `0.4` (or run `make` again: it copies the sources over `build/`).

---

## 5. Tutorial: run an experiment

An **experiment** runs several variants of a scenario, with several seeds,
and compares them.

```bash
./build/Robotik-Headless \
    build/plugins/PickAndPlace/experiments/pick_and_place.experiment.yaml \
    --report pick.json
```

```text
[PASS] nominal                  seed=11     controller succeeded at 7.4 s (oracle perception)  time=7.4  ...
[PASS] wide_layout              seed=11     ...
[PASS] camera_down_at_start     seed=11     controller succeeded at 8.9 s (oracle perception)  time=8.9  ...
pick_and_place_experiment: 9/9 runs passed
```

3 variants × 3 seeds = 9 runs. `pick.json` contains every run, every
assertion, every metric. (`oracle perception` means that, without a window,
no image is rendered, so the true cube position replaced the camera. See
[03-Worlds-and-Scenarios.md §7](03-Worlds-and-Scenarios.md#7-truth-description-and-belief).)

---

## 6. The other programs

Some demos also have their own program, for uses the two hosts do not cover:

```bash
./build/Robotik-LineFollower --seed 4 --laps 1 --save map.png   # draws the path
./build/Robotik-LineFollower --view                              # shows the camera
./build/Robotik-PickAndPlaceRL --policy converged --envs 16 --threads 8 --episodes 32
./build/Robotik-Fly --brain reflex
./build/Robotik-Fly --headless --envs 8 --episodes 8 --seed 123456
```

All demos are described in [09-Demos.md](09-Demos.md).

---

## 7. Where to go next

- Change the world and the scenario, without C++:
  [03-Worlds-and-Scenarios.md](03-Worlds-and-Scenarios.md).
- Write your own C++ code: [04-Your-First-Plugin.md](04-Your-First-Plugin.md).
