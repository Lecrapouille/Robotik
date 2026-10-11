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

make download-external-libs      # once: fetch MuJoCo, Pinocchio, ...
make compile-external-libs       # once: compile them
make -j"$(nproc --all)"          # Robotik, the two hosts and all plugins
```

After the build, the `build/` folder contains:

```text
build/
├── Robotik-Simulator      the 3D window
├── Robotik-Headless       the same, without window
├── librobotik-core.so     the library
└── plugins/               one folder per plugin (demo), ready to run
    ├── HelloWorld/
    ├── PickAndPlaceBT/
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

Useful things in the window:

| Element | What it does |
|---------|--------------|
| **Demos** menu | Loads a scenario |
| **Scenario file** panel | Path of the scenario, **Play / Pause**, **Step** (one step), the **seed**, **Replay**, **New seed** |
| **Scenario** panel | The assertions: `[pass]` in green, `[FAIL]` in red, or the current value while waiting. Also truth against belief |
| **Skills** panel | Which skills run, wait or failed, and why |
| **Behavior tree** panel | The tree, with the state of each node |
| **Robot camera** panel | What the robot camera sees |
| **Resources** panel | Which skill holds which resource, which resource is in fault |
| **Teach** panel | Moves the joints by hand, changes the tool |
| Plugin panel | Each plugin can add its own panel (here: **Hello world**) |

You can also give a scenario file directly:

```bash
./build/Robotik-Simulator build/plugins/HelloWorld/scenarios/wave.yml
```

Now try the reference demo, **Demos → Pick and place**: a 6-axis arm finds
a red cube with its camera, takes it with a vacuum cup and drops it in a box.

---

## 3. Run without a window: headless

The same scenario, in a terminal:

```bash
./build/Robotik-Headless build/plugins/HelloWorld/scenarios/wave.yml
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
| `--report file.json` | Writes a full JSON report |
| `--trace` | Prints the history of the skills |

```bash
./build/Robotik-Headless build/plugins/HelloWorld/scenarios/skills.yml --trace
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

1. Open `build/plugins/HelloWorld/scenarios/wave.yml` in an editor (this is a
   copy made by `make`; the source is in `plugins/HelloWorld/scenarios/`).
   Do not create a new file there yet: a scenario must be listed in
   `plugin.yaml`, otherwise Headless says `scenario is not in a plugin package`.
2. Change the assertion `wave.swing > 0.4` into `wave.swing > 0.9`.
3. Run it again:

   ```bash
   ./build/Robotik-Headless build/plugins/HelloWorld/scenarios/wave.yml; echo "exit code: $?"
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
   **Scenario** panel, but the simulation **does not stop**: a failed
   assertion is a verdict, not an emergency stop.
5. Put back `0.4` (or run `make` again: it copies the sources over `build/`).

---

## 5. Tutorial: run an experiment

An **experiment** runs several variants of a scenario, with several seeds,
and compares them.

```bash
./build/Robotik-Headless \
    build/plugins/PickAndPlaceBT/experiments/pick_and_place_benchmark.yaml \
    --report pick.json
```

```text
[PASS] nominal                  seed=11     controller succeeded at 7.4 s (oracle perception)  time=7.4  ...
[PASS] wide_layout              seed=11     ...
[PASS] camera_down_at_start     seed=11     controller succeeded at 8.9 s (oracle perception)  time=8.9  ...
pick_and_place_benchmark: 9/9 runs passed
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
