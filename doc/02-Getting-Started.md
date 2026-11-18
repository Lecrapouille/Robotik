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

make forge                       # optional, once: Pinocchio and MuJoCo in external/forge
make                             # the libraries, Robotik, the two hosts and the plugins
```

The first `make` fetches the third parties ([MyMakefile](https://github.com/Lecrapouille/MyMakefile)): Compages, BlackThorn, Dear ImGui, kairos and the libraries of the plugins, at the commits of `external/versions.lock`. `PLUGINS` lists the plugins built; MicroDuck and HumanoidWalk download large models: `make PLUGINS+="MicroDuck HumanoidWalk"`.

After the build, the `build/linux-gcc-debug/` folder contains:

```text
build/linux-gcc-debug/
├── bin/
│   ├── Robotik-Simulator      the 3D window
│   └── Robotik-Headless       the same, without window
├── lib/
│   ├── librobotik-core.so     the library
│   └── robotik/plugins/       one folder per plugin (demo), ready to run
│       ├── HelloWorld/
│       ├── PickAndPlace/
│       └── ...
└── share/robotik/packs        the asset packs (a link to data/packs)
```

The CI also publishes the simulator as a web page: [Robotik-Simulator.html](https://lecrapouille.github.io/Robotik/Robotik-Simulator.html). The versioned files of `plugins/` are in the page. Pinocchio, MuJoCo and the 3D view are not.

Unit tests, for developers:

```bash
make tests -j"$(nproc --all)"
```

---

## 2. Watch a robot: the simulator

```bash
./build/linux-gcc-debug/bin/Robotik-Simulator
```

Open the **Demos** menu. Each entry is a plugin; each sub-entry is one of its
scenarios. Choose **Hello world → 2. Move a joint**: the arm turns left and
right for six seconds.

The default Mission layout keeps the scene, an Inspector and a short Timeline:

```text
┌ File  Demos  View  Help                    [Plugin] [Robot] [Mission] [Autonomy] ┐
│ Play / Step    time and state    Replay / seed / speed           Emergency stop │
├──────────────────────────────────────────────────────────┬────────────────────┤
│                                                          │ Inspector          │
│                    3D scene                              │                    │
│                                      camera inset        │ selected item      │
├──────────────────────────────────────────────────────────┤                    │
│ Timeline                                                 │                    │
├──────────────────────────────────────────────────────────┴────────────────────┤
│ Scenario / assertions / faults / selection                                    │
└───────────────────────────────────────────────────────────────────────────────┘
```

The **run bar** is the same in every workspace: play, step, the simulated
time, the state of the run, replay and the seed, the speed, and the
emergency stop when the scenario has a `Stop` skill. The **status bar** says
what to look at: how many assertions pass, how many resources are in fault,
and the item selected. Each of them is a link: a click shows it in the
Mission workspace.

**Optional windows.** In Robot, Mission and Autonomy, **View** lists every
simulation window. Tick a name to open it; untick it or use its close button
to hide it. The choice is saved with the layout, separately for each
workspace. Windows are available across these workspaces. When a view moves
to the main area, its former window disappears and releases its space.
Choose it in **View** to return it to a separate window.

**Selection.** A click on a skill, a resource, a robot or an object (in the
Timeline, the Behavior tree, the Skills, Resources or World model panel, a
click in the 3D view, or a link of the Inspector) selects it in every panel: the **Inspector** of each workspace
shows it (for a skill: state and why, resources and who holds them,
preconditions, last phases; for a resource: who holds it, who waits for it
and its faults; for a robot: its skills and joints; for an object: what the
robot believes against the truth, the error, the confidence, where the
belief comes from), and the Timeline highlights its row. Selecting never commands the robot. **Pin**
keeps an item in the Inspector while you select others; **Overview** shows
what runs, waits and is in fault.
Scheduling details, recent phases and resource history start folded.
Failed preconditions and current faults stay visible. Each selected item
keeps its own expanded sections during the session.

**Robot details.** The robot Inspector shows joint positions and active
skills. Expand **Include inactive skills** to see the others. Its buttons
open Joint controls, Teach Pendant or Plots without changing who drives
the robot. Joint controls show positions first; **Show details** adds
limits, velocity, acceleration, effort and jog controls. Plots start with
one joint and one measurement. Choose joints in the plot itself and choose
**All measurements** only when you need the four plots together.

**Camera in the scene.** When the robot has a camera, its picture (with
the detections of the perception) sits in the bottom-right corner of the 3D
scene, in every workspace: no need for a Cameras panel to see what the
robot sees. `<` and `>` change the camera, `x` hides it, **Camera** shows
it again; `+` (or a double click on the picture) shows it large.

**Main area.** The large area in the middle of Robot, Mission and Autonomy
shows the 3D scene by default. Its selector (top left) puts another view in
its place: Camera, Timeline, Joint plots, Behavior tree, Planner, Execution
graph or Events, at the size of the scene. The **execution graph** reads
from left to right: with a PDDL planner, one box per step of the plan
(done, running, *waits for step 3*, failed) with the skills it runs, an
arrow from a step to the steps that wait for it, so the steps that run
together are in the same column; without a planner, the behavior tree, the
root on the left. Click a skill to inspect it; the graph scrolls to the
skill selected elsewhere; drag the background to move it. Its separate window hides while the main area shows it. Use **View** to
open it separately again. Each workspace keeps its own choice.

**Events.** The tab next to the Timeline tells the run as dated lines: the
skills that wait (and for what), start and end (succeeded after 0.5 s,
failed and why, cancelled), the resources that fall in fault and come
back, the plans and their steps, the failures of the tree, the verdicts of
the assertions. The same line repeated (a skill refused at each step) is
one line, *x50, until 0.99 s*. Tick the kinds to show, *Problems only* for
the failures and faults, or type a word to filter. A click on a time shows
it in the Timeline; a click on a line selects its skill or resource. The
list follows the new lines; scroll up to read, come back to the bottom to
follow again.

**Full view.** When a view needs room, **Ctrl+Space** over it (or its *Full
view* button) shows it over the whole work area: the 3D scene, the
Timeline (every row at once), the camera, the joint plots, the behavior
tree or the planner. The run bar stays above it.
**Ctrl+Space** again, **Esc** or *Restore* brings the panels back, where
they were.

A **workspace** is a set of panels for one task. Switch with the buttons at
the top, or with **Ctrl+1** to **Ctrl+4**:

| Workspace | Panels | Use it to |
|-----------|--------|-----------|
| **Plugin** | Plugin (description, files of the package, dependencies with their licenses), Files (read-only, with syntax colours for YAML and PDDL, one tab per kind: Scenario (titled RL for an `x.rl.yaml`), World, Behavior tree, Planner (domain, problem, output, generated tree; or a domain clicked alone), Experiment, Asset (the file clicked in Assets), and one for any other file; Behavior tree, Planner and Asset are always there, empty when there is nothing to show), one view beside it that follows the tab read: Scenario (its settings and assertions), World (entity tree, with the physics of each entity: its level abstract, rigid or contact, and what MuJoCo makes of it), Behavior tree, Planner (goal, plan, beliefs) or Experiment (cases, seeds, evaluation, command to run it); Assets (asset packs: models, files, version, license, source; a click on a model or a file shows it in the Asset tab of Files: a URDF, STL or glTF model in 3D that the mouse turns around, the picture for an image, the text for any other file). No 3D view. A click in the files of the package loads a scenario, or shows the file in its tab | Read what is loaded, where it comes from, and whether it passes |
| **Robot** | Scene with camera inset, Inspector showing the robot. Open Joints, Teach Pendant or Joint plots from **View** when needed. Manual mode suspends the mission | Inspect the body, then choose a manual control |
| **Mission** | Scene with camera inset, Inspector and compact Timeline. Open Behavior tree, Skills, Resources, Planner, Events or Assertions from **View** | Watch execution and inspect a problem |
| **Autonomy** | Scene with camera inset and Inspector. Open World model, Perception or Cameras from **View** | Compare beliefs and perception with the scene |

The panel of the plugin (here **Hello world**) opens by itself next to the
panels of the workspace, except in **Plugin**, which only reads the package. You can move and dock every panel; the layout of
each workspace is saved in `~/.config/robotik/simulator.ini`. **View → Reset
layout** restores its default windows and scene view.

Main shortcuts (all of them in **Help → Keyboard shortcuts**, F1):

| Key | Does |
|-----|------|
| Space | Play / pause |
| `.` | One step, when paused |
| Ctrl+R / Ctrl+N | Replay with the same seed / the next seed |
| Ctrl+E | Emergency stop, or release it |
| Ctrl+Space | The view under the mouse (scene, timeline, camera) over the whole work area, or back; Esc comes back too |
| F5 | Reload the scenario file (after you edit it) |

The 3D view moves like the camera of a strategy game: it looks at a point
of the floor. The mouse acts only over the view; the keys too, and not
while you type in a text field.

| Mouse or key | Does |
|--------------|------|
| Click | Select the robot or the object under the mouse (nothing: clear the selection). A selected object gets a light box where it is and an amber box where the robot believes it is |
| Left drag | Drag the floor: the point under the mouse stays under it |
| Right or middle drag | Turn around the point looked at, and up or down |
| Wheel, R / F | Zoom in / out |
| W A S D, arrows | Move along the floor |
| Q / E | Turn |
| Shift | Faster |

The camera glides to where you send it, at the same speed whatever the
frame rate.

You can also give a scenario file directly:

```bash
./build/linux-gcc-debug/bin/Robotik-Simulator build/linux-gcc-debug/lib/robotik/plugins/HelloWorld/scenarios/wave.scenario.yaml
```

`--screenshot view.png` plays the scenario, saves the 3D view into a PNG
file and quits; `--at 6` takes it after 6 simulated seconds (3 by
default, or at the end of the run if it ends before):

```bash
./build/linux-gcc-debug/bin/Robotik-Simulator build/linux-gcc-debug/lib/robotik/plugins/DualArm/scenarios/conveyor.scenario.yaml --screenshot conveyor.png --at 6
```

`--screenshot-ui ui.png` does the same with the whole window, panels
included (`--at 0`: without playing). `--workspace robot` opens a workspace
other than Mission, and `--select skill:Grasp` (or `resource:gripper`,
`robot:left`) chooses what the Inspector shows: handy to check the
interface without a person in front of it.

Now try the reference demo, **Demos → Pick and place**: a 6-axis arm finds
a red cube with its camera, takes it with a vacuum cup and drops it in a box.

---

## 3. Run without a window: headless

The same scenario, in a terminal:

```bash
./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/HelloWorld/scenarios/wave.scenario.yaml
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
./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/HelloWorld/scenarios/skills.scenario.yaml --trace
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

1. Open `build/linux-gcc-debug/lib/robotik/plugins/HelloWorld/scenarios/wave.scenario.yaml` in an editor (this is a
   copy made by `make`; the source is in `plugins/HelloWorld/scenarios/`).
   Do not create a new file there yet: a scenario must be listed in
   `plugin.yaml`, otherwise Headless says `scenario is not in a plugin package`.
2. Change the assertion `wave.swing > 0.4` into `wave.swing > 0.9`.
3. Run it again:

   ```bash
   ./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/HelloWorld/scenarios/wave.scenario.yaml; echo "exit code: $?"
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
./build/linux-gcc-debug/bin/Robotik-Headless \
    build/linux-gcc-debug/lib/robotik/plugins/PickAndPlace/experiments/pick_and_place.experiment.yaml \
    --report pick.json
```

```text
[PASS] nominal                  seed=11     controller succeeded at 7.4 s (perception truth)  time=7.4  ...
[PASS] wide_layout              seed=11     ...
[PASS] rigid                    seed=11     ...
[PASS] camera_down_at_start     seed=11     controller succeeded at 8.9 s (perception truth)  time=8.9  ...
pick_and_place_experiment: 12/12 runs passed
```

4 variants × 3 seeds = 12 runs. `pick.json` contains every run, every
assertion, every metric. (`perception truth` means that, without a window,
no image is rendered, so the true cube position replaced the camera, as the
scenario asks with `perception: { blind: truth }`. See
[03-Worlds-and-Scenarios.md §7](03-Worlds-and-Scenarios.md#7-truth-description-and-belief).)

---

## 6. The other programs

Some demos also have their own program, for uses the two hosts do not cover:

```bash
./build/linux-gcc-debug/bin/Robotik-LineFollower --seed 4 --laps 1 --save map.png   # draws the path
./build/linux-gcc-debug/bin/Robotik-LineFollower --view                              # shows the camera
./build/linux-gcc-debug/bin/Robotik-PickAndPlaceRL --policy converged --envs 16 --threads 8 --episodes 32
./build/linux-gcc-debug/bin/Robotik-Fly --brain reflex
./build/linux-gcc-debug/bin/Robotik-Fly --headless --envs 8 --episodes 8 --seed 123456
```

All demos are described in [09-Demos.md](09-Demos.md).

---

## 7. The three layers in code

RobotIK has three layers. Each one works without the ones above it, so you
pick the smallest one that fits your project. The words are explained in
the [glossary](Glossary.md).

| Layer | You have… | You write… |
|-------|-----------|------------|
| **Robot** | Scene with camera inset, Inspector showing the robot. Open Joints, Teach Pendant or Joint plots from **View** when needed. Manual mode suspends the mission | Inspect the body, then choose a manual control |
| **Task** | robots, objects and a job | a robot cell in C++, with skills and a controller |
| **Package** | YAML files and a plugin | a scenario file that builds the cell and judges the run |

**Robot**: open a robot and move a joint
(`tests/Robot/TestOpenRobot.cpp`).

```cpp
auto arm = robotik::openRobot("robotik.robots.robot_6axis");
if (!arm)
    return fail(arm.error());                  // a message telling what to fix
robotik::JointSet& joints = arm.value()->joints();
joints.moveTo(joints.revolute("joint2"), robotik::Radians(0.3));
for (int i = 0; i < 200; ++i)
    arm.value()->step(robotik::Seconds(0.01)); // MuJoCo moves the joint
```

**Task**: a robot and a cube described in C++, and a skill run by the
scheduler (`tests/Examples/TestLayerExamples.cpp`).

```cpp
robotik::CellDescription description;
robotik::EntitySpec arm;
arm.id = "arm";
arm.type = robotik::EntitySpec::Type::Robot;
arm.model = "robot_6axis.urdf";
robotik::EntitySpec cube;
cube.id = "cube";
cube.shape = robotik::ShapeSpec{};             // a 4 cm box
cube.pose.position = robotik::Vector3(0.4, 0.2, 0.02);
description.world.entities = { arm, cube };

compages::world::World world;
robotik::Cell cell(world, description);
auto const raise = cell.skills().add<robotik::MoveJointSkill>(
    { .name = "Raise" }, "joint2", 0.5);
cell.skills().request(raise);
while (cell.skills().state(raise) != robotik::SkillState::Succeeded)
    cell.step(robotik::Seconds(0.01));
```

**Package**: the scenario file builds the cell; the simulation judges the
run with the assertions of the file (same test file).

```cpp
auto scenario = robotik::loadScenario("hello.scenario.yaml", assets);
if (!scenario)
    return fail(scenario.error());
compages::world::World world;
robotik::Simulation simulation(world, scenario.take(), nullptr, &mission);
while (!simulation.finished())
    simulation.step(robotik::Seconds(0.01));
bool const ok = simulation.passed();
```

A plugin is the same Package layer, loaded by the simulator or by
Robotik-Headless: [04-Your-First-Plugin.md](04-Your-First-Plugin.md).

---

## 8. Where to go next

- Change the world and the scenario, without C++:
  [03-Worlds-and-Scenarios.md](03-Worlds-and-Scenarios.md).
- Write your own C++ code: [04-Your-First-Plugin.md](04-Your-First-Plugin.md).
