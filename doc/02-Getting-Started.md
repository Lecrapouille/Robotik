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

The Mission layout puts the execution (the steps of the plan or the nodes
of the behavior tree, with their skills) in the middle, a small scene and
the resources on the right and a short Timeline below. A scenario with no tree
and no plan gets the scene in the middle instead:

```text
┌ File  Demos  Windows  Layout  Help         [Plugin] [Robot] [Mission] [Autonomy] ┐
│ Play / Step    time and state    Replay / speed / Run options           Emergency stop │
├──────────────────────────────────────────────────────────┬────────────────────┤
│                                                     [⤢]  │ 3D scene      [⤢]  │
│      Execution graph / Planner (steps, skills,               │                    │
│      state, priority, resources, why)                    ├────────────────────┤
│                                                          │ Resources          │
├──────────────────────────────────────────────────────────┤ Assertions Planner │
│ Timeline  Events                                         │                    │
├──────────────────────────────────────────────────────────┴────────────────────┤
│ Scenario / assertions / faults / selection                                    │
└───────────────────────────────────────────────────────────────────────────────┘
```

The **run bar** is the same in every workspace: play, step, the simulated
time, the state of the run, replay, a compact speed selector, and the
emergency stop when the scenario has a `Stop` skill. **Run options...**
contains the seed, **Next seed and replay**, and continuous speed control.
The **status bar** says
what to look at: how many assertions pass, how many resources are in fault,
and the item selected. Each is a link that opens the relevant editor
in the current layout.

The interface uses charcoal surfaces and one amber accent, with a 14 px
font and tight spacing so that tables show many rows. Status colours
distinguish running, success and failure.

**Windows.** Every editor is available from the **Windows** menu, including
Plugin, Files, Package details, Assets and the loaded plugin's own panels.
Choose an editor to open it or bring its existing tab to the front. Close
it with the tab's `x` (the only close button). Each editor has one window across all layout presets.
Each preset remembers its own editors: what you open or move in Robot is
still there when you come back from Mission.

**Layout** has two recovery actions. **Place open panels** keeps the open
editors and returns them to the current preset's positions.
**Restore recommended layout** restores that preset's default editors.
The current positions, visibility and preset are saved in
`~/.config/robotik/simulator.ini`. The first launch after this interface
update starts with the new default layout.

**Selection.** A click on a skill, a resource, a robot or an object (in the
Timeline, the Execution graph, the Resources or World
model panel, a click in the 3D view, or a link of the Inspector) selects it
in every panel, and the Timeline highlights its row. Selecting never
commands the robot. The shared **Inspector** shows the selection the same
way for every kind of item: its kind (SKILL, RESOURCE, ROBOT, OBJECT) with
**Pin** and **Overview** beside it, its name and state, one sentence that
says what it does now ("Waiting. busy: arm held by Approach(box)", "Free:
no skill holds it", "Believed 33.9 mm from where it really is"), then a
table of facts, all visible:

| Item | Facts |
|------|-------|
| Skill | robot, priority and whether it can be preempted, what it does when busy, its resources (shared or exclusive, free or held by whom), its preconditions (a false one in red), its last runs |
| Resource | held by, waited for by, needed by, its faults in this run |
| Robot | who drives it (MISSION or MANUAL), model, tool, skills now, a joint table (value and where it is between its limits) |
| Object | belief, truth, error, confidence, how often and when it was seen, size |

**Pin** keeps an item while you select others; **Overview** shows the run:
robots, what runs, waits, failed, is in fault, the objects believed.

No workspace opens the Inspector at first: the first click on a name opens
it, later clicks leave the focus where it is.

**Robot details.** The robot Inspector shows joint positions and active
skills. Expand **Include inactive skills** to see the others. Its buttons
open Joint controls, Teach Pendant or Plots without changing who drives
the robot. Joints say who drives (**Mission drives** or **Manual**; the
tooltip of **Manual** explains it), then the tool position and the gripper
on one line. Joint controls show positions first; **Show details** adds
limits, velocity, acceleration, effort and jog controls. Plots start with
one joint and one measurement. Choose joints in the plot itself and choose
**All measurements** only when you need the four plots together.

**Camera in the scene.** When the robot has a camera, its picture (with
the detections of the perception) sits in the bottom-right corner of a
large 3D scene: no need for a Cameras panel to see what the robot sees.
The inset is left out when the Cameras panel is shown (Autonomy) and when
the scene is small (Mission). `<` and `>` change the camera, the cross
hides it, **Camera** shows it again; the expand icon (or a double click on
the picture) shows it large.

**Scene and execution graph.** The 3D scene is an editor that you can close,
move, dock and expand like any other window. The **Execution graph** is the
middle of Mission; open it from **Windows** in another layout. **List**
(the default) shows each step of the plan, or each node of the behavior
tree indented by depth, followed by its skills: their state, priority,
resources and why they wait. **Graph** draws the same steps as boxes, with
arrows to the steps that wait for them; independent steps share a column,
and a tree reads from left to right. Click a skill to inspect it; drag the
background to move the graph. The graph writes the priority and the
resources after each skill (`p100  arm`). The list is also where you run a
skill by hand: **Request** on an idle skill, **Cancel** on a running or
waiting one. The skills that no step runs (`Home`, `Stop`…) are under
**Other skills** at the end; without a plan or a tree, the list is all the
skills.

**Resources.** One table: a box to tick, the resource, its state (free,
shared, exclusive, or in fault with who made it and the time left), the
skills that hold it and those that need it, and **Restore** on a resource
in fault. Under it: tick resources, give a duration (ever: until you
restore them) and press **Inject**; **Restore all** ends every fault. Then
the faults that the scenario plans.

**Compact timeline.** The initial view follows the latest five simulated
seconds. At startup it uses a shorter axis to keep early actions readable.
**Hide unused** starts enabled: skills never requested do not take a row.
Untick it to see all skills. **Whole run**, wheel zoom and panning remain
available. Close **Timeline** with its tab's `x` when the scene needs more room.

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

**Expand.** Every editor has a small icon in its top-right corner (four
corners pointing out). Click it, or click the editor and press
**Ctrl+Space**: the same window fills the work area while the run controls
stay visible. This works for every editor, including Inspector, Files and
plugin panels. The icon then points in: click it, or press **Ctrl+Space**
again or **Esc**, to go back to the original positions and tab order. You can also right-click a window title for
**Expand**, **Restore** or **Close**. Opening another editor from **Windows**
restores the layout and focuses that editor. Exiting while expanded saves
the original layout. A small live 3D view stays in the bottom-right corner
while another editor is expanded. Its cross hides it; **3D scene** shows
it again. Its restore icon returns to the layout.

The top buttons select a **layout preset**. **Ctrl+1** to **Ctrl+4** select
the same presets. The first visit opens the default editors listed below,
with the tab in **bold** in front. Then you can close or add editors from
**Windows** and move them: leaving the preset keeps them, coming back shows
them again where they were. The open editors of each preset are saved in
`simulator.ini`; their positions are kept for the session. Editor identities
and content state remain shared, so switching does not create copies.
Clicking the active preset, or **Layout → Restore recommended layout**,
reapplies its defaults.

| Preset | Default editors | Use it to |
|--------|-----------------|-----------|
| **Plugin** | Plugin, Files, Package details, Assets | Read the package, scenario, world, experiments and asset packs. Files shows YAML, PDDL and other files as text. Package details has no tabs of its own: it follows the tab read in Files (Scenario, World, Experiment); Behavior tree opens Execution graph; Planner opens its own editor. Right-click the folder name in Plugin to copy its path or read it again. |
| **Robot** | left: small 3D scene above Joint plots (one line of controls: measurement, joints, time window); right: **Joints**, Teach Pendant | The body: inspect joints and limits, move the robot by hand. |
| **Mission** | Execution graph as a list and Planner when used (tabs in the middle); right: small 3D scene, then **Resources**, Assertions and plugin panels; below: **Timeline**, Events | The execution, and running skills by hand. Without a tree or a plan, the scene takes the middle and the list of skills sits on the right. |
| **Autonomy** | above: 3D scene and Cameras, the same size; below: World model and Perception side by side | Compare beliefs with the truth and with what the cameras see. |

The loaded plugin's panels open on first use and follow the same close,
reopen, dock and expand rules as the other editors. They remain accessible
in every preset, including Plugin. Their default position in Mission is
beside Resources; Robot places them beside Joints, Autonomy beside World
model, and Plugin beside Files. Autonomy stays focused on sensing and
beliefs; planning and execution belong to Mission.

The **Cameras** editor shows every
camera of the robot at once, in the grid that gives the largest pictures.

Main shortcuts (all of them in **Help → Keyboard shortcuts**, F1):

| Key | Does |
|-----|------|
| Space | Play / pause |
| `.` | One step, when paused |
| Ctrl+R / Ctrl+N | Replay with the same seed / the next seed |
| Ctrl+E | Emergency stop, or release it |
| Ctrl+Space | Expand the focused editor or restore the layout; Esc restores it too |
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
included (`--at 0`: without playing). `--workspace robot` restores the Robot preset
and its default editors, and `--select skill:Grasp` (or `resource:gripper`,
`robot:left`) chooses what the Inspector shows: handy to check the
interface without a person in front of it. `--view timeline` opens and
focuses the Timeline editor; `--maximize timeline` expands that editor.

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
[PASS] (> wave.swing 0.4)  (wave.swing = 0.598172)
[PASS] (joint_limits_respected robot)  ((joint_limits_respected robot) = 1)
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
2. Change the assertion `(> wave.swing 0.4)` into `(> wave.swing 0.9)`.
3. Run it again:

   ```bash
   ./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/HelloWorld/scenarios/wave.scenario.yaml; echo "exit code: $?"
   ```

   ```text
   [PASS] controller.success  (controller.success = 1)
   [FAIL] (> wave.swing 0.9)  (wave.swing = 0.598172)
   [PASS] (joint_limits_respected robot)  ((joint_limits_respected robot) = 1)
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
| **Robot** | a URDF file | one robot, its joints and its physics |
| **Task** | robots, objects and a job | a robot cell in C++, with skills and a controller |
| **Package** | YAML files and a plugin | a scenario file that builds the cell and judges the run |

Three small programs show them. Each one is a test, so it always builds and
runs. They share one engine: a skill runs through the same scheduler whether
you request it, a behavior tree asks for it, or a plan needs it.

**1. Move a robot, without a planner** (Robot layer,
`tests/Robot/TestOpenRobot.cpp`).

```cpp
auto arm = robotik::openRobot("robotik.robots.robot_6axis");
if (!arm)
    return fail(arm.error());                  // a message telling what to fix
robotik::JointSet& joints = arm.value()->joints();
joints.moveTo(joints.revolute("joint2"), robotik::Radians(0.3));
for (int i = 0; i < 200; ++i)
    arm.value()->step(robotik::Seconds(0.01)); // MuJoCo moves the joint
```

**2. Run a skill, without a mission** (Task layer,
`tests/Examples/TestLayerExamples.cpp`). A robot and a cube described in
C++; `openCell` creates the world that holds them.

```cpp
robotik::EntitySpec arm;
arm.id = "arm";
arm.type = robotik::EntitySpec::Type::Robot;
arm.model = "robot_6axis.urdf";
robotik::EntitySpec cube;
cube.id = "cube";
cube.shape = robotik::ShapeSpec{};             // a 4 cm box
cube.pose.position = robotik::Vector3(0.4, 0.2, 0.02);
robotik::CellDescription description;
description.world.entities = { arm, cube };

auto opened = robotik::openCell(std::move(description));
robotik::Cell& cell = *opened.value();
auto const raise = cell.skills().add<robotik::MoveJointSkill>(
    { .name = "Raise" }, "joint2", 0.5);
cell.run(raise, robotik::Seconds(10.0));       // SkillState::Succeeded
```

**3. Reach a goal, without writing a sequence of skills** (Task layer, same
test file). You give the goal; the planner finds the actions in the PDDL
domain, and the cell runs their skills. The mission gives the skills and the
observers that read the beliefs ([chapter 11](11-PDDL-Planner.md)).

```cpp
PickPlaceMission mission(true, true);          // skills + observers
auto opened = robotik::openCell(description, &mission);
robotik::Cell& cell = *opened.value();

robotik::PlannerSpec goal;
goal.domain = "pick_and_place.domain.pddl";
goal.goal = { robotik::parseLiteral("(in red_cube box)").take() };
cell.achieve(goal);
cell.run(robotik::Seconds(60.0));              // Status::SUCCESS: detect, pick, place
```

The `planner:` block of a scenario file calls the same `achieve`. The YAML
of the Package layer only reads files and calls the Task layer; it adds no
behaviour of its own.

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
