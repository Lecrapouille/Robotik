# AGENTS.md — context for coding agents

RobotIK: C++20 library to write a robot mission once and run it in a 3D
simulator, headless (CI), in RL environments, and later on hardware. It glues
MuJoCo (physics), Pinocchio (kinematics), Compages (ECS world + rendering),
BlackThorn (behavior trees) and Kairos (PDDL planner). Linux only, plain `make` (no CMake).

Read only what the task needs. Load a skill instead of reading the docs:

| Task | Skill (`.claude/skills/`) | Long doc |
|------|---------------------------|----------|
| Create / change a plugin (C++ + manifest) | `robotik-plugin` | doc/tutorials/04-your-first-plugin.md |
| Write world / scenario / experiment YAML, assertions | `robotik-scenario` | doc/reference/scenario-file.md, world-file.md |
| Write a skill, a behavior tree, handle faults | `robotik-skill` | doc/concepts/skills-and-behavior-trees.md |
| Plan from a PDDL goal (domain, observers, bindings) | `robotik-skill` | doc/concepts/pddl-planner.md |
| Change the simulator interface (panels, layouts, scenario editor) | `robotik-simulator` | doc/tutorials/01-getting-started.md |
| Edit the documentation | `robotik-docs` | doc/README.md |
| Write an app in C++ without YAML (Robot or Task layer) | — | doc/tutorials/02-three-programs.md |
| Find a class or method | — | doc/reference/api.md |
| A word you do not know | — | doc/reference/glossary.md |
| Change the core | — | doc/concepts/architecture.md |

## Layout

Three layers, each usable without the ones above (`make check-layers`
compiles every file alone where only the layers below exist, so an include
upwards is "file not found"; `Task/Skills` and `Task/Planning` are compiled
without `Task/RoboticCell.hpp` and `Robot/RobotSession.hpp`, so they cannot
call the truth nor reach the simulator). Words:
doc/reference/glossary.md.

```text
include/Robotik/<Layer>/...  public headers; src/Robotik/<Layer>/... implementation
  Common/   Math (units, Pose, from Compages), Result (+ROBOTIK_TRY), Signal,
            Config (editable YAML tree), Random (Seed), File, Version
  Robot/    one robot: Robot, RobotSession, openRobot, JointSet (SoA), LinkRef,
            Devices, Resources, Actuators, TeachPendant, Sensors/, Backends/
            (MuJoCo, Pinocchio, SceneView), Assets/ (packs, resolver),
            Components/ (ECS data on entities)
  Task/     a job in C++: RoboticCellDescription (data), RoboticCell (step loop, truth,
            controllers, perception fidelity), ScenarioLogic, SkillScheduler,
            RobotContext, Faults, Metrics, Skills/, Perception/ (WorldModel),
            Planning/ (Pddl, PlanningDomain, PlanExecutor, PlanGraph,
            Kairos/), Grasp/, Learning/ (RL environments), Agents/
  Package/  files and plugins: Simulation (scenario = cell + verdict),
            RunReport, Scenario/ (World, Scenario, Experiment, Assertions,
            Trajectory, PackagePath, FileRole), Plugin/ (PluginABI.h,
            PluginAPI.hpp, catalog, session)
simulator/   Robotik-Simulator (ImGui host)      headless/  Robotik-Headless (CLI host)
plugins/<Name>/  plugin.yaml, Makefile, src/, worlds/, scenarios/, experiments/, data/
plugins/plugin.mk  shared plugin build recipe        data/packs/<id>/  asset packs (URDF)
tests/       GoogleTest; tests/Plugin uses plugins/HelloWorld as fixture
external/    third-party sources (do not edit)      build/  outputs (generated)
```

Plugins: HelloWorld (tutorial + test fixture), PickAndPlace (reference: BT,
faults, RL scenario), Painter, LineFollower, FlyBrain, HumanoidWalk (ONNX policy),
DualArm (two robots, parallel and temporal PDDL plans), MicroDuck (a small
walking robot pushing a box in a room, ONNX policy).

## Commands

```bash
make                         # core, hosts, the plugins of PLUGINS
make <target>                # one plugin; stages build/linux-gcc-debug/lib/robotik/plugins/<Name>/
make check                   # builds and runs build/linux-gcc-debug/bin/Robotik-tests
make check-layers            # compiler: no include upwards, no RoboticCell.hpp nor RobotSession.hpp in skills/planning
make check-links             # every relative link of the Markdown files resolves
make check-asan              # tests with ASan (+UBSan if libubsan), then in a random order
make check-valgrind          # tests under valgrind memcheck (values read before written)
./build/linux-gcc-debug/bin/Robotik-tests --gtest_filter='Plugin*'
./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/<Name>/scenarios/<s>.scenario.yaml [--seed N] [--trace] [--report r.json]
```

Headless exit code: 0 pass, 2 a run failed, 1 load error. `make` copies
`worlds/ scenarios/ experiments/ data/` over `build/linux-gcc-debug/lib/robotik/plugins/<Name>/`: edit
sources, not `build/`. Third parties are MyMakefile 3 targets: the core
ones in `external/external.mk` (Compages, BlackThorn
read by their `.mk` files), a plugin's own in its `<Name>.mk`; their commits
are in `external/versions.lock`. `PLUGINS` (root `Makefile`) lists the plugins
built.

## Rules that are easy to break

- The core and the hosts stay agnostic of the demos: no demo object, joint,
  tool or task in `src/`, `include/`, `simulator/`, `headless/`. The demos
  prove RobotIK fits any robot project. Known leaks and the rule:
  `robotik-plugin` skill, "Core stays agnostic". Report a new leak.

- Skills see only `RobotContext` (robot API + `WorldModel` beliefs). Never
  give them ground truth (`RoboticCell::requirePose/requireQuery`); that is for metrics,
  assertions, RL reward. The ECS world, the entities, the backend and the
  writable kinematics are on `RobotSession` (own header), not on the
  `Robot` of a skill; `Task/Skills` and `Task/Planning` are compiled
  without `RobotSession.hpp`. A vacuum cup measures `holding()`; what it
  holds is the `label` the grasp skill wrote (the entity is truth:
  `RoboticCell::held`). A scenario may choose to feed beliefs from the
  truth (`perception.fidelity: truth | noisy_truth`, per object): it is a
  mock of the detectors, done by the `RoboticCell` and written in the run report,
  never by a skill.
- File names carry their role: `x.world.yaml`, `x.scenario.yaml`,
  `x.rl.yaml`, `x.bt.yaml`, `x.domain.pddl`, `x.trajectory.yaml`,
  `x.experiment.yaml` (doc/concepts/worlds-scenarios-experiments.md).
  id = name without `.yaml` and `.scenario`, other dots → `_`
  (`pick.rl.yaml` → `pick_rl`).
- Paths in world/scenario/experiment YAML start at the package root (folder
  of `plugin.yaml`): `world: worlds/arm.world.yaml`. `..` and symlinks are allowed
  but the canonical path must stay in the package; absolute paths are
  refused (`PackagePath.hpp`).
- A library used by one plugin goes in `dependencies:` of its `plugin.yaml`
  (`system:` pkg-config names, `thirdparties:` `{ repo, license, description }`;
  the license is required for the credits) and is built by a rule of its
  Makefile (`PLUGIN_REQUIRES`, `PLUGIN_DEPENDS`, see
  `plugins/plugin.mk`), not in the core manifest nor in `plugin.yaml`.
- Plugins publish metrics (`ScenarioLogic::measure`); verdicts live in scenario
  YAML assertions. No pass/fail logic in C++.
- Conditions and planner literals use the PDDL notation: `(< time 30)`,
  `(inside red_cube box)`. They are compiled at load (Task/Condition.hpp):
  a typo is a load error. From C++, ask `cell.requireQuery("inside", {a, b})`,
  never build a condition string; add a query with `RoboticCell::addQuery`.
- A step of the scenario logic has three sides, kept apart by the types: `act(RobotContext&)`
  commands from the robot and its beliefs (no truth), `simulate(RoboticCell&)` is
  what the world does by itself (sim only, never commands a robot),
  `observe(RoboticCell const&)` reads the truth for metrics (a const cell commands
  nothing: `skills()`, `robot()`, `robots()` are read only).
- `ScenarioLogic::status` ends a run only when the scenario has no behavior tree
  and no planner.
- Several robots in a world share one physics, one `ResourceManager`, one
  `SkillScheduler` and the beliefs: device names are unique in the world;
  `SkillDescription::robot` says which robot a skill drives; with several
  robots the beliefs are in the world frame (`RoboticCell::beliefFrame`), a
  robot alone keeps its base frame. `RoboticCell::robot()` is the first one.
- Plans run their independent steps together (`planner.parallel`, default
  true): the order comes from what each end of an action reads and changes,
  and from the resources of its skills. Do not add ordering in C++.
- Planner: the PDDL domain is the only place of actions and effects (no
  symbolic effects in C++); observers read beliefs, never the truth; only
  `src/Robotik/Task/Planning/Kairos/` includes Kairos headers. An effect is
  done only when the beliefs show it (`verify: observed`: seen after the
  start of the step).
- Names must match: target `robotik_x` → `librobotik_x.so` =
  `library.entrypoint`; `plugin.id` = `describe().id`; scenario `id` in
  `plugin.yaml` = string tested in `Plugin::setup`. A scenario not listed in
  `plugin.yaml` is refused ("scenario is not in a plugin package").
- Nothing C++ crosses the plugin border except through `PluginAPI.hpp`; the
  C table carries the Simulation and the ScenarioLogic as `void*`, so a
  plugin loads only in the RobotIK build that made it (build id checked).
- The plugin does not own world entities; find them by id each time.
- SI units, z up; typed handles (`Revolute`, `Prismatic`, `Seconds`, `Radians`).
- Code must work headless (`SceneView*` null, no camera image; see
  `perception.blind`).
- Errors from files and user input are a `Result` (`ROBOTIK_TRY`,
  `ROBOTIK_TRY_ASSIGN`), never an exception: the YAML readers have no
  try/catch, `ConfigNode::checkKeys` and the typed reads return a `Result`.
  A mistake in the code (unknown joint name written in C++) throws, and the
  name says it: a lookup by a name written in C++ starts with `require`
  (`joints.requireRevolute("j")`, `robot.requireFramePose("tool0")`,
  `cell.requirePose("box")`, `cell.requireQuery(...)`); the handle version
  found once does not throw (`robot.worldPose(link)`). `add` refuses a
  duplicate name (`@throws`). `robotik::require(result)` takes a value
  where throwing is the way (a plugin method, a test).
- Plain structs are data: what acts on them is a free function
  (`satisfies(version, need)`, not `version.satisfies(need)`). Classes keep
  their methods. Events are `Signal`s (`SkillScheduler::changed`,
  `Simulation::ended`), not polling.
- Hot loops use handles found once (`JointId`, `LinkRef`, `ResourceId`),
  never a name. Measure before changing a storage
  (doc/concepts/architecture.md, "Fast data").
- Only `Task/Skills/SkillNodes.hpp` includes BlackThorn; other public
  headers name tree types with `TreeTypes.hpp` and answer `robotik::Status`
  (checked by the compiler in `tests/Common/TestHiddenBlackThorn.cpp`).
- BlackThorn `Timeout`/`Delay`/`Wait`/`Cooldown` read the simulated time
  (`bt::Tree::clock`, set by `RoboticCell` and `PlanExecutor`). A tree built
  elsewhere keeps the wall clock unless you give it one.

## Style

- `.clang-format` (Google based, braces on their own line, 4 spaces).
- Parameters `p_name`, members `m_name`, constants `UPPER_CASE`.
- Doxygen `///` comments (`///<` on the same line); class banners `// ****...`, method banners `// ----...`. File-local functions are `static`, with a `///` comment.
- A public class answers why / when / how / guarantees / limits / why this
  choice, with an `@code` example from a test or a demo; docs follow the
  writing rules of the `robotik-docs` skill (Diátaxis kinds, ASD-STE100
  rules, no signs of machine writing).
- A header starts with `@file`: what it is for, in everyday words, then
  `@par Contract` (promises, expects, errors). Define a technical word or
  link doc/reference/glossary.md.
- Every source file starts with the SPDX header
  `GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial` (copy it from a neighbour).
- Commit subject: imperative sentence, no prefix (e.g. "Give sensors a
  measurement contract"). Docs: simple English for French readers.
