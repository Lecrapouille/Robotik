# AGENTS.md — context for coding agents

RobotIK: C++20 library to write a robot mission once and run it in a 3D
simulator, headless (CI), in RL environments, and later on hardware. It glues
MuJoCo (physics), Pinocchio (kinematics), Compages (ECS world + rendering),
BlackThorn (behavior trees) and Kairos (PDDL planner). Linux only, plain `make` (no CMake).

Read only what the task needs. Load a skill instead of reading the docs:

| Task | Skill (`.claude/skills/`) | Long doc |
|------|---------------------------|----------|
| Create / change a plugin (C++ + manifest) | `robotik-plugin` | doc/04-Your-First-Plugin.md |
| Write world / scenario / experiment YAML, assertions | `robotik-scenario` | doc/03-Worlds-and-Scenarios.md |
| Write a skill, a behavior tree, handle faults | `robotik-skill` | doc/05-Skills-and-Behavior-Trees.md |
| Plan from a PDDL goal (domain, observers, bindings) | `robotik-skill` | doc/11-PDDL-Planner.md |
| Change the simulator interface (panels, layouts, scenario editor) | `robotik-simulator` | doc/02-Getting-Started.md |
| Edit the documentation | `robotik-docs` | doc/README.md |
| Find a class or method | — | doc/07-API-Reference.md |
| Change the core | — | doc/06-Architecture.md |

## Layout

```text
include/Robotik/<Module>/   public headers; src/Robotik/<Module>/ implementation
  Planning/  PDDL symbols, PlanningDomain (observers, bindings), Kairos engine,
             PlanExecutor (plan -> behavior tree, checks, replans)
  Runtime/   Simulation (step loop, truth, end of run), Mission, SkillScheduler,
             ResourceManager, FaultInjector, Metrics, RobotContext, RunReport
  Robot/     Robot, RobotSession, JointSet (SoA), actuators
  Scenario/  World/Scenario/Experiment YAML parsing, AssertionEngine, Trajectory
  Plugin/    PluginABI.h (C border), PluginAPI.hpp (C++ helper), catalog, session
  Skills/ Sensors/ Perception/ Environment/ Backends/ Assets/ Agents/ ECS/ Scene/ Systems/ Math/
simulator/   Robotik-Simulator (ImGui host)      headless/  Robotik-Headless (CLI host)
plugins/<Name>/  plugin.yaml, Makefile, src/, worlds/, scenarios/, experiments/, data/
plugins/plugin.mk  shared plugin build recipe        data/packs/<id>/  asset packs (URDF)
tests/       GoogleTest; tests/Plugin uses plugins/HelloWorld as fixture
external/    third-party sources (do not edit)      build/  outputs (generated)
```

Plugins: HelloWorld (tutorial + test fixture), PickAndPlace (reference: BT,
faults, RL scenario), Painter, LineFollower, FlyBrain, HumanoidWalk (ONNX policy).

## Commands

```bash
make -j"$(nproc)"                         # core, hosts, every plugins/*/Makefile
make -C plugins/<Name> -j"$(nproc)"       # one plugin; stages build/plugins/<Name>/
make tests -j"$(nproc)"                   # builds and runs build/Robotik-tests
./build/Robotik-tests --gtest_filter='Plugin*'
./build/Robotik-Headless build/plugins/<Name>/scenarios/<s>.scenario.yaml [--seed N] [--trace] [--report r.json]
```

Headless exit code: 0 pass, 2 a run failed, 1 load error. `make` copies
`worlds/ scenarios/ experiments/ data/` over `build/plugins/<Name>/`: edit
sources, not `build/`. First build only: `make download-external-libs`
(core `external/manifest`, then the `dependencies.thirdparties` of each
`plugin.yaml`);
libraries are then built by Make rules, there is no `external/compilation`.

## Rules that are easy to break

- The core and the hosts stay agnostic of the demos: no demo object, joint,
  tool or task in `src/`, `include/`, `simulator/`, `headless/`. The demos
  prove RobotIK fits any robot project. Known leaks and the rule:
  `robotik-plugin` skill, "Core stays agnostic". Report a new leak.

- Skills see only `RobotContext` (robot API + `WorldModel` beliefs). Never
  give them ground truth (`Simulation::pose/query`); that is for metrics,
  assertions, RL reward.
- File names carry their role: `x.world.yaml`, `x.scenario.yaml`,
  `x.rl.yaml`, `x.bt.yaml`, `x.domain.pddl`, `x.trajectory.yaml`,
  `x.experiment.yaml` (doc/03 §1). id = name without `.yaml` and
  `.scenario`, other dots → `_` (`pick.rl.yaml` → `pick_rl`).
- Paths in world/scenario/experiment YAML start at the package root (folder
  of `plugin.yaml`): `world: worlds/arm.world.yaml`. `..` and symlinks are allowed
  but the canonical path must stay in the package; absolute paths are
  refused (`PackagePath.hpp`).
- A library used by one plugin goes in `dependencies:` of its `plugin.yaml`
  (`system:` pkg-config names, `thirdparties:` `{ repo, license, description }`;
  the license is required for the credits) and is built by a rule of its
  Makefile (`PLUGIN_REQUIRES`, `PLUGIN_DEPENDS`, see
  `plugins/plugin.mk`), not in the core manifest nor in `plugin.yaml`.
- Plugins publish metrics (`Mission::measure`); verdicts live in scenario
  YAML assertions. No pass/fail logic in C++.
- `Mission::status` ends a run only when the scenario has no behavior tree
  and no planner.
- Planner: the PDDL domain is the only place of actions and effects (no
  symbolic effects in C++); observers read beliefs, never the truth; only
  `src/Robotik/Planning/Kairos/` includes Kairos headers.
- Names must match: `TARGET_NAME robotik_x` → `librobotik_x.so` =
  `library.entrypoint`; `plugin.id` = `describe().id`; scenario `id` in
  `plugin.yaml` = string tested in `Plugin::setup`. A scenario not listed in
  `plugin.yaml` is refused ("scenario is not in a plugin package").
- Nothing C++ crosses the plugin border except through `PluginAPI.hpp`.
- The plugin does not own world entities; find them by id each time.
- SI units, z up; typed handles (`Revolute`, `Prismatic`, `Seconds`, `Radians`).
- Code must work headless (`SceneView*` null, no camera image; see
  `perception.oracle`).
- BlackThorn `Timeout`/`Delay`/`Wait`/`Cooldown` read the simulated time
  (`bt::Tree::clock`, set by `Simulation` and `PlanExecutor`). A tree built
  elsewhere keeps the wall clock unless you give it one.

## Style

- `.clang-format` (Google based, braces on their own line, 4 spaces).
- Parameters `p_name`, members `m_name`, constants `kName` or `UPPER_CASE`.
- Doxygen `///` comments (`///<` on the same line); class banners `// ****...`, method banners `// ----...`. File-local functions are `static`, with a `///` comment.
- Every source file starts with the SPDX header
  `GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial` (copy it from a neighbour).
- Commit subject: imperative sentence, no prefix (e.g. "Give sensors a
  measurement contract"). Docs: simple English for French readers.
