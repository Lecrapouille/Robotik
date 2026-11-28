# RobotIK roadmap

Goal: a small, intuitive public API (the Compages way), little code in the
plugins, rules checked by the compiler, and an execution of skills that can be
trusted before LLM, JEV and RL work start. Commits are made only on request,
one commit per feature. The history is in `git log`; this file keeps only
what is open, and one line per finished task.

Status: `[ ]` todo, `[~]` in progress, `[x]` done, `[!]` blocked.

A task is done when: its public contract is written (errors, cancel,
timeouts, limits); unit tests pass and one integration test covers the
normal path and one important failure; the logs and metrics explain a
failure; a demo or test shows it; the docs give its dependencies and limits;
`make check` runs its tests.

## 📋 To do

- [ ] Rename the accessors of the public headers to `get`/`set`/`is`/`has`/`count`
  (AGENTS.md, Style), first `include/`, then the plugins and the docs
  (doc/reference/api/README.md). Done: `TeachPendant`, `Robot`, `RobotRuntime`. This leaves the bare names
  of Compages on purpose. In the same pass, replace the bare `double` of a
  physical quantity in a public API by its SI type (AGENTS.md); done:
  `TeachPendant`, `RobotRuntime::measureJoint/placeJoint`.

### Execution and control

- [ ] Joint servo: one PD per joint with the same gains (kp 150, kd 15)
      whatever the inertia, and it damps the speed, not the speed error. It
      lags the reference by kd·v/kp (≈ 0.065 rad) and the shoulder overshoots
      ≈ 0.027 rad when the other joints brake (measured 2026-11-24; kd 30/45,
      velocity feedforward and a decelerating reference do not fix it). Fix:
      gains from the inertia of each joint and gravity/Coriolis feedforward
      (computed torque), or MuJoCo position actuators. Let the YAML set the
      gains of a joint (today only C++ can, `Joints.cpp`).
- [ ] Cell test of a stalled clock (the session clock tests exist).
- [ ] Composite resources: `ResourceManager::group(name, {members})`
      resolved to elementary ids at init, expanded and merged in
      `SkillScheduler::add`, `ResourceLease::CAPACITY` 8 → 16, YAML key
      `resources: groups:`. Start with the first demo that needs it (a
      humanoid that moves its arms and torso while it walks).
- [ ] Writes checked by the compiler: the lease gives a writable view of its
      joints, and nothing else can write. Do it if the runtime check of A7
      still finds mistakes late.
- [ ] Motion blending (several compatible commands on one joint).
- [ ] Tool calibration, with the same scheme as the estimated base.
- [ ] Deterministic planner budget: a number of expansions in Kairos (change
      upstream, `external/` is not edited here), instead of a timeout.
- [ ] Pddl.cpp has its own domain reader: use SExpression (see 19).

### Physics fidelity (old/FIDELITY.md, steps 4-6)

- [ ] `contact` level for the vacuum cup (`ContactGrasp`, adhesion force);
      scenario "the cup lets go of a too heavy object". Today
      `physics.fidelity: contact` is refused at load (`Scenario.cpp`).
- [ ] `fingers` actuator with friction contact, and a common gripper
      interface (`holding()` knows only the vacuum cup). Then remove the
      vacuum assumptions of the core (see 19).
- [ ] Humanoid `abstract` level: a kinematic base that follows the command,
      never falls.
- [ ] Cylinders (`addCylinder`) for the glasses of the DualArm tray (boxes
      today).

### Plugins and demos

- [ ] LineFollower writes its navigation estimate (`estimateBase`,
      `localization: plugin`) and is judged with `base_error`.
- [ ] HumanoidWalk: when the robot falls, the walk skill does not fail (does
      not turn red). Decide when it counts as fallen.
- [ ] Case studies written as lessons: the problem, what you learn, what to
      read first, run it, a tour of the files, variants to try, the next
      demo to read (today 12 to 45 lines each).
- [ ] MicroDuck and HumanoidWalk: a "Learn" submenu (train the policy) next
      to "Pretrained policy"; then harder tasks (meet a person).
- [ ] PickAndPlace PDDL: the tool choice (`swap_tool`) is not planned.
- [ ] New PDDL demo: two arms solve the Tower of Hanoi (DualArm).
- [ ] New demo: peg in a hole with the force/torque sensor (contact,
      tolerance, recovery). Needs the `contact` level.
- [ ] RL: observations that read the truth (PickPlaceControl) declared
      `privileged` in `.rl.yaml` (actor/critic split).
- [ ] RL: a policy is an `Agent` run inside a skill; share the observation,
      inference and action code of MicroDuck between training and run.

### Simulator

- [ ] Joints plot: add the reference and the error, not only the position
      (`RobotPanel.cpp`).
- [ ] Sensor panel: live readings of the IMU, range scanner and
      force/torque sensor (the World view only lists the sensors).
- [ ] Several robots: the Joints and Teach Pendant panels show only the
      first robot (`cell().robot()`).

### Build, tools and API

- [ ] The plugin build id does not see a changed header at the same version:
      a plugin built before a header change can crash instead of being refused.
- [ ] Code coverage target that leaves out `external/`.
- [ ] Open a saved run report or trace in the simulator, without a
      running simulation (the timeline inspects only the current run).
- [ ] Headless: list the plugins found, their versions and why one is
      refused; compare two run reports (`--report` JSON).
- [ ] `RoboticCellDescription` helpers: `robotik::robot("arm", "x.urdf")`,
      `robotik::box("cube", pose)` (see 19).
- [ ] Document what each backend can do and its limits (MuJoCo dynamics and
      contacts, Pinocchio kinematics only, the fidelity levels).

## 🩺 Degraded mode (design of 2026-11-25)

Today a fault fails the running skill, the `Sequence` fails and the run
ends; repairing the resource starts nothing again. Only `Detect` is retried
(`UntilSuccess`). Four questions, four places:

| Question | Where |
|----------|-------|
| What is measured? | Sensors and drivers: image received and its stamp, `holding()`, resource availability |
| Which problem is likely or confirmed? | Monitors: no frames, frozen image, grasp not confirmed, load lost during a carry |
| What can the robot still do, and with which conditions? | Capability supervisor, with rules |
| What now? | Behavior tree or planner: try again, other method, wait, safe stop |

Rules: the scheduler keeps the authority on the resources (it fails fast,
with the reason); the supervisor informs, the tree or the planner decides.
Everything comes from measures and beliefs, never from the truth. The core
stays agnostic: failure codes, components and rules of a tool live in the
plugin. "Not seen" is not "broken": an empty scene is not a camera fault,
`holding() == false` after a grasp is "grasp not confirmed" (bad position,
porous object, leak, pump, sensor), not "cup broken". Losing the camera
while holding a part does not open the cup.

- [x] 1. Typed failures: a skill returns `fail(code, why)` (code chosen by
      the plugin, not a core enum); the scheduler keeps it per run
      (`failure(RunId)`, trace); the tree reads `<skill>.failure`.
- [ ] 2. Monitors that last the whole run: camera without frames or with a
      frozen image; load lost during a carry (holding went false with no
      Release). A diagnosis has a component, a state (`ok`, `suspected`,
      `confirmed`, `unknown`), a time and a source; no information is
      `unknown`, never `ok`.
- [ ] 3. Valid beliefs: an action says how recent and precise a belief must
      be (the planner has `max_age`; give it to tree conditions too).
      "Seen once" does not allow to go on forever.
- [ ] 4. PickAndPlace recovery tree, in place of the `_faults` scenarios:
      grasp tried again a limited number of times after a new approach; a
      dropped cube is detected and picked again; wait for a repaired
      resource with a timeout; reach a known pose from a recent belief
      when the camera is lost; safe stop (Home, else Stop) with the reason.
      Faults injected from the simulator panel take the same path.
- [ ] 5. Capability supervisor: diagnoses → capabilities with their
      conditions ("detect: no", "move to a known pose: yes if the belief is
      younger than 2 s", "insert with visual correction: no", "carry a held
      part: yes"), and the explanation read from the same rules. C++ rules
      first, behind a `Reasoner` interface; the same capabilities feed the
      tree conditions and PDDL predicates (`(can ?capability)`). The
      consequence depends on the job: a camera may be needed by one job and
      not by another.
- [ ] 6. Optional Prolog adapter of `Reasoner` (SWI), from
      `BacASable/Prolog/DegradedModeSystem`: add the unknown state, suspected
      vs confirmed, a time and a source per fact, capabilities with
      conditions, explanations from the capability rules.
- [ ] 7. Other method when a capability is lost: another tool
      (`swap_tool`), another camera, or a new plan.

## 🧭 Architecture note of 2026-10-11 (version 1.0 for researchers)

Source: the note `2026-10-11-Architecture logicielle Robotik.md` (deleted
2026-11-25, its open points are here). Already in the code:
plugin manifests and third parties, shared asset packs with a version and a
license, the plugin owns no world entity, world / scenario / experiment
files, assertion phases (`always`, `end`, `within`, `sequence`, `stop`),
`pose_error`, `joint_limits_respected`, collision count, trajectories in
their own files, RL `reset/step` with termination apart from truncation,
measurement contract of the sensors (frame, stamp, noise, latency; the IMU
measures the specific force), detectors apart from sensors, truth hidden
from the skills. What is left:

- [ ] Several flies in one world (FlyBrain has one fly): one environment
      holds N agents; do not mix it with N environments.
- [ ] Experiment cases that start from a saved physical state (joints,
      speeds, sensor state, random state), not only from the world file.
- [ ] Demo "reach a target" learnt by RL: joint state first, then noisy
      observations; compare seeds and policies.
- [ ] Demo "peg in a hole" (see Plugins and demos).
- [ ] Version 1.0 checklist, to check one by one: stable skill and
      execution API; RL contract tested without GUI; known limits of each
      backend written down (see Build, tools and API).

## 🧩 Petri nets (note of 2026-10-28)

Source: the note `Intégrer les réseaux de Petri.md` (deleted 2026-11-25).
Work in `Robotik-grafcet`
(TimedPetriNetEditor linked in `external/`). Start after the scheduler is
reliable alone (arbitration A1-A12 done). Rules: the editor and RobotIK use
the same firing rules, not the same loop; no Dear ImGui in the shared
library; RobotIK does not depend on the editor application; Petri nets do not
replace the behavior trees (a net says when a skill may start and follows its
result).

- [ ] Step 1, in TimedPetriNetEditor: one contract for the simulator and the
      runtime (coloured and timed nets): same reference tests, marking
      snapshots, firing events, resume, rule for transitions enabled at the
      same time.
- [ ] Step 2, RobotIK adapter: load a net from a plugin, bind a transition
      to a skill, take and give back resources (also on failure, cancel and
      timeout; give the motors back only when the controller has stopped,
      as `Stopping` does).
- [ ] Step 3, one cell: a robot, a camera, a part; two skills that share
      the camera; one fault and a controlled restart; open the same net in
      the editor to see the marking and replay the run.
- [ ] Step 4, flowshop: machines, conveyors, buffers and their capacities
      (capacity of a resource, of a buffer, number of parallel machines are
      three different things); parts as coloured tokens; flow time, rate,
      bottlenecks with (max,+); the flowshop is a view of the cell, not a
      second model.
- [ ] Step 5, planner and GEMMA: compile a PDDL plan to an intermediate form
      (actions, preconditions, dependencies, expected results, resources),
      then to the production net; GEMMA gives the run modes (safety,
      production, control nets with their own life cycle); test stops,
      restarts and gaps between plan and run.

## 📈 Performance and API (audit of 2026-11-21)

Kept as written by the audit: they are open plans with their evidence.

### 15. Performance baseline — P1, before storage changes

The audit ran the existing disabled sensor benchmark: 64 IMUs, 8 lidars,
32 beams per lidar, 2,000 steps, physics disabled, debug binaries. One native
run reported 292.269 microseconds per step. This is a diagnostic sample,
not a release baseline or a comparison with the older architecture document.

Callgrind collected only `RobotRuntime::step` and its callees. Cache and
branch figures below are simulated, not hardware counters. The model used
32 KiB I1, 48 KiB D1 and 32 MiB LL caches, with 64-byte lines.

| Event | Collected value |
|-------|-----------------|
| Instructions | 7,312,922,611 |
| D1 read + write misses | 19,369,188, about 0.43% of data references |
| Last-level data misses | 54,941 |
| I1 misses | 209,562,517, about 2.87% of instruction references |
| Conditional branch mispredictions | 5,266,243 / 187,969,974 |
| Indirect branch mispredictions | 139,503,590 / 208,999,754 |

The indirect count includes shared-library calls and other indirect transfers.
It does not measure virtual dispatch alone. Debug code expands small maths,
container and unit operations into many calls. Do not use this profile to
claim that vtables cause the total branch cost.

| Callee, inclusive | Share of instructions | Share of D1 read misses |
|-------------------|-----------------------|-------------------------|
| `Robot::propagate` | 30.32% | 84.31% |
| `Imu::sample` | 28.06% | 8.00% |
| `Imu::deliver` | 20.37% | 1.35% |
| `RangeScanner::sample` | 11.59% | 1.22% |
| `World::update` | 5.69% | 2.38% |
| `RangeScanner::deliver` | 2.96% | 0.71% |

Inclusive rows contain their callees; they are not exclusive CPU times.
The 72 sensors perform 288,000 sample/delivery virtual calls in this run.
That count is much smaller than the total indirect branch count. Preserve
the plugin extension points until an optimised comparison identifies a cost.

Measured `sizeof` values on this build, excluding heap allocations:

| Type | Bytes | Storage issue to investigate |
|------|-------|------------------------------|
| `Sensor` | 72 | Timing and name share the base object |
| `Imu` / `ImuConfig` / `ImuReading` | 616 / 184 / 136 | Each device also owns a latency queue |
| `RangeScanner` / `RangeScan` | 472 / 104 | Range samples allocate a vector |
| `WorldObject` | 104 | Names share each record with numeric beliefs |
| `ConditionNode` | 136 | Strings, callback and child vector in every node |
| `SkillDescription` | 120 | Configuration separate from scheduler state already |
| `ecs::Shape` | 80 | Names and rendering colour travel with geometry |
| `Random` | 8 | Compact state; keep this design |

- [ ] Add a repeatable benchmark target using `release-with-debug`, not
      `fast`: preserve floating-point and deterministic replay semantics.
      MyMakefile already enables LTO in optimised modes; inspect the emitted
      commands before adding another optimisation flag.
- [ ] Measure physics alone, kinematics alone, sensors, scheduler, perception,
      assertions, and a complete headless scenario separately.
- [ ] Cover 1/2/8 robots, 0/64/256 IMUs, 1/8 lidars, 32/360 beams,
      10/100/1,000 objects, and 10/100/1,000 registered skills. Distinguish
      registered skills from active skills. Keep fixtures deterministic.
- [ ] Warm up before timing. Run at least five samples; report the median,
      tail step time, allocations, peak memory and scenario verdicts. Record
      compiler, flags, CPU, dependency commits and object counts.
- [ ] Collect hardware cycles, instructions, branches, branch misses and
      available cache events when permitted. `perf` is absent in this audit
      environment; the simulated cache profile cannot replace these counters.
- [ ] Compare virtual and batched versions with identical work, noise streams,
      publication, traces and physics. Attribute dispatch, allocation and
      locality changes separately; do not compare different fidelity levels.

Reproduce the diagnostic cache profile, with Valgrind available:

```bash
valgrind --tool=callgrind --cache-sim=yes --branch-sim=yes \
  --collect-atstart=no '--toggle-collect=robotik::RobotRuntime::step*' \
  --callgrind-out-file=/tmp/robotik-sensors.callgrind \
  ./build/linux-gcc-debug/bin/RobotIK-tests \
  --gtest_also_run_disabled_tests --gtest_filter='SensorBench.*'
callgrind_annotate --auto=no --inclusive=yes --show=Ir,D1mr,D1mw \
  /tmp/robotik-sensors.callgrind
```

### 16. Remove repeated work and allocations — P1

| Path and evidence in the code | Change to compare | Required check |
|-------------------------------|-------------------|----------------|
| `Robot.cpp`: `sense` updates the shared world once per robot | Propagate all robots, update the world once, then sample all sensors | Cameras, frame sources and detectors see the same current transforms; standalone robot stepping still works |
| `Robot.cpp`: `propagate` publishes joints and base on every step | Separate required FK from ECS publication; update unchanged data only when consumers allow it | Never skip sensor history, timestamps or noise samples merely because the robot is stationary |
| `RangeScanner.cpp`: each sample allocates boxes and ranges, recalculates beam trig, scans all shapes | Reuse scratch buffers, precompute local beam directions, share geometry for one world step | Preserve decorations, dynamic objects, range limits, mounts, reset and latency |
| `MujocoBackend.cpp`: `contactList` creates a vector and body-name strings; truth queries can request it repeatedly | Build one internal contact snapshot per physics step, using numeric body ids; expose a span for that step | Invalidate on step/reset/recompile; preserve public reports and contacts involving the floor |
| `RoboticCell.cpp`: `syncBodies` asks `bodyPose(string)` for each body; MuJoCo resolves names | Resolve a backend body handle once and batch pose reads | Handles become invalid when the compiled model changes |
| `RigidGrasp.cpp`: scans all actuators with RTTI, then calls `findObject` for every candidate | Bind typed gripper and body handles inside the core; index graspable records | A plugin still owns no world entity; core handles are validated or rebuilt after reset |
| `AbstractGrasp.cpp`: full shape scans for nearest object and support | Reuse a compact geometry view; add spatial indexing only when object counts justify it | Preserve selection order and ties, placement and container rules |
| `Noise.hpp`: every sensor uses a deque even at zero latency | Deliver zero-latency samples directly; compare a reused ring for bounded positive latency | Never discard delayed frames silently; define overflow and variable-step behaviour |
| Sensor `deliver`: copies samples into ECS; each lidar scan owns ranges | Reuse publication capacity or provide immutable snapshots with defined lifetimes | A queued reading cannot alias mutable scratch storage; consumers retain valid samples |
| `Scheduler.cpp`: full skill scans and two sorts each tick | Reuse active/waiting ids and invalidate ordering on request, priority or state changes | Keep priority, FIFO ties, preemption, cancellations and event order |
| `RoboticCellTruth.cpp` and `Metrics.cpp`: new metric table each step, linear string lookups | Register metric ids once and reuse numeric values; bind compiled conditions to ids | Preserve missing-metric semantics and plugin metric registration |
| `Condition.cpp`: evaluation formats explanatory strings during checks | Separate compact evaluation from report formatting; format when observed or requested | Live UI, assertion events and final diagnostics stay correct |

`PlanExecutor::observe` constructs candidate tuples and id strings when called
for planning or history. It is not automatically a per-tick bottleneck. Measure
planning and frequent replanning separately before caching grounded candidates.

The lidar's brute-force box loop costs O(beams × objects) per scanner. Buffer
reuse removes allocations but does not change that scaling. Benchmark a shared
broad phase separately if large worlds make that loop dominant.

### 17. Reduce dispatch and improve locality where measured — P2

- [ ] Preserve `JointSet` numeric arrays. The MuJoCo member already caches
      numeric qpos/dof/actuator bindings. Avoid replacing these with name maps.
- [ ] Test built-in sensor batches after task 16. `DeviceSet` currently owns
      separately allocated polymorphic devices. A pointer vector is contiguous;
      the sensor objects themselves are not. Compare a contiguous batch per
      built-in sensor type and a polymorphic fallback for plugin-defined types.
- [ ] Place sampling times, availability, frame indices, current values and
      noise state in the batch's frequently used data. Keep names, manifests
      and diagnostic text outside the numerical loop. Use AoS or SoA according
      to which fields that loop actually reads; do not assume SoA always wins.
- [ ] If lidar backend dispatch is visible, compare one batch raycast call per
      scan with the current virtual call per beam. Keep the hardware and custom
      backend extension contract. A function-pointer table still dispatches
      indirectly and does not itself solve scattered storage.
- [ ] Reuse a typed list of grippers instead of repeating `dynamic_cast` in
      both grasp models and penetration tracking. Rebuild it when devices change.
- [ ] Keep `Mission`, `Skill`, `RobotBackend`, `Detector` and `PlanningEngine`
      as extension points unless their own profiles justify a change. Calls
      once per robot/skill/frame differ from calls once per beam or object.
- [ ] Keep scheduler state separate from skill descriptions, as it is today.
      Compare active-id lists before adding another SoA or an ECS scheduler.
- [ ] If beliefs dominate, retain compact sequential values and separate the
      string-to-id index. A vector scan can beat hashing for small worlds.
- [ ] If condition evaluation dominates, replace per-node child vectors with
      offsets into one child-index array. Separate source text and callbacks
      from small operation records. The current flat node vector still has
      per-node heap allocations and 136-byte records.
- [ ] Evaluate shape data used by scans separately from names and colours.
      Do not duplicate the complete world in a new registry. One immutable
      per-step geometry view can serve scanners and grasp calculations.
- [ ] Measure false sharing before padding or aligning RL worker state.
      Keep each environment's mutable world independent. Do not add parallel
      sensor writes to a shared EnTT registry without a publication design.

Acceptance: publish before/after step times, allocations, cache/branch evidence
and code-size delta. Keep a change only when its benefit exceeds measurement
variation and its maintenance cost. Do not promise a speedup percentage now.

### 18. Privatise headers with explicit migration dependencies — P1

All paths below start at `include/RobotIK/`. Private means moved to `src/RobotIK/`
and absent from the installed SDK. Removing an umbrella include is not enough.
Internal tests and the two hosts may include private headers without making
them part of the supported user API.

These three headers have no include consumer among current public headers,
plugins or hosts. Their direct consumers are core sources and internal tests:

| Header | Proposed destination | Migration |
|--------|----------------------|-----------|
| `Common/File.hpp` | `src/RobotIK/Common/File.hpp` | Update four core source includes |
| `Common/SExpression.hpp` | `src/RobotIK/Common/SExpression.hpp` | Update core parsers and its internal tests |
| `Task/Grasp/GraspModel.hpp` | `src/RobotIK/Task/Grasp/GraspModel.hpp` | Keep the forward declaration and out-of-line cell destructor |

The following headers need consumer changes or a split before moving:

| Header | Target status | Dependency to remove first |
|--------|---------------|----------------------------|
| `Robot/Kinematics.hpp` | Private implementation | Replace public `Robot::getKinematics()` with robot-level frame/axis queries; migrate TeachPanel, TeachPendant and Camera; retain public `IkGoal` data separately |
| `Robot/Backends/MujocoBackend.hpp` | Private concrete implementation | Extract physics options and rotor specifications used by Walk/Duck; provide session construction through data; retain `RobotBackend` for custom drivers |
| `Robot/Components/Queries.hpp` | Private lookup helper | Replace PickPlaceSkills' ECS lookup with belief access; keep privileged lookups inside the core |
| `Robot/Components/Objects.hpp` | Split, then private ECS components | Migrate ArmSkills/PickPlaceSkills to beliefs; expose render shape data for SceneView without publishing physics markers; migrate Fly rendering and simulator inspectors |
| `Robot/Components/Robots.hpp` | Private components | Share privately between joint publication and TeachPanel; keep public sensor readings in `Measurements.hpp` |
| `Task/Grasp/ContainerBounds.hpp` | Private simulator geometry | Migrate PickPlaceSkills to belief geometry, or expose only the pure geometry functions it needs |
| `Task/Planning/PlanExecutor.hpp` | Private controller | Replace mutable `planner()` exposure with a const inspection view for UI, reports and tests; keep planner setup in description/domain APIs |
| `Task/Planning/PlanGraph.hpp` | Private graph algorithms | Extract immutable inspection data currently returned by `PlanExecutor::order()` |
| `Task/Planning/Backchaining.hpp` | Private algorithms | Extract the tree inspection data returned by `PlanExecutor::backchain()` |
| `Task/Condition.hpp` | Split | Keep Evaluation and custom query registration public; hide AST nodes, compiler vocabulary and evaluator implementation |
| `Task/Faults.hpp` | Split | Keep Fault/RandomFault descriptions public; hide FaultInjector after replacing mutable cell access with configuration |
| `Package/Scenario/Assertions.hpp` | Split | Keep AssertionSpec, Check and verdict data; hide AssertionEngine and its probe callback |
| `Package/Scenario/World.hpp` | Split | Keep `loadWorld` only if standalone world loading remains supported; hide YAML-node interpretation helpers behind the scenario reader |
| `Package/Scenario/FileRole.hpp` | Private host/file helper | Migrate simulator file browser and internal tests; no current plugin directly includes it |
| `Package/Scenario/PackagePath.hpp` | Private after public path service exists | FlyBrain's CLI currently includes it; preserve package-constrained resolution through that service |
| `Package/Plugin/PluginHost.hpp` | Private host implementation | Hide behind session ownership; preserve public Host/Canvas wrappers in PluginAPI |
| `Package/Plugin/PluginManager.hpp` | Private dynamic loader | Move lifecycle state used by PluginSession into its implementation |
| `Package/Plugin/PluginCatalog.hpp` | Split | Keep package/scenario/license metadata for browsers; hide scanner implementation after extracting metadata types |
| `Package/Plugin/PluginSession.hpp` | Private orchestration after task 19 | Migrate Headless and App to the owning simulation entry point and a private host adapter |
| `Task/TreeTypes.hpp` | Confine to optional BT extension | Move factory/tree access out of the default cell interface; retain extension declarations used by SkillNodes |

`AssetPack.hpp` should remain public as metadata: `AssetResolver::packs`, `find`
and `require` expose it. Its load function can move to an optional asset-author
header. Hiding the struct now would break the resolver's useful inspection API.

Keep the following extension APIs public, outside the default umbrella where
possible: `PluginABI.h`, `PluginAPI.hpp`, `RobotBackend.hpp`, `SceneView.hpp`,
`Sensor.hpp`, concrete sensor headers, `Detector.hpp`, `PlanningEngine.hpp`,
`PlanningDomain.hpp`, `Pddl.hpp`, `Skill.hpp`, `SkillNodes.hpp`, and RL headers.
The project promises custom drivers, sensors, planners and plugins. Making
these interfaces private would remove that capability.

Keep ordinary application data and operations public: Result, Math, Seed,
Signal, configuration, version/asset metadata, Robot, joints, devices,
resources, opaque link handles, measurements, beliefs, Mission, skill
registration, RoboticCellDescription, scenario/experiment/trajectory data,
Simulation and run reports. Reduce their transitive includes separately.

- [ ] Add a consumer include matrix before each move. Direct include counts
      do not detect every transitive or external user; identify API breaks.
- [ ] Compile Robot, Task, Package, plugin and optional extension examples
      against a staged installed SDK with no `src/` include path.
- [ ] Keep private test includes explicit. Retain layer checks after moves.
- [ ] Measure supported headers, transitive headers, preprocessed bytes and
      consumer compile time. Six convenient entry headers do not mean six
      total public headers. Do not hide includes merely by moving them to
      an installed `detail/` directory.
- [ ] Use opaque ownership only where it removes a real dependency. Do not
      add one heap-allocated PIMPL to every joint, reading or condition node.

### 19. Compact entry points and actual code removal — P1

- [x] Add `openSimulation(path, options)` using existing package and plugin
      services. Done 2026-11-28: `StandaloneSimulation::open(scenario or
      file, logic)` and `openPlugin(file, setup)`, which owns the world, the
      simulation and the plugin library in destruction order, and `step`
      calls the plugin. Used by the headless host (one library load per
      run), the LineFollower app and the tests. Own the world, mission and plugin library in destruction order.
      Keep plugin updates, events, custom clock handling and reports consistent
      with the two existing hosts. Decide its exact return type during design.
- [ ] Expose step, run, reset, time, completion and report operations for the
      common case. Keep rendering, planner inspection and extension registration
      in optional headers. Return `Result` for loading failures.
- [x] Keep `openRobot` and `openRoboticCell`; hide their worlds behind safe
      ownership. Public unique_ptr members currently let callers break lifetime
      invariants. Review move assignment as well as destruction order.
      Done 2026-11-28: `StandaloneRobot` and `StandaloneCell` keep the
      world and the robot (cell) private, move without assignment;
      `RobotOptions` became `StandaloneRobot::Setup { tool, home,
      std::optional<MujocoOptions> physics }`.
- [ ] Add small free functions to construct robot/box specifications. Reuse
      RoboticCellDescription rather than introducing a second builder model.
- [ ] Reduce RobotIK.hpp dependencies. It currently includes MuJoCo, RL and
      SkillNodes, exposing BlackThorn through the umbrella. Keep the complete
      extension API reachable explicitly.
- [x] Read plugin manifests with ConfigNode (f6c6026, also pack.yaml).
      Delete the duplicate YAML helpers after preserving diagnostics and
      unknown-key checks.
- [ ] Reuse the SExpression reader for PDDL domain syntax. Keep PDDL case rules
      and semantic validation; remove the redundant Reader/Sexpr implementation.
- [x] Share physical model assembly between StandaloneRobot::open and RoboticCell. Replace
      hardcoded flange/tool_mount assumptions with declared attachment data.
      Done 2026-11-28: `ToolMount { urdf, flange, mount }` read by the
      kinematics, the runtime and `MujocoBackend::loadRobot`; the cell fills
      it from `attach:` (`toolMountOf`). Also: `SimulationEnvironment` holds a
      `StandaloneCell`, `TestTeachPendant` builds its arm with
      `StandaloneRobot::open`.
- [ ] Remove Painter-specific openCanvas/ink operations from SceneView once a
      generic dynamic-texture extension serves their consumers. Remove vacuum
      assumptions only when a generic gripper contract has a second use.
- [ ] Extract internal cell construction/controller helpers only when they
      delete duplication or isolate an invariant. Splitting files alone does
      not reduce code size. Preserve the existing scheduler and single physics.
- [ ] Add native CI checks for tests, layers and SDK examples. The current
      workflow builds web variants but does not run these native checks.

Acceptance: common examples build against the staged SDK, both hosts use the
same lifecycle, and scenario verdicts remain covered. Count deleted/added code
lines without comments, installed headers, binary text size and example size.
Keep each feature in its own reviewable commit; no implementation is done by
this roadmap update.

Order: 15 optimised baseline → 16 repeated work → 17 measured storage
experiments. 18 headers and 19 entry points can start at any time. Recheck
the consumers before each move: the include inventory is a snapshot.

## 🧱 Ownership and names (review of 2026-11-27)

The public split "what a skill sees / what runs the robot" exists in the
methods, not in the storage: `Robot` holds the ECS world, the backend, the
entities and the true base as `protected` members, and `RobotSession`
derives from it only to get write access. There is no polymorphic family
there. Other classes have the same kind of problem. One commit per line.

### 20. The robot boundary — P1

- [x] 20.1 `RobotContext::team` is `std::span<Robot* const>`: the pointer is
      const, the robot is not, so a skill can command another robot and
      bypass the scheduler leases. Use `std::span<Robot const* const>` and
      `std::vector<Robot const*>` in `RoboticCell`. The demos read it const
      already.
- [x] 20.2 `RobotModel`: what does not change after the load (name, joint
      names, types and limits, frames, tool, q/v indices, the Pinocchio
      model), shared as `std::shared_ptr<RobotModel const>`, read with
      `Robot::getModel()`. Done without the Pinocchio model: it stays in
      the `Kinematics` of each robot (model and data together), so two
      robots of one URDF do not share it yet. Mounting or removing a tool changes the joints,
      so it makes a new model (check the callers first). A `TeachPendant`
      waypoint keeps the joint names; `compatible()` compares the names and
      their order, not only their count (two robots with six joints pass
      today).
- [x] 20.3 Rename `RobotSession` to `RobotRuntime` and make it own a `Robot`
      by value (composition, `friend class RobotRuntime` in `Robot`). Move to
      the runtime: the `World&`, the root entity, `m_world_from_urdf`, the
      backend, the true base, the URDF and tool loading, `propagate()`.
      `Robot` becomes `final`, without `protected` nor `virtual`, and does
      not include the Compages world. `RobotBackend`, `GraspModel` and
      `RoboticCell` take a `RobotRuntime&`; the skill gets
      `runtime.robot()`. `OpenRobot.hpp`: `OpenedRobot` holds a
      `RobotRuntime`; with task 19 its members become private.
      Several robots stay in `RoboticCell` (`robots()`), not in the runtime.
- [x] 20.4 No entity on the skill side: `JointSet::add(name, Entity)`,
      `JointSet::link()` and `m_links` move to the runtime (one entity per
      `JointId`). The sensors keep their `m_holder` private; the runtime
      writes the reading to the entity.
- [x] 20.5 Sensors read only for skills. `Robot::getSensors()` returns a
      `SensorList&`, so a skill can call `Camera::source(nullptr)`,
      `seed()` or `invalidate()`. Give `SensorList const&` to the skill,
      make the reading methods `const` (caches `mutable`), and reach
      `source`, `seed`, `invalidate` only from `RobotRuntime`. A real
      command of a sensor (trigger a capture, pan a camera) is an actuator.
- [x] 20.6 `DeviceList` / `DeviceSet`: the base class holds the devices as
      `protected` and the derived owner adds `add`. Done as one class
      `DeviceSet` whose `add` is private (`friend class RobotRuntime`);
      `RobotRuntime::addSensor` / `addActuator` are the only way to add.
- [x] 20.7 `PinocchioBackend` is not a `RobotBackend`: rename it
      `Kinematics` (with the privatisation of task 18). `SceneView` gives
      rendering hooks, it is not a backend: move it out of `Backends/`.
- [x] 20.8 `RobotBackend` is the hardware abstraction layer of one robot
      (`attach`, `reset`, `step`: commands to the actuators, encoders to
      `measureJoint`), but it also measures and changes the simulated world
      (`raycast`, `wrench`, `bodyPose`, `contactList`, `contacts`, `weld`,
      `placeBody`), that a hardware driver leaves empty. Move these to a
      `PhysicsWorld` interface, only in simulation: `MujocoBackend`
      implements both, a kinematic model or a driver only `RobotBackend`.
      Simulated sensors, the truth and `ScenarioLogic::simulate` use a
      `PhysicsWorld*` (null on hardware); a hardware sensor reads a source
      given by the driver, as `FrameSource` for the camera. After 20.3.
- [x] 20.9 Hardware contract of `RobotBackend`, written before a first
      driver:
      - Time: in simulation `step(dt)` moves the world by `dt`; on hardware
        time moves alone and `step` writes and reads at the bus period. The
        contract says both, and what a late step does.
      - Errors: a lost link or a drive fault is a `Result` or a fault
        (`Faults`, `getHealth()`), never an exception inside a step.
      - Safety: emergency stop and hardware limits are in the driver, not in
        the skills.
      Done: the contract is in `RobotBackend.hpp` and
      doc/concepts/architecture/03-robot-runtime.md; `RobotBackend::Health`,
      and a backend that is not healthy fails the actuator resources
      (`tests/Robot/TestBackendHealth.cpp`).

Acceptance: `make check`, `make check-layers`; compile-time tests (as
`TestHiddenBlackThorn.cpp`) that code holding a `Robot&` or a
`RobotContext&` cannot call `Camera::seed`, write a joint of a team robot,
nor reach an entity.

- [x] 20.10 Follow-up of the review of `Robot.hpp` (2026-11-28): the three
      fidelities in one header `Robot/Fidelity.hpp` (`Fidelity` renamed
      `PhysicsFidelity`); `JointPosture` is two tables, `angles` in
      `Radians` and `lengths` in `Length` (the world reader classes the
      numbers of `home:` by the joint kinds of the URDF, `readJointTypes`);
      the robot clock is `RobotRuntime::getTime` (a skill reads
      `RobotContext::time`); `Robot::m_kinematics` and `m_model` are values; `RobotModel` is nested as `Robot::Model` (`sameJoints` removed, no caller; `readJointTypes` in `Joints.hpp`).
      Kept: `LinkRef` (4.7 times faster steps), `m_serial` in `Robot` (it
      names an instance, not a model), the private constructor of `Robot`.

### 21. Names and ownership in the other layers — P2

- [ ] 21.1 `struct Scenario : RoboticCellDescription`: inheritance of data.
      A scenario has a cell and a verdict: `Scenario { RoboticCellDescription
      cell; ... }`.
- [ ] 21.2 `PlanExecutor` prepares, compiles and runs plans, and carries the
      state of two planning methods. First extract the domain and problem
      preparation (read, check, build the PDDL problem); then move the PA-BT
      growth state next to `Backchaining.hpp`. Step tracking and replanning
      stay in the executor.
- [ ] 21.3 `RoboticCell` builds what it runs (`buildDevices`,
      `connectBackend`, `addRigidProps`, URDF links). A cell builder makes
      the parts from the `RoboticCellDescription`; the cell keeps their
      lifetime and the step order. Same work as "share physical model
      assembly between openRobot and RoboticCell", now StandaloneRobot::open (task 19): do them
      together.
- [ ] 21.4 Plugin classes: `PluginManager` loads one library (a
      `PluginLibrary`); `PluginSession` is one more "session"; the plugin
      side `Host` and the host side `PluginHost` are two names for one
      thing. Rename what task 19 keeps public; what it makes private can
      wait.
- [ ] 21.5 `Task/Scheduler.hpp` holds `SkillScheduler`: rename the file.
- [ ] 21.6 Simulator `UiState` mixes three lifetimes: the application
      (preferences, layout), the loaded package (open files, caches,
      experiment) and the episode (manual faults, run history). Split it in
      three; the episode state is made again at each load, so nothing of the
      previous scenario stays.

### 22. Architecture documentation — after 20.3

- [x] Move `doc/concepts/06-architecture.md` to `doc/concepts/architecture/`:
      `README.md` (index, principle, three layers, why), `01-decision-path.md`
      (decision path, one step, time), `02-robot.md` (model, state,
      backend, JointSet, fast data), `03-robot-runtime.md` (new: `Robot`,
      `RobotModel`, `RobotRuntime`, who sees what), `04-resources.md`,
      `05-truth-and-belief.md` (sensors, WorldModel, who owns which state),
      `06-hosts-and-layout.md`. Update the links (AGENTS.md, skills,
      concepts README) and the rules of AGENTS.md; `make check-links`.

Order: 20.1 → 20.2 → 20.3 → 20.4 → 20.5 → 20.6 → 20.7 → 20.8 → 20.9 → 22; then 21 in any
order (21.3 with task 19, 21.4 after task 19).

## 🔭 Later

- LLM integration: skills described with their parameters and results, one
  command queue read by the simulation thread, a receptionist demo (goal,
  direct skill, cancel, "why can you not move?").
- JEV: check its SDK contract first.
- Motion planning with collisions (OMPL or MoveIt 2), kept apart from the
  PDDL task planner.
- Perception: generic detector adapters (OpenCV, AprilTag, PCL/Open3D).
- Force control and compliance.
- Optional ROS 2 bridge; import of robots and scenes from standard formats;
  imitation learning.
- Humanoid social demo (a person comes, the robot guides them): write the
  steps first; needs a human model, detection (YOLO), navigation, LLM.

## ✅ Done

| Task | Commits |
|------|---------|
| Refuse unknown YAML keys ("did you mean") | 9453851 |
| Hide BlackThorn behind SkillNodes.hpp | 0deae92 |
| Conditions in the PDDL notation, compiled once | 336cdda, 834ab4d, a8652da |
| Layers checked by the compiler | 03b5030 |
| Compact plugin API | a77203a |
| Three programs, one per layer | 4980744 |
| Documentation rules, who owns which state, Diátaxis move, Doxygen pass | 9809198, 2e8bafd, b282968, 5cb7f38, e0d8022 |
| ScenarioLogic split act / simulate / observe; Mission and Cell renamed | cbd8959, 10b80a3, 6079a34 |
| LineFollower links with two OpenCV modules | 9a384c7 |
| ONNX Runtime from NuGet (HumanoidWalk, MicroDuck build again) | 0465ae1 |
| Errors from files are `Result`; lookups that throw start with `require` | cd8d025, 713a1ed, f6c6026 |
| Sanitizers, random test order, uninitialised values, IK Jacobian | 8fa1aa7, 2a0fb11, c3297fc |
| Validate a cell description; refuse a dt that does not move the clock | 9632c62, 63a88ba |
| Current prop rotation; bodies copied before sensors and truth beliefs | dc42d45, 764e494 |
| Const access; plugin build id; HumanoidWalk pack | 7e5c156, 04d58ac, 3c2b2a3 |
| Skill capabilities: measured vacuum, RobotSession only for the simulator, opaque LinkRef, measured state written by the session, estimated base, belief assertions | f5a69f9, 626aa37, 6da63c5, aaa29b2, 7946c73, c8bdbc5 |
| Web page with the light plugins | b11a30b, b2f1e2d, 41a9204, 4be61fd |
| Arbitration of skills and time (A1-A12): resource ids, transactional preemption, run tickets, `Stopping`, settled speed, robot clock, resume after preemption, leases on writes | 2e7b186, 255ae17, abba320, ea57baa, 1b98305, 0e7b105, 9570530, 7bcbba1, 8de6cb6, 00812ac, 80b6baa |
| Earlier plans of `old/`: plugins and manifests, PDDL planner (Kairos), parallel and temporal plans, DualArm, fidelity `abstract`/`rigid`, BT clock, simulator panels | see `git log` |
