# 🎯 `ScenarioLogic`

[⏫ API reference](README.md)

Header: `RobotIK/Task/ScenarioLogic.hpp`. What your plugin adds to a generic simulation. All methods are optional.

| Method | Called | Typical use |
|--------|--------|-------------|
| `setup(RoboticCell&, SceneView* view)` | Once, after the world is built, before the first reset. `view` is null in headless | Add skills, detectors, decorations; maybe your own backend |
| `reset(RoboticCell&, Seed)` | Each episode start, after the robot and objects are reset | Clear state; derive random streams from the seed |
| `act(RobotContext&, Seconds dt)` | After each robot step | Command the robot from what it measures and believes; no ground truth |
| `simulate(RoboticCell&, Seconds dt)` | After `act`, in simulation only | What the world does by itself: a conveyor, ink on a canvas |
| `observe(RoboticCell const&, Seconds dt)` | After `simulate` | Read the truth and count for the metrics; a const cell commands nothing |
| `measure(RoboticCell const&, Metrics&) const` | When assertions or reports need values | `metrics.set("name", value)` |
| `Status status(RobotContext const&) const` | After each step, **only without behavior tree** | `SUCCESS` / `FAILURE` end the run; `RUNNING` / `IDLE` continue |
