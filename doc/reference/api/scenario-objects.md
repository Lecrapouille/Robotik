# 🎬 Scenario objects

[⏫ API reference](README.md)

Headers in `RobotIK/Package/Scenario/`. Usually built by the host; useful in tests.

| Class | Role |
|-------|------|
| `loadScenario(path, assets)` | Parse a scenario and its world; returns a `Result<Scenario>` |
| `ScenarioSource`, `ConfigNode` | The YAML as an editable tree: `loadScenarioSource`, `overrideValue(source, "world.entities.x.y", ...)`, then `readScenario` |
| `Experiment` | Cases, seeds, evaluation: `loadExperiment`, `scenarioOf(experiment, case, assets)` |
| `WorldDescription`, `EntitySpec` | The parsed world (Task layer data: `findEntity`, `firstRobot`, `robotsOf`) |
| `Trajectory` | A cartesian or joint path: `loadTrajectory` |
| `AssertionEngine`, `Check` | Verdicts: `verdict`, `text`, `detail`, and `ok(check)` |
