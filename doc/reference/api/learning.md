# 🎲 Learning

[⏫ API reference](README.md)

Headers in `RobotIK/Task/Learning/`. See [Build reinforcement learning environments](../../guides/reinforcement-learning.md).

| Class | Role |
|-------|------|
| `Environment` | `reset(seed, observation)`, `step(action, observation)`, `observationSpace()`, `actionSpace()` |
| `Space` | Named, bounded dimensions (`add`, `add3`), with units; `privileged()` |
| `SimulationEnvironment` | A cell description (a scenario is one) + `ObservationBuilder` + `ActionAdapter` + `RewardEvaluator` |
| `EnvironmentPool` | N environments on threads, derived seeds |
| `EpisodeLog` | JSON log of the episodes |
| `Agent` (`RobotIK/Task/Agents/Agent.hpp`) | `Action update(Observation const&, Seconds)`: any brain |
