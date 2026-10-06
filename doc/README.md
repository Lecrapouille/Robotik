# Documentation Robotik

Ce dossier décrit l’architecture actuelle : API robot (joints, capteurs, actionneurs, ressources), perception, scheduler de skills, behavior trees BlackThorn, scénarios rejouables et environnements RL.

## Sommaire

| Document | Sujet |
|----------|--------|
| [Ecosysteme.md](Ecosysteme.md) | Diagramme et rôles de Compages, Pinocchio, MuJoCo, BlackThorn |
| [Architecture-Robotik.md](Architecture-Robotik.md) | Dossiers `include/Robotik/`, flux de données, types clés |
| [Scenario-et-Simulation.md](Scenario-et-Simulation.md) | Scénario YAML, seeds, pannes, `Simulation`, assertions |
| [BehaviorTree-et-Skills.md](BehaviorTree-et-Skills.md) | Action BlackThorn, `SkillScheduler`, skills |
| [Demos.md](Demos.md) | Buts, CLI, tutoriel Simulateur / Headless / LineFollower / RL |

## Carte rapide `include/Robotik/`

| Dossier | Rôle |
|---------|------|
| **`Math/`** | Alias Compages (`Pose`, `Quat`, `Vector3`) dans `Geometry.hpp` ; `Seed` / `Random` |
| **`Robot/`** | `Robot`, `RobotSession`, `JointSet` (SoA), `Actuators.hpp` |
| **`Sensors/`** | `Camera`, `Imu`, `RangeScanner`, `ForceTorqueSensor`, lectures ECS |
| **`Perception/`** | `Detector`, `PerceptionPipeline`, `WorldModel`, `localize` |
| **`Runtime/`** | `ResourceManager`, `SkillScheduler`, `FaultInjector`, `Mission`, `Metrics`, `Simulation` |
| **`Skills/`** | `Skill`, skills de mouvement et de pick-and-place, `SkillNodes.hpp` (pont BT) |
| **`Scenario/`** | Parser YAML de mission |
| **`Environment/`** | `Environment`, `EnvironmentPool` (RL) |
| **`Backends/`** | Pinocchio (FK/IK), MuJoCo (physique) |

En-tête unique : `include/Robotik/Robotik.hpp`.

## Applications

- **Simulateur** : `src/Applications/Simulator/` — hôte visuel (menu pick-and-place, line follower, RL random/convergé sur un env).
- **Headless** : `src/Applications/Headless/` — n’importe quel scénario + mission, sans rendu.
- **LineFollower** : `src/Applications/Demos/LineFollower/` — même mission, CLI pour le CI (`--save`).
- **PickAndPlaceRL** : `src/Applications/Demos/PickAndPlaceRL/` — `EnvironmentPool` multi-thread, hors GUI.

Exemple de trajectoire LineFollower (vert : vérité, rouge : estimation) : `Robotik-LineFollower --seed 4 --save map.png`.
