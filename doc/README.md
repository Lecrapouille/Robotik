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
| [Plugins.md](Plugins.md) | Pourquoi un plugin, cycle de vie, tutoriel pour en ajouter un |

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
| **`Plugin/`** | Contrat C, aide C++, catalogue et session chargés par les hôtes |

En-tête unique : `include/Robotik/Robotik.hpp`.

## Hôtes et démos

- **Simulateur** : `simulator/` — fenêtre, menus, panneaux. Le menu **Demos** charge les paquets de `build/plugins/`.
- **Headless** : `headless/` — le même scénario, sans rendu (`Robotik-Headless`).
- **Démos** : `demos/<Nom>/` — un plugin C++, un `plugin.yaml`, plusieurs scénarios. Line follower, le pool RL et la mouche gardent un exécutable dans `demos/<Nom>/app` pour le CI et les environnements parallèles.

Exemple de trajectoire LineFollower (vert : vérité, rouge : estimation) : `Robotik-LineFollower --seed 4 --save map.png`.
