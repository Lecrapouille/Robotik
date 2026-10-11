# Documentation Robotik

Ce dossier décrit l’architecture actuelle : API robot (joints, capteurs, actionneurs, ressources), perception, scheduler de skills, behavior trees BlackThorn, scénarios rejouables et environnements RL.

## Sommaire

| Document | Sujet |
|----------|--------|
| [Ecosysteme.md](Ecosysteme.md) | Diagramme et rôles de Compages, Pinocchio, MuJoCo, BlackThorn |
| [Architecture-Robotik.md](Architecture-Robotik.md) | Dossiers `include/Robotik/`, flux de données, types clés |
| [Scenario-et-Simulation.md](Scenario-et-Simulation.md) | Monde, scénario, expérience, assertions, trajectoires, vérité terrain |
| [BehaviorTree-et-Skills.md](BehaviorTree-et-Skills.md) | Action BlackThorn, `SkillScheduler`, skills |
| [Demos.md](Demos.md) | Buts, CLI, tutoriel Simulateur / Headless / LineFollower / RL |
| [Apprentissage.md](Apprentissage.md) | Environnements RL : espaces, observation / action / récompense, fins d’épisode, pool, journal |
| [Plugins.md](Plugins.md) | Pourquoi un plugin, manifeste, packs d’assets, cycle de vie, tutoriel |
| [Tools.md](Tools.md) | Robot flange, tool URDF, TCP (English) |

## Carte rapide `include/Robotik/`

| Dossier | Rôle |
|---------|------|
| **`Assets/`** | Packs d’assets versionnés, `AssetResolver` (`robotik.tools.vacuum` → fichier) |
| **`Math/`** | Alias Compages (`Pose`, `Quat`, `Vector3`) dans `Geometry.hpp` ; `Seed` / `Random` |
| **`Robot/`** | `Robot`, `RobotSession`, `JointSet` (SoA), `Actuators.hpp` |
| **`Sensors/`** | `Camera`, `Imu`, `RangeScanner`, `ForceTorqueSensor`, en-tête `Measurement`, `NoiseModel`, latence |
| **`Perception/`** | `Detector`, `PerceptionPipeline`, `WorldModel`, `localize` |
| **`Runtime/`** | `ResourceManager`, `SkillScheduler`, `FaultInjector`, `Mission`, `Metrics`, `Simulation`, `RunReport` |
| **`Skills/`** | `Skill`, skills de mouvement, `SkillNodes.hpp` (pont BT) |
| **`Scenario/`** | Monde, scénario, expérience, moteur d’assertions, trajectoires |
| **`Environment/`** | `Environment`, `Space`, `SimulationEnvironment`, `EnvironmentPool`, `EpisodeLog` (RL) |
| **`Backends/`** | Pinocchio (FK/IK), MuJoCo (physique) |
| **`Plugin/`** | Contrat C, aide C++, catalogue et session chargés par les hôtes |

En-tête unique : `include/Robotik/Robotik.hpp`.

## Hôtes et démos

- **Simulateur** : `simulator/` — fenêtre, menus, panneaux. Le menu **Demos** charge les paquets de `build/plugins/`.
- **Headless** : `headless/` — le même scénario, ou une expérience entière, sans rendu (`Robotik-Headless`).
- **Démos** : `plugins/<Nom>/` — un plugin C++ (`src/`), un `plugin.yaml`, ses mondes, scénarios et expériences, éventuellement son pack d’assets (`data/`). Line follower, le pool RL et la mouche gardent un exécutable dans `plugins/<Nom>/app` pour le CI et les environnements parallèles.
- **Assets** : `data/packs/<id>/` — robots et outils partagés, chacun avec son `pack.yaml`.

Exemple de trajectoire LineFollower (vert : vérité, rouge : estimation) : `Robotik-LineFollower --seed 4 --save map.png`.
