# Documentation Robotik

Ce dossier décrit l’architecture actuelle : API robot (joints, capteurs, actionneurs, ressources), perception, scheduler de skills, behavior trees BlackThorn, scénarios rejouables et environnements RL.

## Sommaire

| Document | Sujet |
|----------|--------|
| [Ecosysteme.md](Ecosysteme.md) | Diagramme et rôles de Compages, Pinocchio, MuJoCo, BlackThorn |
| [Architecture-Robotik.md](Architecture-Robotik.md) | Dossiers `include/Robotik/`, flux de données, types clés |
| [Scenario-et-Simulation.md](Scenario-et-Simulation.md) | Scénario YAML, seeds, pannes, `Simulation`, assertions |
| [BehaviorTree-et-Skills.md](BehaviorTree-et-Skills.md) | Action BlackThorn, `SkillScheduler`, skills |

## Carte rapide `include/Robotik/`

| Dossier | Rôle |
|---------|------|
| **`Math/`** | `Pose`, `Quaternion`, `Seed` / `Random` |
| **`Robot/`** | `Robot`, `RobotSession`, `RobotBackend`, `SceneView`, `JointSet` |
| **`Sensors/`**, **`Actuators/`** | `Camera`, `Image` ; `Motor`, `JointGroup`, `VacuumGripper` |
| **`Perception/`** | `Detector`, `PerceptionPipeline`, `WorldModel`, `localize` |
| **`Runtime/`** | `ResourceManager`, `SkillScheduler`, `FaultInjector`, `Simulation` |
| **`Skills/`** | `Skill`, skills de mouvement et de pick-and-place |
| **`Behavior/`** | Pont BlackThorn (`registerSkills`) |
| **`Scenario/`** | Parser YAML de mission |
| **`Environment/`** | `Environment`, `EnvironmentPool` (RL) |
| **`Backends/`** | Pinocchio (FK/IK), MuJoCo (physique) |

En-tête unique : `include/Robotik/Robotik.hpp`.

## Applications

- **Simulateur** : `src/Applications/Simulator/` — vue 3D, caméra poignet rendue, détection couleur, panneaux skills / ressources / pannes, arrêt d’urgence, rejeu par seed.
- **Headless** : `src/Applications/Headless/` — même mission sans rendu (perception oracle), trace des skills.
- **LineFollower** : `src/Applications/Demos/LineFollower/` — robot différentiel, suivi de ligne (OpenCV), localisation par AprilTags, odométrie biaisée.
- **PickAndPlaceRL** : `src/Applications/Demos/PickAndPlaceRL/` — `EnvironmentPool`, politique experte / aléatoire, rejeu bit à bit.

Exemple de trajectoire LineFollower (vert : vérité, rouge : estimation) : `Robotik-LineFollower --seed 4 --save map.png`.
