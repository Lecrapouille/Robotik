# Documentation Robotik

Ce dossier décrit l’architecture actuelle : ECS sur Compages, runtime, backends Pinocchio/MuJoCo, behavior trees BlackThorn et skills.

## Sommaire

| Document | Sujet |
|----------|--------|
| [Ecosysteme.md](Ecosysteme.md) | **Diagramme** et rôles de Compages, Pinocchio, MuJoCo, BlackThorn |
| [Architecture-Robotik.md](Architecture-Robotik.md) | Dossiers `include/Robotik/`, flux de données, types clés |
| [Scenario-et-Simulation.md](Scenario-et-Simulation.md) | Fichier scénario YAML, spawn, `Simulation`, assertions |
| [BehaviorTree-et-Skills.md](BehaviorTree-et-Skills.md) | Action BlackThorn vs skill Robotik, `registerSkill` |

## Carte rapide `include/Robotik/`

| Dossier | Rôle |
|---------|------|
| **`Backends/`** | Adaptateurs Pinocchio (FK/IK) et MuJoCo (physique) |
| **`Model/`** | `RobotLoader` — URDF → Compages + ECS + bindings |
| **`ECS/`** | Composants joints, robot, objets, perception, backends |
| **`Systems/`** | PD, sync MuJoCo/Pinocchio, projection, grasp |
| **`Skills/`** | `Skill::tick` — comportements impératifs |
| **`Behavior/`** | Pont BlackThorn (`registerSkill`, `SkillTrace`) |
| **`Runtime/`** | `RobotRuntime`, `Simulation`, `RobotContext` |
| **`Scenario/`** | Parser YAML de mission |

## Applications

- **Simulateur** : `src/Applications/Simulator/` — scénario, vue 3D, frise skills.
- **Headless** : `src/Applications/Headless/` — même pipeline sans rendu.
- **API condensée** : `include/Robotik/Robotik.hpp`.
