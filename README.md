# Robotik

**Bibliothèque C++20 pour simuler, visualiser et piloter des manipulations robotiques** — avec la même logique de mission en mode graphique, headless ou (à terme) sur le robot réel.

Démo : [vidéo YouTube](https://www.youtube.com/watch?v=BgFjewCz328)

> Projet actif en évolution : l’API et les scénarios peuvent changer ; les tests et le simulateur servent de référence.

---

## En quoi Robotik se distingue

Beaucoup d’outils excellent sur **un** axe (physique, motion planning, ROS, ou BT industriel). Robotik vise un **couplage explicite** entre mission déclarative, comportements réutilisables et **deux moteurs complémentaires** sur le même modèle URDF.

| Idée | Robotik | Approche classique ailleurs |
|------|---------|------------------------------|
| **Analytique + contact** | **Pinocchio** (FK/IK, skills TCP) et **MuJoCo** (dynamique, préhension) synchronisés sur un **ECS** commun | Souvent un seul moteur, ou deux mondes désynchronisés |
| **Mission** | Fichier **scénario YAML** (robot, objets, BT, assertions) + arbre **BlackThorn** | Launch files ROS lourds, ou code C++ monolithique |
| **Comportements** | **Skills** (`Home`, `MoveTCP`, pick-and-place, gripper) branchées au BT via `registerSkill` | Nodes ROS custom, ou logique noyée dans la simu |
| **Scène** | **Compages** : monde, rendu, caméra embarquée, détection couleur simple | RViz / Isaac / GUI ad hoc |
| **Backend swap** | Les skills lisent `JointState` / écrivent `JointCommand` — pas MuJoCo en dur | Skills souvent liées au simulateur ou au driver |

En résumé : **déclarer la mission**, **composer en behavior tree**, **exécuter via des skills testables**, **simuler le contact avec MuJoCo** tout en gardant **l’IK Pinocchio** pour le contrôle outil — le tout accroché à une hiérarchie 3D Compages.

Schéma des dépendances : [doc/Ecosysteme.md](doc/Ecosysteme.md).  
Architecture du code : [doc/Architecture-Robotik.md](doc/Architecture-Robotik.md).

---

## Stack technique

- **[Compages](https://github.com/Lecrapouille/Compages)** — monde, transforms, rendu OpenGL.
- **[Pinocchio](https://github.com/stack-of-tasks/pinocchio)** — cinématique et IK.
- **[MuJoCo](https://github.com/google-deepmind/mujoco)** — dynamique et contacts.
- **[BlackThorn](https://github.com/Lecrapouille/BlackThorn)** — behavior trees et YAML.

Robotik (`librobotik-core`) fournit : chargement URDF, composants ECS, pipeline de systems, runtime, skills et pont BT.

---

## Compilation

### Prérequis

Debian / Ubuntu :

```bash
sudo apt-get install build-essential cmake git \
    libeigen3-dev libgl1-mesa-dev libglew-dev libglfw3-dev \
    libsfml-dev swi-prolog-dev swi-prolog

# Optionnel, pour `make tests` :
sudo apt-get install libgtest-dev libgmock-dev
```

Fedora :

```bash
sudo dnf install gcc-c++ make cmake git curl \
    eigen3-devel libglvnd-devel glew-devel glfw-devel \
    SFML-devel swi-prolog-core pkgconf-pkg-config

# Optionnel, pour `make tests` :
sudo dnf install gtest-devel gmock-devel
```

Compages doit être disponible (`pkg-config --exists Compages`) ou cloné via [external/manifest](external/manifest). Pinocchio et MuJoCo sont récupérés dans `external/forge` par `make compile-external-libs` (conda-forge, sans root). Le projet est **C++20**.

`cmake` sert surtout à construire rapidyaml (backend YAML de BlackThorn). SFML (module réseau) alimente la visualisation distante Oakular pour les BT.

> **Astuce :** le build passe par `pkg-config`. Si un paquet manque, toute la chaîne échoue souvent avec des erreurs trompeuses. Vérifier :  
> `pkg-config --exists eigen3 gl glew glfw3 sfml-network swipl`

### Build

```bash
git clone https://github.com/Lecrapouille/Robotik --recurse-submodules
cd Robotik
make download-external-libs
make compile-external-libs
make -j8
make applications -j8

# Optionnel :
make tests -j8
sudo make install
```

Artefacts typiques dans `build/` : `librobotik-core.so`, **Robotik-Simulator**, **Robotik-Headless**.

---

## Applications

**Headless** — exécute un scénario complet (BT + physique + assertions), sans fenêtre :

```bash
./build/Robotik-Headless data/scenarios/pick_and_place.yml
```

**Simulateur** — même pipeline + vue 3D, caméra robot, panneaux BT/skills :

```bash
./build/Robotik-Simulator data/scenarios/pick_and_place.yml
```

Clic droit sur la vue « World » pour orbiter la caméra.

---

## Exemple minimal (API)

Les skills ne parlent pas à MuJoCo directement : elles passent par le contexte et l’ECS ; le runtime enchaîne Pinocchio, MuJoCo et les systems.

```cpp
#include "Robotik/Robotik.hpp"

compages::world::World world;
robotik::RobotRuntime runtime(world, std::filesystem::path("robot.urdf"));
robotik::MoveJointSkill skill("revolute_joint", Radians(0.5));

robotik::RobotContext context = runtime.context();
if (skill.tick(context, Seconds(0.002)) != robotik::Status::FAILURE)
{
    runtime.step(Seconds(0.002));
}
```

Skills fournies : `Home`, `MoveJoint`, `MoveJoints`, `MoveTCP`, gripper, pick-and-place (`Detect`, `Approach`, `Grasp`, …). Enregistrement BT : `registerSkill` dans [Behavior/SkillNodes.hpp](include/Robotik/Behavior/SkillNodes.hpp).

---

## Documentation

| Document | Contenu |
|----------|---------|
| [doc/README.md](doc/README.md) | Index de la doc |
| [doc/Ecosysteme.md](doc/Ecosysteme.md) | Liens Compages / Pinocchio / MuJoCo / BlackThorn |
| [doc/Architecture-Robotik.md](doc/Architecture-Robotik.md) | Dossiers `include/Robotik/`, flux de données |
| [doc/Scenario-et-Simulation.md](doc/Scenario-et-Simulation.md) | YAML de mission |
| [doc/BehaviorTree-et-Skills.md](doc/BehaviorTree-et-Skills.md) | Action BT vs skill Robotik |

---

## Références

- [Cours robotique — Jacques Gangloff](https://www.youtube.com/playlist?list=PLMXdciyMZwAAUlCQ_9mVs_CqQ9YaRTptX)
