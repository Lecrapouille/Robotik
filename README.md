# Robotik

**Bibliothèque C++20 pour simuler, visualiser et piloter des robots** — avec la même logique de mission en mode graphique, headless, en apprentissage par renforcement ou (à terme) sur le robot réel.

Démo : [vidéo YouTube](https://www.youtube.com/watch?v=BgFjewCz328)

> Projet actif en évolution : l’API et les scénarios peuvent changer ; les tests et les démos servent de référence.

---

## En quoi Robotik se distingue

| Idée | Robotik |
|------|---------|
| **API robot compacte** | `Robot` = joints (`JointSet`, tableaux contigus en unités SI), capteurs, actionneurs, ressources, IK. Les skills ne connaissent ni MuJoCo ni le matériel : un `RobotBackend` est branché derrière. |
| **Ressources et pannes** | Chaque capteur et actionneur est une ressource. Une panne la rend indisponible ; le `SkillScheduler` arbitre (priorités, préemption, annulation) et explique pourquoi une skill attend ou échoue. |
| **Perception** | `Camera` → `PerceptionPipeline` (étapes `Detector` interchangeables) → `WorldModel` (croyances) ; localisation par amers (AprilTags). |
| **Mission** | Scénario YAML (robot, capteurs, actionneurs, objets randomisés, pannes, BT, assertions) + arbre **BlackThorn**. |
| **Rejouable** | Une seed maître dérive les graines du monde, des pannes et du bruit de chaque capteur : même seed, même run. |
| **RL** | `EnvironmentPool` : N environnements en parallèle, buffers plats, résultats identiques quel que soit le nombre de threads. |
| **Bibliothèque sans rendu** | Ni OpenCV ni OpenGL dans `librobotik-core` : le rendu passe par `SceneView`, les images par `FrameSource`. |

Schéma des dépendances : [doc/Ecosysteme.md](doc/Ecosysteme.md). Architecture : [doc/Architecture-Robotik.md](doc/Architecture-Robotik.md).

---

## Stack technique

- **[Compages](https://github.com/Lecrapouille/Compages)** — monde, transforms, rendu OpenGL.
- **[Pinocchio](https://github.com/stack-of-tasks/pinocchio)** — cinématique et IK.
- **[MuJoCo](https://github.com/google-deepmind/mujoco)** — dynamique et contacts.
- **[BlackThorn](https://github.com/Lecrapouille/BlackThorn)** — behavior trees et YAML.
- Démos uniquement : **[apriltag](https://github.com/AprilRobotics/apriltag)** (BSD-2) et **OpenCV 4**.

---

## Compilation

### Prérequis

Debian / Ubuntu :

```bash
sudo apt-get install build-essential cmake git \
    libeigen3-dev libgl1-mesa-dev libglew-dev libglfw3-dev \
    libsfml-dev swi-prolog-dev swi-prolog

# Optionnel : démo LineFollower
sudo apt-get install libopencv-dev
# Optionnel, pour `make tests` :
sudo apt-get install libgtest-dev libgmock-dev
```

Fedora :

```bash
sudo dnf install gcc-c++ make cmake git curl \
    eigen3-devel libglvnd-devel glew-devel glfw-devel \
    SFML-devel swi-prolog-core pkgconf-pkg-config

# Optionnel : démo LineFollower
sudo dnf install opencv-devel
# Optionnel, pour `make tests` :
sudo dnf install gtest-devel gmock-devel
```

Compages doit être disponible (`pkg-config --exists Compages`) ou cloné via [external/manifest](external/manifest). Pinocchio et MuJoCo sont récupérés dans `external/forge` par `make compile-external-libs` (conda-forge, sans root), qui compile aussi apriltag en bibliothèque statique.

> **Astuce :** le build passe par `pkg-config`. Si un paquet manque, la chaîne échoue souvent avec des erreurs trompeuses. Vérifier :
> `pkg-config --exists eigen3 gl glew glfw3 sfml-network swipl opencv4`

### Build

```bash
git clone https://github.com/Lecrapouille/Robotik --recurse-submodules
cd Robotik
make download-external-libs
make compile-external-libs
make -j8            # bibliothèque, applications et démos

# Optionnel :
make tests -j8
sudo make install
```

Artefacts dans `build/` : `librobotik-core.so`, `Robotik-Simulator`, `Robotik-Headless`, `Robotik-LineFollower`, `Robotik-PickAndPlaceRL`.

---

## Applications et démos

**Headless** — scénario complet (BT + physique + assertions) sans fenêtre, avec la trace des skills :

```bash
./build/Robotik-Headless data/scenarios/pick_and_place.yml --seed 7
./build/Robotik-Headless data/scenarios/pick_and_place_faults.yml
```

**Simulateur** — hôte visuel des missions (menu pick-and-place, line follower, RL random/convergé sur un env) ; panneaux communs (monde, caméra, skills, ressources, assertions) ; arrêt d’urgence ; rejeu ou nouvelle seed :

```bash
./build/Robotik-Simulator data/scenarios/pick_and_place.yml
```

**LineFollower** — robot différentiel qui se localise sur des AprilTags au sol puis suit une ligne (OpenCV) avec une odométrie volontairement biaisée :

```bash
./build/Robotik-LineFollower --seed 4 --laps 1 --save map.png   # --view pour voir la caméra
```

**PickAndPlaceRL** — pick-and-place en environnements parallèles, politique convergée ou aléatoire (`--train` pour converger), vérification du rejeu :

```bash
./build/Robotik-PickAndPlaceRL --policy converged --envs 16 --threads 8 --episodes 32
./build/Robotik-PickAndPlaceRL --policy random --train --envs 8 --episodes 16
```

---

## Exemple minimal (API)

```cpp
#include "Robotik/Robotik.hpp"
#include "Compages/World/World.hpp"

compages::world::World world;
robotik::RobotSession robot(world, "data/robot_6axis.urdf");
robot.connect(std::make_unique<robotik::MujocoBackend>("data/robot_6axis.urdf"));
robot.actuators().add<robotik::JointGroup>("arm");
robot.hold({ { "joint2", 0.3 } });

robotik::WorldModel beliefs;
robotik::SkillScheduler skills(robot.resources());
auto const move = skills.add<robotik::MoveJointSkill>(
    { .name = "MoveJoint1", .resources = { robot.resources().require("arm") } },
    "joint1", 0.5);
skills.request(move);

Seconds const dt(0.01);
robotik::RobotContext context{ robot, beliefs };
while (skills.state(move) != robotik::SkillState::Succeeded &&
       skills.state(move) != robotik::SkillState::Failed)
{
    context.time = robot.time();
    context.dt = dt;
    skills.update(context);   // admission, préemption, tick
    robot.step(dt);           // MuJoCo, Pinocchio, capteurs
}
```

Skills fournies : `Home`, `MoveJoint`, `MoveJoints`, `MoveTCP`, `Stop`, et pour le pick-and-place `Detect`, `Approach`, `Reach`, `Grasp`, `Release`. Exposition au BT : `registerSkills` dans [Skills/SkillNodes.hpp](include/Robotik/Skills/SkillNodes.hpp). Une mission (`Mission`) ajoute les skills de tâche ; le Simulateur les affiche via un menu (pick-and-place, line follower, RL).

---

## Documentation

| Document | Contenu |
|----------|---------|
| [doc/README.md](doc/README.md) | Index de la doc |
| [doc/Ecosysteme.md](doc/Ecosysteme.md) | Liens Compages / Pinocchio / MuJoCo / BlackThorn |
| [doc/Architecture-Robotik.md](doc/Architecture-Robotik.md) | Dossiers `include/Robotik/`, choix cache friendly |
| [doc/Scenario-et-Simulation.md](doc/Scenario-et-Simulation.md) | Scénario YAML, seeds, pannes |
| [doc/BehaviorTree-et-Skills.md](doc/BehaviorTree-et-Skills.md) | BT, scheduler, skills |
| [doc/Demos.md](doc/Demos.md) | Démos : buts, CLI, tutoriel |

---

## Références

- [Cours robotique — Jacques Gangloff](https://www.youtube.com/playlist?list=PLMXdciyMZwAAUlCQ_9mVs_CqQ9YaRTptX)
