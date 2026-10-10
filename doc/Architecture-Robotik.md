# Architecture de Robotik

Ce document explique **comment Robotik est construit et pourquoi les responsabilités sont séparées ainsi**.

L'objectif n'est pas de présenter chaque classe. Il est de permettre au lecteur de comprendre le chemin complet :

```text
Scénario
   ↓
Behavior Tree
   ↓
SkillScheduler
   ↓
Skills
   ↓
Robot / actionneurs
   ↓
Backend
   ↓
simulation ou robot réel
   ↓
capteurs
   ↓
perception
   ↓
WorldModel
```

---

## 1. Le principe fondamental : Robotik orchestre

Robotik n'est pas un moteur de simulation monolithique.

Il assemble plusieurs briques spécialisées :

```text
                       Robotik
                          │
        ┌─────────────────┼──────────────────┐
        │                 │                  │
     Robot API         Autonomie          Scénarios
        │                 │                  │
        │              Skills / BT           │
        │                 │                  │
        ├──────────── Backends ──────────────┤
        │                 │                  │
     Pinocchio          MuJoCo          Hardware
        │                 │                  │
       IK              Physics          SO-101...
```

Les bibliothèques externes restent responsables de leur domaine.

Robotik fournit la couche qui permet de les combiner.

---

# 2. Le robot : modèle, état et backend

Un robot doit être décrit indépendamment de la manière dont il est exécuté.

On peut distinguer trois notions :

### Le modèle

Le modèle décrit la structure :

```text
Robot
 ├── Link
 ├── Joint
 ├── sensor
 └── actuator
```

Le modèle peut provenir d'un URDF.

### L'état

L'état décrit ce qui se passe maintenant :

```text
JointState
 ├── position
 ├── velocity
 └── effort
```

Les grandeurs publiques doivent rester exprimées dans des unités physiques explicites.

### Le backend

Le backend sait comment obtenir ou modifier cet état.

```text
Robot
 │
 └── RobotBackend
       ├── MujocoBackend
       └── HardwareBackend
```

Le reste de Robotik ne doit pas avoir à savoir si le robot est simulé.

---

# 3. Simulation contre robot réel

C'est une contrainte architecturale importante.

Une skill doit pouvoir écrire :

```cpp
robot.joint("joint1").command(...);
```

sans faire :

```cpp
if (simulation)
    mujoco_command(...);
else
    serial_command(...);
```

Le backend s'occupe de cette traduction.

```text
              Robotik
                 │
            RobotSession
                 │
          RobotBackend
          /          \
         /            \
      MuJoCo        Hardware
        │               │
     qpos/qvel      servo / bus
```

Cela permet d'envisager le même code pour :

```text
             PickAndPlace
                  │
        ┌─────────┴─────────┐
        ▼                   ▼
   robot simulé          SO-101 réel
```

---

# 4. `RobotSession`

`RobotSession` est le point de rencontre entre le robot, son backend et le monde.

Un usage minimal ressemble à :

```cpp
compages::world::World world;

robotik::RobotSession robot(
    world,
    "data/robot_6axis.urdf"
);

auto physics = std::make_unique<robotik::MujocoBackend>();
physics->load("data/robot_6axis.urdf");
robot.connect(std::move(physics));
```

Puis la boucle de simulation peut avancer :

```cpp
robot.step(dt);
```

Cette boucle met à jour notamment :

- le backend ;
- les états articulaires ;
- la cinématique ;
- les transforms ;
- les capteurs disponibles.

---

# 5. Les articulations : `JointSet`

Robotik privilégie une représentation compacte des états articulaires.

Conceptuellement :

```text
JointSet
 ├── positions[]
 ├── velocities[]
 ├── efforts[]
 └── commands[]
```

Cette organisation permet de manipuler efficacement de nombreux joints sans multiplier les petits objets alloués.

Les unités physiques font partie du contrat de l'API.

Par exemple :

```text
joint revolute
    position  → angle
    velocity  → angle / time
    effort    → torque

joint prismatic
    position  → length
    velocity  → length / time
    effort    → force
```

Les conversions propres au simulateur ou au matériel restent dans les backends.

---

# 6. Actionneurs et ressources

Un actionneur n'est pas seulement une fonction qui envoie une commande.

Dans Robotik, il peut également devenir une **ressource**.

Exemple :

```text
Robot
 ├── arm
 ├── gripper
 └── wrist_camera
```

Une skill `Grasp` peut réserver :

```text
gripper     exclusif
arm         exclusif
wrist_camera partagé
```

Une autre skill qui demande le même gripper doit attendre, échouer ou provoquer une préemption selon ses règles.

Cette notion de ressource est essentielle dès que plusieurs skills peuvent être exécutées ou demandées simultanément.

---

# 7. Les capteurs

Les capteurs appartiennent au robot, mais leur traitement ne doit pas être confondu avec le capteur lui-même.

```text
Camera
   │
   ▼
CameraReading
   │
   ▼
PerceptionPipeline
```

Le cœur de Robotik ne dépend pas d'OpenCV.

Une caméra peut fournir une image générique :

```cpp
robotik::Image
```

puis une application peut utiliser :

```text
OpenCV
AprilTag
réseau neuronal
autre détecteur
```

---

# 8. Perception et `WorldModel`

La perception transforme des mesures en informations utilisables.

```text
Camera
  │
  ▼
Detector
  │
  ▼
Detection
  │
  ▼
WorldModel
```

Exemple :

```text
image
  ↓
AprilTag detector
  ↓
tag #12
  ↓
pose du tag
  ↓
WorldModel
```

Ou :

```text
image
  ↓
ObjectDetector
  ↓
red_cube
  ↓
position + confiance
  ↓
WorldModel
```

Le `WorldModel` représente alors ce que le robot **croit savoir** du monde.

C'est volontairement différent de la vérité interne du simulateur.

---

# 9. Vérité et croyance

C'est une distinction importante pour tester la perception.

Le simulateur sait :

```text
red_cube = (0.40, 0.20, 0.02)
```

Mais le robot peut croire :

```text
red_cube = (0.39, 0.21, 0.03)
confidence = 0.87
```

Un bon scénario peut donc tester :

```text
Vérité
   │
   ▼
Simulation
   │
   ├── caméra
   │
   ▼
Perception
   │
   ▼
Croyance
   │
   ▼
Skill
```

Cela évite de tricher en donnant directement aux skills les positions parfaites du simulateur.

---

# 10. `SkillScheduler`

Le scheduler est responsable de **quand une skill peut réellement s'exécuter**.

Il gère notamment :

- les ressources ;
- les priorités ;
- les préconditions ;
- les annulations ;
- la préemption ;
- les pannes.

Exemple :

```text
Skill A : MoveArm
    priorité 10
    ressource = arm

Skill B : Stop
    priorité 100
    ressource = arm
```

Si `Stop` est demandé :

```text
MoveArm
   │
   ├── priorité 10
   │
   ▼
préempté / annulé

Stop
   │
   ├── priorité 100
   │
   ▼
exécuté
```

Le scheduler ne sait pas comment déplacer le bras.

Il sait seulement **si et quand la skill est autorisée à tourner**.

---

# 11. Une étape de simulation

Conceptuellement, un pas de simulation suit cette logique :

```text
Simulation::step(dt)
       │
       ├── FaultInjector
       │
       ├── Behavior Tree
       │       └── demande / annule des skills
       │
       ├── SkillScheduler
       │       ├── ressources
       │       ├── priorités
       │       └── tick
       │
       ├── RobotSession
       │       ├── actionneurs
       │       ├── backend
       │       ├── cinématique
       │       └── transforms
       │
       ├── capteurs
       │
       ├── perception
       │
       └── WorldModel
```

L'ordre exact dépend des composants, mais l'idée est toujours la même :

> **les décisions produisent des commandes ; le robot évolue ; les capteurs observent ; la perception met à jour les croyances.**

---

# 12. Compages

Compages est le monde 3D utilisé par Robotik.

Robotik ne fait pas lui-même le rendu.

```text
Robotik
   │
   └── SceneView
           │
           ▼
        Compages
```

Compages s'occupe notamment de :

- entités ;
- transforms ;
- monde 3D ;
- affichage.

Cela permet au cœur de Robotik de fonctionner sans fenêtre.

---

# 13. Pourquoi le rendu est séparé ?

Le mode headless est indispensable pour :

- CI ;
- tests ;
- RL ;
- benchmarks ;
- serveurs ;
- génération de milliers d'épisodes.

On veut pouvoir faire :

```bash
Robotik-Headless scenario.yml
```

sans créer une fenêtre OpenGL.

Le simulateur graphique devient donc un **client de Robotik**, pas le cœur de Robotik.

---

# 14. Pinocchio

Pinocchio fournit les calculs de cinématique et les algorithmes associés.

Robotik l'utilise derrière une interface dédiée :

```text
Robot
   │
   ▼
PinocchioBackend
   │
   ├── FK
   ├── Jacobian
   └── IK
```

Une skill cartésienne peut donc demander :

```text
MoveTCP(target)
       │
       ▼
Pinocchio
       │
       ▼
joint targets
```

Robotik n'a pas vocation à devenir une seconde bibliothèque Pinocchio.

---

# 15. MuJoCo

MuJoCo est utilisé comme backend de simulation physique.

```text
Skill
  │
  ▼
Joint command
  │
  ▼
MuJoCo
  │
  ├── dynamique
  ├── contacts
  └── intégration
  │
  ▼
JointState
```

Le backend traduit ensuite cet état vers les structures Robotik.

---

# 16. Architecture des dossiers

L'organisation actuelle suit les responsabilités :

```text
include/Robotik/
├── Math/
├── Robot/
├── Sensors/
├── Perception/
├── Runtime/
├── Skills/
├── Scenario/
├── Environment/
├── Backends/
├── Systems/
└── ECS/
```

### `Robot/`

API du robot :

```text
Robot
RobotSession
JointSet
Motor
JointGroup
VacuumGripper
```

### `Sensors/`

Capteurs et mesures :

```text
Camera
Imu
RangeScanner
ForceTorqueSensor
```

### `Perception/`

Transformation des mesures en informations :

```text
Detector
PerceptionPipeline
Detection
WorldModel
Localization
```

### `Runtime/`

Exécution :

```text
Simulation
SkillScheduler
ResourceManager
FaultInjector
Mission
Metrics
```

### `Skills/`

Capacités du robot :

```text
Skill
MotionSkills
SkillNodes
```

Les skills de pick-and-place vivent dans `demos/PickAndPlaceBT`.

### `Scenario/`

Description déclarative d'une expérience :

```text
robot
world
sensors
actuators
faults
behavior tree
assertions
seed
```

### `Environment/`

Abstraction destinée aux expériences RL :

```text
Environment
EnvironmentPool
```

### `Backends/`

Adaptateurs vers des systèmes externes :

```text
RobotBackend
MujocoBackend
PinocchioBackend
SceneView
```

---

# 17. Pourquoi cette séparation est importante

Elle permet par exemple de remplacer :

```text
MuJoCo
```

par :

```text
SO101Backend
```

sans réécrire :

```text
Scenario
Behavior Tree
Skills
Scheduler
WorldModel
Assertions
```

Ou de remplacer :

```text
OpenCV Detector
```

par :

```text
Neural Detector
```

sans modifier le scénario.

C'est cette interchangeabilité qui constitue une grande partie de la valeur de Robotik.
