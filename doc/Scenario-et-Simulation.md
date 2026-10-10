# Scénarios et simulation

Le scénario est le point de départ recommandé pour comprendre Robotik.

Un scénario répond à une question simple :

> **« Que doit faire mon robot, dans quel monde, avec quelles contraintes, et comment puis-je savoir si l'expérience a réussi ? »**

Il ne décrit pas toute l'implémentation du robot.

---

# 1. Un scénario est plus qu'une démo

Le même scénario peut servir à :

```text
             Scenario
                 │
       ┌─────────┼──────────┐
       ▼         ▼          ▼
     Demo       Test       RL
       │         │          │
       └─────────┼──────────┘
                 ▼
          expérience robot
```

Par exemple :

```yaml
assert:
  - object("red_cube").inside("box")
  - collisions == 0
```

est une vérification indépendante de la façon dont le robot a réussi.

La solution peut venir :

- d'un Behavior Tree écrit à la main ;
- d'un plan PDDL ;
- d'un plan GOAP ;
- d'une politique RL ;
- plus tard d'une politique VLA.

Le scénario ne change pas.

---

# 2. Structure générale

Un scénario Robotik contient typiquement :

```text
Scenario
 ├── seed
 ├── robot
 │    ├── model
 │    ├── home
 │    ├── sensors
 │    └── actuators
 │
 ├── world
 │    └── objects
 │
 ├── faults
 │
 ├── execute
 │    ├── task
 │    └── behavior_tree
 │
 └── assert
```

Exemple minimal :

```yaml
scenario: pick_and_place
seed: 42

robot:
  model: ../robot_6axis.urdf

world:
  objects:
    red_cube:
      type: cube
      position: [0.40, 0.20, 0.02]

execute:
  task: Pick the red cube and put it in the box.
  behavior_tree: pick_and_place.bt.yml

assert:
  - robot.success
  - object("red_cube").inside("box")
```

---

# 3. Pourquoi YAML ?

YAML permet de modifier l'expérience sans recompiler le moteur.

On peut changer :

```yaml
seed: 42
```

ou :

```yaml
red_cube:
  randomize:
    x: [-0.02, 0.02]
    y: [-0.02, 0.02]
```

ou :

```yaml
faults:
  - { at: 2.0, resource: wrist_camera, action: disable }
```

sans modifier le code de la skill.

Le code C++ reste responsable de la logique et des algorithmes.

---

# 4. Le monde

Un scénario peut créer des objets :

```yaml
world:
  objects:
    red_cube:
      type: cube
      size: [0.04, 0.04, 0.04]
      position: [0.40, 0.20, 0.02]

    box:
      type: box
      size: [0.20, 0.20, 0.10]
      position: [0.60, 0.20, 0.05]
```

L'objectif peut ensuite être exprimé sémantiquement :

```text
object("red_cube").inside("box")
```

Cette expression est beaucoup plus robuste pour un test que :

```text
joint1 == 0.31
joint2 == -0.72
...
```

Pourquoi ?

Parce que le résultat qui nous intéresse est :

> « Le cube est dans la boîte. »

et non :

> « Le robot possède exactement ces angles articulaires. »

---

# 5. Vérité du simulateur et perception

Un scénario peut distinguer :

```text
Ground Truth
     │
     ▼
Simulation
     │
     ▼
Sensors
     │
     ▼
Perception
     │
     ▼
WorldModel
```

Le simulateur connaît la position exacte.

Le robot ne la connaît pas forcément.

Exemple :

```text
Ground truth:
red_cube = (0.40, 0.20, 0.02)

Camera:
    image

Detector:
    red_cube
    confidence = 0.91

WorldModel:
    red_cube ≈ (0.41, 0.19, 0.03)
```

Cela permet de tester réellement la chaîne perception → décision.

---

# 6. Capteurs

Un scénario peut déclarer une caméra :

```yaml
robot:
  sensors:
    wrist_camera:
      type: camera
      parent: link6
      position: [0.05, 0.0, 0.0]
      rpy: [0.0, 0.0, 0.0]
      fov: 70
      resolution: [320, 240]
      frequency: 15
      noise: 0.01
```

La caméra appartient au robot.

Le traitement de son image appartient à la perception.

```text
Camera
   │
   ▼
FrameSource
   │
   ▼
Detector
   │
   ▼
WorldModel
```

---

# 7. Actionneurs

Un scénario peut déclarer :

```yaml
actuators:
  arm:
    type: joint_group
  gripper:
    type: vacuum
```

`type: vacuum` est à la fois la ventouse et la clé `vacuum` de `robot.tools`. L’outil est accroché par `tool_mount` sur `flange`. Le point de travail est `tcp`, décrit dans [Tools.md](Tools.md).

```yaml
robot:
  model: robot_6axis.urdf
  tools:
    vacuum: tool_vacuum.urdf
```

Les actionneurs deviennent aussi des ressources.

Une skill peut donc dire :

```text
Grasp
    requires:
        arm
        gripper
```

---

# 8. Les pannes

Les pannes sont des éléments importants pour tester l'autonomie.

Exemple :

```yaml
faults:
  - { at: 2.0, resource: wrist_camera, action: disable }
  - { at: 4.0, resource: wrist_camera, action: restore }
  - { resource: gripper, rate: 0.01 }
```

On peut donc tester :

```text
caméra fonctionne
      ↓
détection
      ↓
caméra tombe en panne
      ↓
Detect échoue
      ↓
BT réagit
      ↓
caméra revient
      ↓
mission reprend
```

Une panne ne doit pas être codée spécialement dans la skill.

Elle rend simplement une ressource indisponible.

---

# 9. Seeds et reproductibilité

Le scénario peut définir :

```yaml
seed: 42
```

La seed maître permet de reproduire :

- les positions randomisées ;
- le bruit ;
- les pannes aléatoires ;
- les expériences RL.

Exemple :

```text
seed 42
   │
   ├── world RNG
   ├── camera RNG
   ├── fault RNG
   └── RL RNG
```

Une expérience qui échoue peut donc être rejouée exactement.

---

# 10. Assertions

Les assertions constituent la partie « test » du scénario.

Exemple :

```yaml
assert:
  - robot.success
  - object("red_cube").inside("box")
  - gripper.empty
  - collisions == 0
  - time < 30
```

On peut aussi utiliser des métriques :

```yaml
assert:
  - cross_track.max < 0.08
  - laps >= 1
```

L'important est que les assertions soient **sémantiques**.

---

# 11. Scenario ≠ Behavior Tree

Le scénario dit :

```text
ce que nous voulons expérimenter
```

Le Behavior Tree dit :

```text
comment cette mission est actuellement exécutée
```

Exemple :

```text
Scenario
    goal:
        cube inside box

        │

        ▼

Behavior Tree
    Detect
      ↓
    Approach
      ↓
    Reach
      ↓
    Grasp
      ↓
    MoveTo
      ↓
    Release
```

Plus tard, le même scénario pourrait utiliser :

```text
Scenario
    │
    ▼
PDDL / GOAP
    │
    ▼
Behavior Tree généré
```

---

# 12. Vers une description de scénario plus expressive

L'idée à long terme est de se rapprocher d'un **OpenSCENARIO pour robots**, mais sans reproduire sa complexité.

On pourrait vouloir exprimer :

```text
object("red_cube").inside("box")
object("red_cube").on("table")
object("red_cube").held_by("gripper")

robot("arm").at("home")
robot("arm").collision_free()

camera("wrist").sees("red_cube")
```

Ces expressions doivent rester indépendantes de la technologie qui réalise la mission.

Elles peuvent être utilisées comme :

- objectifs ;
- préconditions ;
- assertions ;
- conditions de déclenchement.

---

# 13. Une même mission, plusieurs stratégies

Supposons :

```text
Goal:
    object("red_cube").inside("box")
```

On peut comparer :

```text
                    Goal
                      │
        ┌─────────────┼─────────────┐
        ▼             ▼             ▼
       BT           PDDL          GOAP
        │             │             │
        └─────────────┼─────────────┘
                      ▼
                    Skills
                      │
                      ▼
                   Robot
```

Cela transforme les scénarios en véritables **benchmarks robotiques**.

---

# 14. Simulation

`Simulation` orchestre l'exécution d'un scénario.

Conceptuellement :

```cpp
robotik::Scenario scenario;
scenario.load("pick_and_place.yml");

robotik::Simulation simulation(scenario);

while (!simulation.finished())
{
    simulation.step(dt);
}
```

Le simulateur graphique et le mode headless utilisent la même logique.

---

# 15. Mode graphique

```text
Robotik-Simulator
       │
       ▼
    Scenario
       │
       ▼
   Simulation
       │
       ├── Robotik core
       │
       └── SceneView
              │
              ▼
           Compages
```

Le rendu permet d'observer :

- robot ;
- monde ;
- caméra ;
- Behavior Tree ;
- skills ;
- ressources ;
- assertions.

---

# 16. Mode headless

Le même scénario peut tourner sans fenêtre :

```bash
./build/Robotik-Headless \
    build/plugins/PickAndPlaceBT/scenarios/pick_and_place.yml \
    --seed 11
```

C'est particulièrement utile pour :

- CI ;
- tests de non-régression ;
- RL ;
- benchmarks ;
- recherche de cas limites.

---

# 17. Simulation → robot réel

L'objectif final est que la frontière soit le backend :

```text
                  Scenario
                     │
                     ▼
                  Robotik
                     │
               RobotBackend
                /          \
               /            \
          MuJoCo          SO-101
         simulation         réel
```

Une mission ne devrait pas être écrite spécialement pour MuJoCo.

Elle doit manipuler le robot au travers de Robotik.

C'est ce qui permet de passer progressivement :

```text
simulation
   ↓
simulation headless
   ↓
hardware-in-the-loop
   ↓
robot réel
```

---

# 18. RL

Le scénario peut également servir à définir un environnement RL.

Par exemple :

```text
Scenario
   │
   ▼
Environment
   │
   ├── observation
   ├── action
   ├── reward
   └── reset
```

Plusieurs environnements peuvent être exécutés en parallèle :

```text
EnvironmentPool
 ├── env 0
 ├── env 1
 ├── env 2
 ├── ...
 └── env N
```

Le rendu n'est pas obligatoire.

---

# 19. Une mission comme contrat

On peut finalement considérer le scénario comme un contrat :

```text
Entrées
    ↓
Monde + robot + perturbations
    ↓
Exécution
    ↓
Observations
    ↓
Décision
    ↓
Actions
    ↓
Résultat
    ↓
Assertions
```

C'est ce qui permet à Robotik de comparer différentes architectures sans réécrire les tests.

---

# 20. Exemple complet

```yaml
scenario: pick_and_place
seed: 42

robot:
  model: ../robot_6axis.urdf

  sensors:
    wrist_camera:
      type: camera
      parent: link6
      resolution: [320, 240]
      frequency: 15
      noise: 0.01

  actuators:
    arm:
      type: joint_group

    gripper:
      type: vacuum
      length: 0.06

world:
  objects:
    red_cube:
      type: cube
      size: [0.04, 0.04, 0.04]
      position: [0.40, 0.20, 0.02]

faults:
  - { at: 2.0, resource: wrist_camera, action: disable }
  - { at: 4.0, resource: wrist_camera, action: restore }

execute:
  task: Pick the red cube and put it in the box.
  behavior_tree: pick_and_place.bt.yml

assert:
  - robot.success
  - object("red_cube").inside("box")
  - gripper.empty
  - collisions == 0
  - time < 30
```

Ce fichier contient suffisamment d'informations pour reproduire une expérience complète, sans contenir les détails de l'algorithme de cinématique, de la physique ou du rendu.

C'est précisément la séparation recherchée par Robotik.
