# Scénario YAML et Simulation

Un **scénario** décrit la mission de façon déclarative : quel robot avec quels capteurs et actionneurs, quels objets (et comment les placer aléatoirement), quelles pannes injecter, quel behavior tree exécuter, et comment juger le résultat. La logique de mouvement reste dans les **skills** C++.

## Où ça vit dans le code

| Élément | Fichier / type | Rôle |
|--------|----------------|------|
| Schéma parsé | `robotik::Scenario` (`Scenario/Scenario.hpp`) | Rempli par `Scenario::load` |
| Exécution | `robotik::Simulation` (`Runtime/Simulation.hpp`) | Robot, objets, skills, BT, pannes, assertions |
| Applications | `Robotik-Headless`, `Robotik-Simulator` | `Scenario::load` puis boucle `Simulation::step` |

## Structure du YAML

Exemple de référence : `data/scenarios/pick_and_place.yml` ; variante avec pannes : `pick_and_place_faults.yml`.

```yaml
scenario: pick_and_place
seed: 42                      # graine maître

robot:
  model: ../robot_6axis.urdf  # relatif au fichier scénario
  home: { joint1: 0.0, joint2: 0.3, ... }   # radians / mètres
  sensors:
    wrist_camera:
      type: camera            # camera | rgbd
      parent: link6           # lien URDF porteur
      position: [0.05, 0.0, 0.0]
      rpy: [0.0, 0.0, 0.0]    # repère optique : z devant, x à droite, y en bas
      fov: 70                 # vertical, degrés
      resolution: [320, 240]
      frequency: 15           # Hz
      noise: 0.01             # bruit gaussien, fraction de la plage
  actuators:
    arm: { type: joint_group }                 # joints: [...] ; vide = tous
    gripper: { type: vacuum, length: 0.06 }    # parent: lien (défaut : outil)
    # wheel: { type: motor, joint: left_wheel_joint }

world:
  objects:
    red_cube:
      type: cube              # cube | box
      size: [0.04, 0.04, 0.04]
      color: [0.9, 0.1, 0.1]
      position: [0.40, 0.20, 0.02]          # centre, repère base robot
      randomize: { x: [-0.02, 0.02], y: [-0.02, 0.02] }

faults:
  - { at: 0.0, resource: wrist_camera, action: disable }
  - { at: 1.0, resource: wrist_camera, action: restore }
  - { resource: gripper, rate: 0.01 }       # panne aléatoire, taux par seconde

execute:
  task: Pick the red cube and put it in the box.
  behavior_tree: pick_and_place.bt.yml

assert:
  - robot.success
  - object("red_cube").inside("box")
  - gripper.empty
  - collisions == 0
```

Chaque capteur et actionneur est une **ressource** du même nom : les skills la réservent, les pannes la rendent indisponible. Sans section `actuators`, la simulation crée `arm` (tous les joints) et `gripper` (ventouse sur l’outil).

### Assertions

Évaluées par `Simulation::checks()` :

- `robot.success` : le BT a terminé en SUCCESS ;
- `object("a").inside("b")` : boîte englobante ;
- `gripper.empty` : la ventouse ne tient rien ;
- `collisions == 0` : pic de contacts MuJoCo.

## Seeds et rejouabilité

`Simulation::reset(Seed)` (par défaut la `seed` du scénario) dérive une graine par sous-système :

| Graine | Usage |
|--------|-------|
| `seed.derive("world")` | Placement aléatoire des objets (`randomize`) |
| `seed.derive("faults")` | Pannes aléatoires (processus de Poisson) |
| `seed.derive(<nom du capteur>)` | Bruit de chaque capteur |

Même seed, même scénario : même trace, au bit près (`Robotik-Headless scenario.yml --seed N`). Le robot ne connaît que les positions nominales des objets : son `WorldModel` part de ces a priori et les corrige par la perception.

## Cycle de vie d’une run

1. `Scenario::load(path)` : parse et résolution des chemins.
2. `Simulation(world, scenario, view = nullptr)` :
   - `RobotSession` sur l’URDF, `MujocoBackend`, posture `home` ;
   - capteurs et actionneurs ; avec une `SceneView`, chaque caméra reçoit sa source d’images rendue, branchée sur `PerceptionPipeline` → `WorldModel` ;
   - objets (entités Compages `SceneObject`) ;
   - skills enregistrées au scheduler, puis exposées au BT (`registerSkills`).
3. `reset(seed)` : ressources restaurées, objets replacés, a priori du `WorldModel`, robot en `home`, BT rechargé.
4. Boucle `step(dt)` : pannes → BT → scheduler → robot (physique, capteurs, perception) → ventouse.
5. `finished()` quand le BT est en SUCCESS ou FAILURE ; puis `checks()`.

**Headless** : pas de vue, donc pas d’image ; le `WorldModel` reçoit la vérité terrain tant que la caméra est disponible (oracle de perception). **Simulateur** : rendu Compages de la caméra poignet, `ColorDetector` (application) dans la `PerceptionPipeline`, panneaux ressources / skills / pannes.

## Skills disponibles dans le BT

| Action | Ressources | Rôle |
|--------|-----------|------|
| `Home` | `arm` | Posture `home` |
| `Detect(o)` | caméra (partagée) | Attend une observation récente de `o` |
| `Approach(o)` | `arm` | 10 cm au-dessus de `o` (croyance du `WorldModel`) |
| `Reach(o)` | `arm` | Ventouse au contact du dessus de `o` |
| `Grasp(o)` | `gripper` | Aspiration ; précondition « gripper is empty » |
| `Release` | `gripper` | Relâche |
| `Stop` | tous les actionneurs | Arrêt d’urgence, priorité 1000 : préempte tout |

## Extension

1. Copier le scénario et le BT ;
2. ajouter capteurs, actionneurs, objets, pannes ;
3. pour de nouvelles actions, enregistrer des skills dans `Simulation::addSkills` (`src/Robotik/Runtime/Simulation.cpp`), ou construire son propre `SkillScheduler` comme la démo LineFollower ;
4. ajouter des `assert`, ou étendre `Simulation::checks`.

Le parseur est en YAML aujourd’hui ; un lexer/parser maison avec AST est prévu pour plus tard, d’où la séparation nette entre `Scenario` (données) et `Simulation` (exécution).
