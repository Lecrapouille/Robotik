# Monde, scénario, expérience et simulation

Une expérience Robotik répond à une question :

> **« Dans tel monde, quand il se passe telle chose, le robot réussit-il, et comment le sait-on ? »**

La réponse tient dans trois fichiers, qui ne parlent pas de la même chose :

| Fichier | Dit | Exemple |
|---------|-----|---------|
| **Monde** (`worlds/*.yaml`) | Ce qui existe physiquement et où | le bras, son pinceau, le chevalet, le tableau, les pots |
| **Scénario** (`scenarios/*.yml`) | Ce qui se passe, quand ça s’arrête, ce qui est vérifié | peindre un portrait sans déplacer le chevalet |
| **Expérience** (`experiments/*.yaml`) | Comment lancer et comparer | trois cas (nominal, sol glissant, tableau lourd), deux graines |

Aucun ne dit **comment** le robot résout la tâche. Le scénario donne seulement une stratégie par défaut (un arbre de comportement), qu’une expérience peut remplacer. Les mêmes assertions jugent toutes les stratégies.

---

# 1. Le monde

Le monde liste des **entités**. Chacune a un identifiant, un type, un modèle (un identifiant d’asset, voir [Plugins.md](Plugins.md) §5) ou une forme primitive, et une pose.

```yaml
world:
  id: painting_lab
  floor: { friction: 0.8 }

  entities:
    - id: robot
      type: robot
      model: robotik.robots.robot_6axis
      home: { joint1: 0.0, joint2: 0.3, joint3: 1.3, joint4: 0.0, joint5: 1.54, joint6: 0.0 }
      actuators:
        arm: { type: joint_group }

    - id: brush
      type: tool
      model: robotik.tools.default
      attach: { parent: robot.flange, frame: tool_mount }

    - id: easel
      type: body
      model: robotik.painter.easel
      physics: free
      friction: 0.6
      pose: { position: [0.65, 0.0, 0.0], rpy: [0.0, 0.0, 3.141592653589793] }

    - id: canvas
      type: body
      shape: { type: box, size: [0.02, 0.40, 0.50] }
      mass: 0.5
      physics: free
      attach:
        parent: easel.canvas_mount
        transform: { position: [0.011, 0.0, 0.251] }

    - id: red
      type: prop
      shape: { type: box, size: [0.07, 0.07, 0.07] }
      color: [0.82, 0.18, 0.18]
      pose: { position: [0.42, 0.22, 0.035] }
```

Les repères sont Z vers le haut, en mètres et radians, comme l’URDF.

## Les types d’entités

| Type | Ce que c’est | Physique |
|------|--------------|----------|
| `robot` | Le robot piloté (un seul). `home`, `sensors`, `actuators`. | Chaîne du backend, fixée à sa `pose` |
| `tool` | Un outil, qui garde son identité une fois monté. Il apporte ses propres actionneurs et capteurs. | Accroché à la bride |
| `body` | Un corps rigide simulé : un modèle URDF (`free` ou `fixed`) ou une boîte (`free`, avec `mass`). | Oui |
| `prop` | Un objet cinématique : immobile sauf si le système de préhension le déplace. Forme `box` ou `container` (bac ouvert). | Non |

`randomize: { x: [min, max], y: ..., z: ... }` décale la position vraie d’un `prop` à chaque reset, tirée de la graine. Le robot ne connaît que la position nominale.

## Attacher une entité

`attach.parent` vise une entité, ou l’un de ses liens : `easel.canvas_mount` est le lien `canvas_mount` de l’URDF du chevalet. `transform` place l’entité dans ce repère. Pour un corps libre, c’est sa pose de départ ; la physique décide ensuite.

L’URDF décrit la structure d’un modèle. Le monde décrit l’instance : où elle est, et de quoi elle dépend.

## Les outils

Un outil s’accroche toujours par son lien `tool_mount` sur la bride du robot (`robot.flange`), voir [Tools.md](Tools.md). Un seul est monté à la fois. Les autres outils déclarés sans `attach` sont **rangés** : ils ne sont pas construits, mais le simulateur peut les monter à la place du premier (panneau **Teach**).

```yaml
    - id: vacuum
      type: tool
      model: robotik.tools.vacuum
      attach: { parent: robot.flange, frame: tool_mount }
      actuators:
        gripper: { type: vacuum }
    - { id: drill, type: tool, model: robotik.tools.drill }
```

L’actionneur `gripper` est celui de la ventouse. Monter la perceuse retire la ventouse et son actionneur : rien ne colle plus à la bride.

## Capteurs et actionneurs

Chaque capteur et chaque actionneur est aussi une **ressource** du même nom : les skills la réservent, les pannes la rendent indisponible.

```yaml
      sensors:
        wrist_camera: { type: camera, parent: link6, position: [0.05, 0.0, 0.0],
                        fov: 70, resolution: [320, 240], frequency: 15, noise: 0.01 }
        imu:
          type: imu
          parent: base_link
          frequency: 200
          latency: 0.002
          accelerometer: { sigma: 0.02, bias: 0.05, saturation: [-160, 160] }
          gyroscope: 0.001
        lidar: { type: range, parent: base_link, beams: 180, fov: 360, max_range: 10,
                 noise: { sigma: 0.01, resolution: 0.001 } }
        wrist_ft: { type: force_torque, link: link6, force: 0.5, torque: 0.02 }
      actuators:
        arm: { type: joint_group }                       # sans joints : tous
        left_wheel: { type: motor, joint: left_wheel_joint }
        gripper: { type: vacuum }
```

| Capteur | Mesure | Repère des données |
|---------|--------|--------------------|
| `camera`, `rgbd` | image (et profondeur) | optique : z devant, x à droite, y en bas |
| `imu` | force spécifique (+9,81 vers le haut au repos), vitesse angulaire, attitude | le capteur |
| `range` | une distance par rayon, `max_range` quand rien n’est touché | plan XY du capteur |
| `force_torque` | torseur reçu par le lien depuis son parent | le lien mesuré |

Un canal de bruit s’écrit comme un écart type, ou comme un modèle complet : `{ sigma, bias, resolution, saturation: [min, max] }`, appliqués dans cet ordre. `latency` retarde la livraison d’une mesure, qui garde son horodatage de capture. Toute lecture porte un en-tête commun : horodatage, repère, numéro de séquence, validité (fausse tant que la ressource est en panne).

---

# 2. Le scénario

Exemple complet ; le fichier du dépôt n’utilise pas toutes ces clés :

```yaml
scenario:
  id: pick_and_place
  description: Pick the red cube and put it in the box.
  world: ../worlds/pick_place_cell.yaml
  seed: 42

  controller:
    behavior_tree: pick_and_place.bt.yml

  trajectories:
    drop: trajectories/drop.yaml

  perception: { oracle: when_blind }

  faults:
    - { at: 4.0, resource: wrist_camera, action: disable }
    - { at: 6.0, resource: wrist_camera, action: restore }
    - { resource: gripper, rate: 0.01 }

  timeout: 60
  stop_when: inside(red_cube, box)

  assertions:
    - controller.success
    - inside(red_cube, box)
    - { condition: "collision_count(robot) == 0", phase: always }
```

| Champ | Rôle |
|-------|------|
| `world` | Le fichier monde, relatif au scénario. |
| `seed` | Graine maîtresse : disposition, bruit des capteurs, pannes aléatoires en dérivent. |
| `controller` | La stratégie par défaut. Aujourd’hui un arbre de comportement ; sans contrôleur, la mission du plugin pilote. |
| `trajectories` | Chemins nommés, chacun dans son fichier (§4). |
| `perception.oracle` | Quand la vérité terrain peut remplacer la perception (§5). |
| `faults` | Pannes programmées (`at`) ou aléatoires (`rate` par seconde). Une panne rend une ressource indisponible ; aucune skill n’est écrite pour la gérer spécialement. |
| `timeout` | Durée simulée maximale. |
| `stop_when` | Condition qui termine la course dès qu’elle est vraie. |
| `assertions` | Ce qui est vérifié (§3). |

## Quand une course se termine

La course s’arrête quand le contrôleur réussit ou échoue, quand `stop_when` devient vraie, quand `timeout` est atteint, ou quand une assertion déclarée `stop: true` échoue. La raison est donnée par `Simulation::outcome()`. La physique continue ensuite (le simulateur garde l’image vivante), mais les verdicts sont figés.

---

# 3. Les assertions

Une assertion est une **condition** jugée à un **moment**.

```yaml
assertions:
  - controller.success                                         # raccourci : phase end
  - { condition: "collision_count(robot) == 0", phase: always }
  - { condition: "pose_error(easel) < 0.002", phase: always }
  - { condition: "on(canvas, easel)", phase: end }
  - { condition: "distance(robot.tcp, red_cube) < 0.05", within: 5.0 }
  - { condition: "holding(gripper)", eventually: true }
  - { condition: "time > 1", at: 2.0 }
  - name: brush loaded before the first stroke
    sequence: [ "skill_succeeded(dip)", "skill_started(mark)" ]
  - { condition: "joint_limits_respected(robot)", phase: always, stop: true }
```

| Moment | Verdict |
|--------|---------|
| `phase: always` | Doit être vraie à chaque pas. La première violation est retenue (valeur, instant). |
| `phase: end` | Jugée à la fin. Pendant la course, le simulateur affiche sa valeur courante. |
| `at: t` | Jugée une fois, à l’instant `t`. Échoue si la course finit avant. |
| `within: t` | Doit devenir vraie avant `t`. |
| `eventually: true` | Doit devenir vraie avant la fin. |
| `sequence: [...]` | Les conditions doivent devenir vraies dans cet ordre. |

Une assertion en échec garde la **valeur mesurée**, la **borne attendue** et le **temps simulé**. Elle n’arrête pas la course, sauf `stop: true`.

## Les conditions

`<valeur> <op> <nombre>`, avec `op` parmi `== != < <= > >=`, ou une valeur seule, vraie si non nulle. Une valeur est une métrique ou une requête sur le monde.

| Valeur | Mesure |
|--------|--------|
| `time` | Temps simulé (s) |
| `controller.success` | 1 si le contrôleur a réussi |
| métriques de la mission | `laps`, `cross_track.max`, `ik.error`… publiées par `Mission::measure` |
| `pose_error(e)` | Déplacement de `e` depuis le début de l’épisode (m) |
| `orientation_error(e)` | Rotation de `e` depuis le début (rad) |
| `distance(a, b)` | Distance entre deux entités ou repères (`robot.tcp`) (m) |
| `inside(a, b)` | `a` repose dans le bac `b`, ou dans la boîte `b` |
| `on(a, b)` | `a` est posé sur `b` : contact physique sous `a`, ou faces jointives pour deux `prop` |
| `touching(a, b)` | Contact entre `a` et `b` |
| `collision_count(e)` | Pic de contacts simultanés de `e` avec d’autres entités, sol exclu, depuis le reset. Pour le robot : outil monté compris, et un objet porté qui traverse une paroi compte. |
| `tool_is_attached(t, robot.flange)` | 1 si l’outil `t` est monté sur ce repère |
| `joint_limits_respected(robot)` | 1 si toutes les articulations sont dans leurs limites (à 1e-3 près) |
| `holding(g)`, `holding(g, objet)` | La ventouse `g` tient quelque chose, ou cet objet |
| `skill_started(s)`, `skill_succeeded(s)`, `skill_failed(s)` | Historique du scheduler |

Une valeur inconnue, ou une requête sur une entité inconnue, fait échouer l’assertion avec la raison.

**Zéro collision ne prouve pas que le chevalet n’a pas bougé.** Il peut avoir glissé sous un contact mal modélisé. C’est pourquoi le Painter vérifie aussi `pose_error(easel)` : sur la course nominale, le chevalet bouge de 0,03 mm et le tableau se tasse de 1,8 mm.

---

# 4. Les trajectoires

Une trajectoire vit dans son propre fichier, avec son propre format. Le scénario la nomme.

```yaml
# scenarios/trajectories/red_stroke.yaml
trajectory:
  type: cartesian
  frame: easel.canvas_mount
  orientation: { rpy: [1.5707963267948966, 0.0, -1.5707963267948966] }
  points:
    - [0.18, 0.12, 0.16]
    - [0.08, 0.08, 0.16]
    - { position: [0.06, 0.08, 0.16], rpy: [1.5707963267948966, 0.0, -1.5707963267948966] }
```

| Type | Points |
|------|--------|
| `cartesian` | Poses du point de travail de l’outil, dans `frame` (une entité ou `entité.lien` ; le monde si vide) |
| `joint` | Configurations des articulations listées dans `joints` |

`Simulation::waypoints("red")` rend une trajectoire cartésienne dans le repère de base du robot. Son repère est placé là où le **fichier monde** le décrit (`Simulation::described`) : c’est ce qu’on a dit au robot, pas la vérité courante. Si le chevalet glisse pendant la course, le robot ne le sait pas.

---

# 5. Vérité, description et croyance

Trois choses à ne pas confondre :

| | Ce que c’est | Qui y a accès |
|--|--------------|---------------|
| **Vérité terrain** | L’état du monde simulé : `Simulation::pose`, `Simulation::query` | Les assertions, les rapports, l’environnement RL |
| **Description** | Le monde tel que le fichier le décrit au départ : `Simulation::described`, `waypoints` | La mission, pour planifier |
| **Croyance** | Ce que le robot pense savoir : le `WorldModel`, nourri par la perception | Les skills, via `RobotContext` |

Les skills ne voient que le `RobotContext` : elles ne peuvent pas tricher. La vérité terrain ne passe dans la croyance que par l’**oracle**, et seulement si le scénario le demande :

```yaml
perception: { oracle: when_blind }    # never (défaut) | when_blind | always
```

`when_blind` active l’oracle quand aucune caméra n’a de source d’image, c’est-à-dire en headless. Le simulateur, lui, rend l’image et utilise la vraie perception. Les rapports signalent les courses obtenues avec l’oracle.

---

# 6. L’expérience

L’expérience du dépôt, avec en plus un changement de contrôleur pour l’exemple :

```yaml
experiment:
  id: pick_and_place_benchmark
  seeds: [11, 12, 13]

  cases:
    - id: nominal
      scenario: ../scenarios/pick_and_place.yml

    - id: wide_layout
      scenario: ../scenarios/pick_and_place.yml
      overrides:
        world.entities.red_cube.randomize: { x: [-0.05, 0.05], y: [-0.05, 0.05] }

    - id: camera_down_at_start
      scenario: ../scenarios/pick_and_place.yml
      controller: { behavior_tree: ../scenarios/pick_and_place.bt.yml }
      setup:
        - inject_fault: { resource: wrist_camera, at: 0.0, action: disable }
        - inject_fault: { resource: wrist_camera, at: 1.5, action: restore }

  evaluation:
    assertions:
      - { condition: "joint_limits_respected(robot)", phase: always }
    metrics: [ time, collision_count(robot), distance(red_cube, box) ]
```

| Champ | Rôle |
|-------|------|
| `seeds` | Chaque cas tourne avec chaque graine. Sans liste, chaque scénario garde la sienne. |
| `world` | Remplace le monde de tous les scénarios. |
| `cases[].overrides` | Chemins pointés appliqués aux fichiers **avant** leur lecture : `world.…` ou `scenario.…`. Dans une liste, un segment désigne l’élément de cet `id` (ou cet indice). |
| `cases[].controller` | Remplace la stratégie du scénario. |
| `cases[].setup` | Pannes ajoutées à celles du scénario. |
| `evaluation.assertions` | Ajoutées aux assertions de chaque scénario. |
| `evaluation.metrics` | Valeurs relevées à la fin de chaque course (mêmes expressions que les conditions). |

Un cas est une **variante**, pas une mutation du cas précédent : chaque course relit ses fichiers et reconstruit son monde. Plusieurs graines d’un même cas repartent du même monde, réinitialisé.

---

# 7. Lancer

```bash
# Un scénario
./build/Robotik-Headless build/plugins/PickAndPlaceBT/scenarios/pick_and_place.yml --seed 7

# Une expérience, avec un rapport JSON
./build/Robotik-Headless \
    build/plugins/PickAndPlaceBT/experiments/pick_and_place_benchmark.yaml \
    --report pick.json
```

Sortie d’une expérience :

```text
[PASS] nominal                  seed=11     controller succeeded at 7.4 s (oracle perception)  time=7.4  ...
[PASS] camera_down_at_start     seed=11     controller succeeded at 8.9 s (oracle perception)  time=8.9  ...
pick_and_place_benchmark: 9/9 runs passed
```

Le JSON reprend, pour chaque course : cas, scénario, graine, issue, temps, perception, métriques, et chaque assertion avec son verdict, sa valeur, sa borne et son instant. Code de sortie : 0 si tout passe, 2 si une course échoue, 1 si un fichier ou un plugin ne se charge pas.

`--trace` affiche en plus l’historique des skills.

Dans le simulateur, le panneau **Scenario** montre la vérité contre la croyance, l’issue de la course et les assertions : `[pass]`, `[FAIL]` avec la valeur, la borne et l’instant, ou la valeur courante tant qu’elle est en attente.

---

# 8. Graines et reproductibilité

```text
seed 42
   ├── world      disposition des props
   ├── faults     pannes aléatoires
   ├── <capteur>  un flux par capteur, à son nom
   └── <mission>  ce que la mission en dérive (politique RL…)
```

Même scénario, même graine : même course, aux arrondis près. Un canal de bruit idéal ne tire aucun nombre, donc couper le bruit d’un capteur ne décale pas les autres. Une course qui échoue se rejoue avec sa graine, lue dans le rapport.

---

# 9. En C++

```cpp
robotik::AssetResolver const assets = robotik::AssetResolver::standard();
compages::world::World world;
robotik::Simulation simulation(world, robotik::Scenario::load(path, assets));
simulation.reset(robotik::Seed{ 7 });
while (!simulation.finished())
{
    simulation.step(Seconds(0.01));
}
for (robotik::Check const& check : simulation.checks())
{
    if (check.verdict == robotik::Check::Verdict::Failed)
    {
        std::cerr << check.text << ": " << check.detail << '\n';
    }
}
```

`robotik::ScenarioSource` garde les deux fichiers sous forme d’arbre modifiable (`ConfigNode`) : `source.override("world.entities.easel.friction", ConfigNode("0.3"))`, puis `Scenario::read(source, assets)`. C’est ce que fait une expérience.

Le scénario sert ainsi de démo, de test de non-régression, de benchmark, et de banc pour comparer des stratégies : seul le contrôleur change, les assertions restent.
