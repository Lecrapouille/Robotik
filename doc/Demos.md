# Les démos Robotik

Les démos ont deux rôles qui se complètent :

1. vous montrer comment utiliser Robotik au quotidien ;
2. servir de petits laboratoires pour essayer de nouvelles idées.

Elles ne remplacent pas le cœur de la bibliothèque, qui reste **`librobotik-core`**.

---

# 1. Ce qu'est une démo Robotik

Une démo typique assemble plusieurs briques :

```text
Monde + scénario + expérience (YAML)
     +
Mission C++
     +
Robotik core
     +
backend / perception / affichage
```

Le YAML pose le cadre de l'expérience : le monde et ses entités, ce qui se passe, ce qui compte comme une réussite, et les variantes à comparer ([Scenario-et-Simulation.md](Scenario-et-Simulation.md)).

Le C++ apporte ce qui est propre à la démo : skills, détecteurs, métriques, politiques RL ou intégrations externes.

---

# 2. Les applications

Le dépôt propose cinq exécutables :

```text
Robotik-Simulator
Robotik-Headless
Robotik-LineFollower
Robotik-PickAndPlaceRL
Robotik-Fly
```

**Robotik-Simulator** (`simulator/`) regarde une mission se dérouler. Son menu **Demos** liste les paquets compilés dans `build/plugins/`.

**Robotik-Headless** (`headless/`) exécute le même scénario sans fenêtre, ou toute une expérience, et écrit un rapport (`--report rapport.json`).

Les sources des démos sont dans `plugins/`. Chaque dossier est un plugin : `plugin.yaml`, le code C++ dans `src/`, puis `worlds/`, `scenarios/` et `experiments/`. Le fonctionnement et un tutoriel pour en ajouter un sont dans [Plugins.md](Plugins.md). Line follower, le pool RL et la mouche gardent en plus un exécutable (`plugins/LineFollower/app`, `plugins/PickAndPlaceRL/app`, `plugins/FlyBrain/app`).

---

# 3. Pick and Place

C'est la démo de référence : celle à regarder en premier pour comprendre l'architecture de bout en bout.

Le but : amener le cube rouge dans la boîte. [🎥 Démonstration vidéo sur YouTube](https://youtu.be/MuWdpaLvGoU?si=qjJWl1kZFv4DRHOo)


```text
         red_cube
             │
             ▼
        ┌─────────┐
        │  boîte  │
        └─────────┘
```

Le déroulé normal :

```text
Detect
  ↓
Approach
  ↓
Reach
  ↓
Grasp
  ↓
Lift
  ↓
MoveTo(box)
  ↓
Release
```

Elle met en jeu un bras six axes, une caméra, une chaîne de perception, des skills, un behavior tree, un gripper simulé, MuJoCo, et des assertions en fin de mission.

---

# 4. Le scénario nominal

Le fichier :

```text
plugins/PickAndPlaceBT/scenarios/pick_and_place.yml
```

Il décrit la mission « qui se passe bien », sans aucune panne, dans le monde `plugins/PickAndPlaceBT/worlds/pick_place_cell.yaml` : le bras, sa caméra, la ventouse montée et quatre outils rangés, le cube et le bac.

Pour la lancer avec l'interface :

```bash
./build/Robotik-Simulator
```

Le menu **Demos → Pick and place** ouvre ce scénario. En ligne de commande, après compilation :

```bash
./build/Robotik-Simulator \
    build/plugins/PickAndPlaceBT/scenarios/pick_and_place.yml
```

Et le même scénario en headless, avec une graine reproductible :

```bash
./build/Robotik-Headless \
    build/plugins/PickAndPlaceBT/scenarios/pick_and_place.yml \
    --seed 11
```

---

# 5. Le scénario avec pannes

Le fichier :

```text
plugins/PickAndPlaceBT/scenarios/pick_and_place_faults.yml
```

Cette variante ajoute notamment une panne de caméra.

Dans **Robotik-Simulator**, choisir **Demos → Pick and place → Pick and place (faults)**, ou lancer :

```bash
./build/Robotik-Simulator build/plugins/PickAndPlaceBT/scenarios/pick_and_place_faults.yml
```

Le but n'est pas simplement de constater que ça échoue. On veut vérifier que l'architecture réagit intelligemment : réessayer, se replier sur autre chose, ou abandonner.

```text
Camera
  │
  ▼
Detect
  │
  X panne
  │
  ▼
FAILURE
  │
  ▼
Behavior Tree
  │
  ├── retry
  ├── fallback
  └── abort
```

Les pannes deviennent ainsi un moyen de tester le comportement autonome, et pas seulement la cinématique.

---

# 6. Les assertions du pick-and-place

La mission exige :

```yaml
assertions:
  - controller.success
  - inside(red_cube, box)
  - holding(gripper) == 0
  - { condition: "collision_count(robot) == 0", phase: always }
  - time < 30
```

C'est bien plus pertinent que de tester des angles articulaires. Un autre contrôleur peut très bien suivre une trajectoire différente et pourtant remplir la mission.

En headless, aucune image n'est rendue : le scénario demande `perception: { oracle: when_blind }`, et la position vraie du cube remplace la caméra. Les rapports le signalent (`oracle perception`). Le simulateur, lui, rend l'image et utilise le détecteur de couleur.

L'expérience `plugins/PickAndPlaceBT/experiments/pick_and_place_benchmark.yaml` rejoue la mission sur trois graines, avec un cube plus dispersé et avec une caméra en panne au démarrage :

```bash
./build/Robotik-Headless \
    build/plugins/PickAndPlaceBT/experiments/pick_and_place_benchmark.yaml \
    --report pick.json
```

---

# 6 bis. Le peintre

Un bras trempe un pinceau dans quatre pots, peint un trait par couleur sur un tableau posé sur un chevalet, et rince le pinceau entre deux couleurs.

Le chevalet est un corps libre : s'il est poussé, il glisse ou bascule. Le tableau est une boîte libre posée sur son rebord. Tous deux sont des entités de `plugins/Painter/worlds/painting_lab.yaml` ; les traits sont des trajectoires de `plugins/Painter/scenarios/trajectories/`, exprimées dans le repère du chevalet.

Zéro collision ne prouve pas que le chevalet est resté en place. Le scénario le mesure :

```yaml
  - { condition: "pose_error(easel) < 0.002", phase: always }
  - { condition: "pose_error(canvas) < 0.005", phase: always }
  - { condition: "on(canvas, easel)", phase: end }
  - name: brush loaded before the first stroke
    sequence: [ "skill_succeeded(dip)", "skill_started(mark)" ]
```

L'expérience `painting_benchmark` reprend le portrait sur un sol glissant et avec un tableau quatre fois plus lourd.

---

# 7. Le line follower

Cette démo aborde une autre famille de problèmes : un robot mobile, de la perception et une boucle de contrôle.
[🎥 Démonstration vidéo sur YouTube](https://youtu.be/T0Oop_cmlPw?si=UKg83e5xunciYAZ1)

```text
             caméra
                │
                ▼
           perception
                │
                ▼
          localisation
                │
                ▼
          contrôleur
                │
                ▼
        robot différentiel
```

Le robot se repère grâce à des AprilTags posés au sol, puis suit une ligne.

L'odométrie simulée est volontairement faussée. On voit ainsi à quoi sert une observation extérieure : corriger l'estimation de la position.

---

# 8. Pourquoi introduire une erreur exprès ?

Une simulation parfaite cache souvent les vraies difficultés du terrain.

Si l'on posait :

```text
odométrie = vérité
```

la localisation ne prouverait plus grand-chose.

Dans la démo, le simulateur donne la vérité terrain, pendant que l'odométrie dérive. Les AprilTags permettent de comparer l'estimation à la référence :

```text
vérité
  │
  ├── simulateur
  │
  └── odométrie volontairement biaisée
              │
              ▼
          estimation
              │
              ▼
           AprilTag
              │
              ▼
        correction pose
```

On peut afficher côte à côte :

```text
ground truth
estimated pose
```

---

# 9. Les commandes du LineFollower

Exécution avec sauvegarde de la carte :

```bash
./build/Robotik-LineFollower \
    --seed 4 \
    --laps 1 \
    --save map.png
```

Avec le flux de la caméra à l'écran :

```bash
./build/Robotik-LineFollower \
    --view
```

Ou via le binaire headless générique :

```bash
./build/Robotik-Headless \
    build/plugins/LineFollower/scenarios/line_follower.yml \
    --seed 4
```

---

# 10. Pick and Place RL

Cette démo met en place la boucle classique d'un environnement d'apprentissage :

```text
observation
     │
     ▼
  policy
     │
     ▼
   action
     │
     ▼
  physics
     │
     ▼
  reward
     │
     └──────────► observation
```

Pour l'instant, le but est surtout de valider l'infrastructure : `Environment`, pool parallèle, reproductibilité.

L'observation, l'action et la récompense sont trois pièces (`PickPlaceObservation`, `PickPlaceAction`, `PickPlaceReward`), partagées par le pool en ligne de commande et par le plugin du simulateur. L'observation lit ce que le robot croit savoir ; seul l'indicateur « livré » vient de la vérité terrain, et il est marqué comme tel. Le détail est dans [Apprentissage.md](Apprentissage.md).

Un vrai entraînement de type PPO ou SAC n'est pas l'objet de cette démo.

---

# 11. Environnements parallèles

L'idée est d'alimenter plusieurs copies du même environnement en même temps :

```text
env 0 ─┐
env 1 ─┤
env 2 ─┤
env 3 ─┼──► policy
...    │
env N ─┘
```

Un exemple de lancement :

```bash
./build/Robotik-PickAndPlaceRL \
    --policy converged \
    --envs 16 \
    --threads 8 \
    --episodes 32
```

On peut ainsi vérifier que les résultats restent identiques d'une exécution à l'autre, même avec de nombreux environnements en parallèle. La dernière colonne donne l'issue de chaque épisode : `success`, `failure`, `time limit` (budget de pas épuisé) ou `aborted` (arrêt technique). `--log episodes.json` écrit le journal des épisodes, avec la version et le commit de Robotik.

---

# 12. Fly brain

This demo puts an agent on the same loop as the RL one: observation, action, environment, seed, headless execution, and several worlds in parallel. [🎥 Démonstration vidéo sur YouTube](https://youtu.be/ao2lA3Rlmrc?si=DECtcn7l44Q-iMCo)

```text
sensors → observation → brain → action → controller → body
```

The brain knows neither MuJoCo, nor Compages, nor the URDF. Two brains share that surface: a reflex rule, and the integrate-and-fire neuron of Shiu et al. 2024. The second can load the FlyWire connectome (`--edges`, `--binding`); without those files it runs on a small circuit that uses the same equations.

The body is the ×100 model (`data/drosophila_x100.urdf`, about 28 cm). Flight is plain kinematics: the wings, the legs, and the neck are a posture, not a wind tunnel.

```bash
./build/Robotik-Fly --brain reflex
./build/Robotik-Fly --view
./build/Robotik-Fly --headless --envs 8 --episodes 8 --seed 123456
```

The scenario is `plugins/FlyBrain/scenarios/fly_obstacle_avoidance.yml`: reach the food while avoiding the obstacles. In the simulator, open it from **Demos → Fly**: the panel *Fly brain* shows the state and switches to the FlyWire connectome. Headless runs it like any scenario:

```bash
./build/Robotik-Headless build/plugins/FlyBrain/scenarios/fly_obstacle_avoidance.yml
```

# 13. Pourquoi le rendu n'est pas obligatoire

Avec 16, 100 ou 1000 environnements, afficher chaque robot n'apporte rien à l'entraînement.

Robotik sépare donc la **simulation** (état, physique, capteurs, observations) :

```text
Simulation
    │
    ├── état
    ├── physique
    ├── capteurs
    └── observation
```

de la **visualisation** :

```text
SceneView
    │
    └── affichage d'un ou plusieurs environnements
```

Le rendu devient un outil d'observation qu'on sort quand on en a besoin, pas une dépendance du cœur RL.

---

# 14. Écrire sa propre démo

Une bonne démo part d'une question précise, à laquelle on peut répondre par oui ou par non. Par exemple :

```text
Puis-je détecter un objet avec une caméra ?
Puis-je localiser un robot avec des AprilTags ?
Puis-je faire échouer proprement une skill ?
Puis-je replanifier après une panne ?
Puis-je comparer deux stratégies de contrôle ?
Puis-je exécuter le même scénario avec deux backends ?
```

Ensuite, concrètement :

```text
1. créer le scénario YAML
2. créer les skills nécessaires
3. brancher les détecteurs
4. définir les assertions
5. ajouter une application si nécessaire
```

---

# 15. Une démo qui grandit devient un scénario

Quand une démo est stable, on doit pouvoir la lancer indifféremment ainsi :

```bash
Robotik-Simulator scenario.yml
```

ou ainsi :

```bash
Robotik-Headless scenario.yml
```

C'est de cette façon qu'une démonstration se transforme peu à peu en test de non-régression.

---

# 16. Les démos comme laboratoire

Robotik est fait pour expérimenter. Un même scénario peut servir de banc d'essai pour comparer des stratégies de haut niveau :

```text
                 même scénario
                       │
        ┌──────────────┼──────────────┐
        ▼              ▼              ▼
       BT            PDDL            GOAP
        │              │              │
        └──────────────┼──────────────┘
                       ▼
                     Skills
```

On peut alors comparer, sur des exécutions strictement identiques :

- le temps d'exécution ;
- les collisions ;
- le nombre d'échecs ;
- la précision ;
- la consommation de ressources ;
- la robustesse aux pannes.

---

# 17. Des démos à imaginer

L'architecture laisse la place à plusieurs prolongements naturels.

### SO-101

```text
simulation MuJoCo
       │
       ▼
    Robotik
       │
       ▼
  SO101Backend
       │
       ▼
   robot réel
```

Le scénario YAML pourrait rester tel quel ; seul le backend changerait.

### Perception RGB-D

```text
Camera
 ↓
Depth
 ↓
Open3D / OpenCV
 ↓
WorldModel
 ↓
Pick
```

### Planification

```text
Goal
 ↓
PDDL / GOAP
 ↓
BT
 ↓
Skills
```

### Raisonnement

```text
WorldModel
 ↓
Prolog
 ↓
facts / rules
 ↓
decision
```

Toutes ces variantes restent comparables, puisqu'elles partagent les mêmes fichiers de scénario.

---

# 18. L'esprit des démos

Une bonne démo Robotik cherche à être :

- **petite** : un seul concept par démo ;
- **reproductible** : graine, assertions, mode headless ;
- **lisible** : du YAML et peu de code métier ;
- **observable** : simulateur ou traces quand c'est utile ;
- **testable** : des critères de réussite explicites ;
- **réutilisable** : le scénario survit à l'application de démonstration.

L'objectif n'est pas de livrer une application finie, mais de mettre en lumière **une idée de robotique**, bien isolée.
