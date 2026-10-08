# Les démos Robotik

Les démos ont deux rôles qui se complètent :

1. vous montrer comment utiliser Robotik au quotidien ;
2. servir de petits laboratoires pour essayer de nouvelles idées.

Elles ne remplacent pas le cœur de la bibliothèque, qui reste **`librobotik-core`**.

---

# 1. Ce qu'est une démo Robotik

Une démo typique assemble plusieurs briques :

```text
Scenario YAML
     +
Mission C++
     +
Robotik core
     +
backend / perception / affichage
```

Le YAML pose le cadre de l'expérience : le monde, le robot, la mission, et ce qui compte comme une réussite.

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

**Robotik-Simulator** permet de regarder une mission se dérouler, avec une interface graphique.

**Robotik-Headless** exécute exactement la même logique, mais sans fenêtre. Idéal pour l'intégration continue et les campagnes de tests.

Les deux autres illustrent des usages plus ciblés : le suivi de ligne et le RL qui sont aussi présents dans Robotik-Simulator.

---

# 3. Pick and Place

C'est la démo de référence : celle à regarder en premier pour comprendre l'architecture de bout en bout.

Le but : amener le cube rouge dans la boîte.

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
data/scenarios/pick_and_place.yml
```

Il décrit la mission « qui se passe bien », sans aucune panne.

Pour la lancer avec l'interface :

```bash
./build/Robotik-Simulator \
    data/scenarios/pick_and_place.yml
```

Et le même scénario en headless, avec une graine reproductible :

```bash
./build/Robotik-Headless \
    data/scenarios/pick_and_place.yml \
    --seed 11
```

---

# 5. Le scénario avec pannes

Le fichier :

```text
data/scenarios/pick_and_place_faults.yml
```

Cette variante ajoute notamment une panne de caméra.

Dans **Robotik-Simulator**, choisir la mission **Pick-and-place (faults)** dans la barre de menu, ou lancer :

```bash
./build/Robotik-Simulator data/scenarios/pick_and_place_faults.yml
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

La mission peut exiger, entre autres :

```text
robot.success
object("red_cube").inside("box")
gripper.empty
collisions == 0
time < 30
```

C'est bien plus pertinent que de tester des angles articulaires. Un autre contrôleur peut très bien suivre une trajectoire différente et pourtant remplir la mission.

---

# 7. Le line follower

Cette démo aborde une autre famille de problèmes : un robot mobile, de la perception et une boucle de contrôle.

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
    data/scenarios/line_follower.yml \
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

On peut ainsi vérifier que les résultats restent identiques d'une exécution à l'autre, même avec de nombreux environnements en parallèle.

---

# 12. Fly brain

This demo puts an agent on the same loop as the RL one: observation, action, environment, seed, headless execution, and several worlds in parallel.

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

The scenario is `data/scenarios/fly_obstacle_avoidance.yml`: reach the food while avoiding the obstacles.

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
