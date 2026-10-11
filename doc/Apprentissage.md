# Apprentissage par renforcement

Robotik n’entraîne pas de politique : il fournit des **environnements** stables, sans fenêtre, reproductibles, qu’un algorithme d’apprentissage (en C++ ou dans un autre processus) peut piloter. Les mêmes mondes et scénarios que les démos servent de base.

---

# 1. Le contrat d’un environnement

`robotik::Environment` (`include/Robotik/Environment/Environment.hpp`) :

| Opération | Ce qu’elle fait |
|-----------|-----------------|
| `reset(seed, observation)` | Démarre un épisode reproductible et écrit la première observation |
| `step(action, observation)` | Applique l’action pendant une durée définie, avance la simulation, écrit l’observation suivante, rend la récompense et la fin éventuelle |
| `observationSpace()`, `actionSpace()` | Disposition des vecteurs : nom, bornes, unité de chaque dimension |

Observations et actions sont des vecteurs plats de `float`, écrits sur place : un pas n’alloue rien.

## Les espaces

```cpp
robotik::Space observation;
observation.add3("tip", -1.5f, 1.5f, "m")        // tip.x, tip.y, tip.z
           .add("holding", 0.0f, 1.0f)
           .add("delivered", 0.0f, 1.0f, "", true); // privileged
```

Une dimension **privilégiée** vient de la vérité terrain du simulateur : un robot réel ne pourrait pas l’observer. Une politique qui s’en sert ne doit pas être prise pour une politique déployable. `Space::privileged()` le dit, et le journal l’enregistre.

## La fin d’un épisode

`StepResult::end` distingue quatre issues, parce qu’elles ne se traitent pas pareil à l’apprentissage ni à l’évaluation :

| `EpisodeEnd` | Sens | État terminal ? |
|--------------|------|-----------------|
| `Success` | La tâche est faite | oui |
| `Failure` | La tâche ne peut plus être faite | oui |
| `TimeLimit` | Le budget de pas est épuisé | non : la valeur de l’état suivant compte encore |
| `Aborted` | Arrêt technique (solveur, état invalide) | non : l’épisode ne dit rien de la politique |

`terminated()` regroupe les deux premières, `truncated()` les deux dernières.

---

# 2. Un environnement sur un scénario

`robotik::SimulationEnvironment` (`SimulationEnvironment.hpp`) fait d’un scénario un environnement. Il s’assemble de trois pièces interchangeables :

```text
           action                        observation
              │                               ▲
              ▼                               │
      ActionAdapter                   ObservationBuilder
   (vecteur → commandes)          (capteurs, croyances → vecteur)
              │                               ▲
              ▼                               │
        Simulation  ── tenue pendant control_period ──┘
              │
              ▼
       RewardEvaluator  ── récompense, Success / Failure
```

| Pièce | Rôle |
|-------|------|
| `ObservationBuilder` | Ce que l’agent a le droit de savoir. Préférer les capteurs et le `WorldModel` ; marquer ce qui vient de la vérité terrain. |
| `ActionAdapter` | Transforme le vecteur en commandes (cibles articulaires, ventouse…). N’avance pas le temps. |
| `RewardEvaluator` | Note le pas à partir de la simulation seule, jamais de l’affichage. Dit quand la tâche est finie. |

Chaque pièce a `setup` (une fois, après la construction) et `reset` (à chaque épisode). Changer la politique ne touche pas au moteur physique ; changer l’observation (proprioception seule, ou caméra) ne touche ni à l’action ni à la récompense.

```cpp
robotik::SimulationEnvironment::Parts parts;
parts.observation = std::make_unique<PickPlaceObservation>();
parts.action = std::make_unique<PickPlaceAction>();
parts.reward = std::make_unique<PickPlaceReward>();

robotik::EnvironmentTiming timing;
timing.physics_step = Seconds(0.01);
timing.control_period = Seconds(0.1);   // une action tenue 10 pas physiques
timing.max_steps = 100;                 // puis TimeLimit

robotik::SimulationEnvironment environment(
    robotik::Scenario::load(path, assets), std::move(parts), timing);
```

Le scénario garde son monde, ses pannes et sa politique de perception. Son contrôleur, son `timeout`, son `stop_when` et ses assertions sont retirés : l’agent pilote, la récompense juge. Une exception levée par une pièce ou par la simulation termine l’épisode en `Aborted`, la raison est dans `abortReason()`.

`reset(seed)` reconstruit l’état initial complet : articulations, base, capteurs (flux aléatoires compris), ressources, pannes, disposition des objets.

---

# 3. Plusieurs environnements

`robotik::EnvironmentPool` fait avancer N environnements sur des threads, derrière des tampons contigus prêts pour un apprenant. L’épisode `k` de l’environnement `i` est semé avec `master.derive(i).derive(k)` : n’importe quel épisode se rejoue seul, et le résultat ne dépend pas du nombre de threads.

```bash
./build/Robotik-PickAndPlaceRL --policy converged --envs 16 --threads 8 --episodes 32 \
    --log episodes.json
```

La commande rejoue ensuite le même lot sur un seul thread et vérifie que tout est identique au bit près.

## Le journal

`robotik::EpisodeLog` écrit en JSON ce qu’il faut pour comparer et rejouer : version et commit de Robotik, scénario, politique, graine maîtresse, période de contrôle, budget de pas, les deux espaces (dimensions privilégiées comprises), puis chaque épisode : environnement, indice, graine, retour, nombre de pas, issue.

---

# 4. Environnements et agents

Ce sont deux notions différentes, qu’il ne faut pas confondre :

| | Ce que c’est | À quoi ça sert |
|--|--------------|----------------|
| **Plusieurs environnements** (`env0` … `envN`) | Des simulations indépendantes, chacune son monde et sa graine | Collecter plus vite, comparer des conditions, statistiques |
| **Plusieurs agents** dans un environnement | Des entités qui agissent dans le même monde et interagissent | Coordination, comportements collectifs |

Les deux se combinent : plusieurs environnements, chacun avec un essaim. Un agent n’est pas forcément appris : un modèle neuronal existant (le cerveau de mouche), un comportement classique ou une politique entraînée partagent la même surface, `robotik::Agent` (`include/Robotik/Agents/Agent.hpp`), qui reçoit une observation et rend une action.

Aujourd’hui, `SimulationEnvironment` pilote un seul robot. La mouche est décrite par un fichier monde et tourne comme plugin ordinaire (simulateur et headless) ; son exécutable garde son propre environnement (`FlyEnvironment`) pour le pool. Plusieurs mouches dans un monde partagé restent à faire.

---

# 5. La liste de vérification

| Question | Réponse dans Robotik |
|----------|----------------------|
| Un reset remet-il tous les états à leur valeur initiale ? | Oui : `Simulation::reset` (robot, capteurs, ressources, pannes, objets), puis `reset` de chaque pièce. Vérifié par un test de rejeu. |
| Espaces d’observation et d’action explicites ? | `Space` : nom, bornes, unité par dimension. |
| Observations cohérentes avec les capteurs disponibles ? | Le pick-and-place lit les croyances du robot ; la seule dimension issue de la vérité terrain est marquée privilégiée. |
| Durée d’application d’une action définie ? | `EnvironmentTiming::control_period`, multiple entier du pas physique. |
| Récompense indépendante de l’affichage ? | `RewardEvaluator` ne voit que la simulation ; aucun environnement n’a de fenêtre. |
| Fin naturelle distinguée d’une troncature et d’un arrêt technique ? | `EpisodeEnd` : `Success`, `Failure`, `TimeLimit`, `Aborted`. |
| Expériences sans GUI ? | Oui : pool, headless, tests. |
| Comparer plusieurs graines et politiques ? | Graines dérivées par environnement et par épisode ; `EpisodeLog` en JSON. |

## Ce qui reste

- Une passerelle stable vers un processus Python (API C au-dessus du pool, ou bindings).
- Plusieurs agents dans un même environnement.
