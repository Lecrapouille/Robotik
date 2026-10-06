# Démos Robotik

Les démos vivent sous `src/Applications/Demos/`. Le **Simulateur** les affiche ; les binaires CLI servent au CI et aux mesures sans fenêtre. OpenCV et AprilTag restent dans les démos, jamais dans `librobotik-core`.

Une mission = un YAML (`data/scenarios/`) + un plugin C++ `Mission` (skills, détecteurs, métriques).

## Compiler

Depuis la racine du dépôt :

```bash
make -C src/Robotik/Core -j8
make -C src/Applications/Simulator -j8
make -C src/Applications/Headless -j8
make -C src/Applications/Demos/LineFollower -j8
make -C src/Applications/Demos/PickAndPlaceRL -j8
```

Les binaires sortent dans `build/` : `Robotik-Simulator`, `Robotik-Headless`, `Robotik-LineFollower`, `Robotik-PickAndPlaceRL`.

Lancer les commandes depuis la racine pour que les chemins `data/...` se résolvent.

---

## Pick-and-place

**But.** Un bras 6 axes (`data/robot_6axis.urdf`) saisit le cube rouge et le pose dans la boîte. Perception couleur (démo), ventouse simulée, behavior tree.

**Scénarios.**

| Fichier | Rôle |
|---------|------|
| `data/scenarios/pick_and_place.yml` | Mission nominale |
| `data/scenarios/pick_and_place_faults.yml` | Caméra coupée puis rétablie, pannes aléatoires |

**Assertions.** `robot.success`, `object("red_cube").inside("box")`, `gripper.empty`, `collisions == 0`, `time < 30`.

**Simulateur.** Menu *Pick-and-place*. Onglets Scenario (vérité / croyance), **Scenario file** (YAML), Robot, World, Robot camera, Behavior tree, Skills, Resources.

```bash
./build/Robotik-Simulator
./build/Robotik-Simulator data/scenarios/pick_and_place_faults.yml
```

**CLI (CI, oracle de perception si pas d’image).**

```bash
./build/Robotik-Headless data/scenarios/pick_and_place.yml --seed 11
./build/Robotik-Headless data/scenarios/pick_and_place_faults.yml
```

`--seed N` rejoue le même placement, le même bruit et les mêmes pannes.

---

## Line follower

**But.** Robot différentiel (`data/simple_diff_drive_robot.urdf`) : se localiser sur des AprilTags au sol, rejoindre la ligne, en faire au moins un tour. L’odométrie est volontairement biaisée ; les tags corrigent la pose. La perception (AprilTag + ligne) tourne sur une caméra CPU (`FloorCamera`). Le GPU ne sert qu’à l’opérateur (`SceneView::ground()`).

**Scénario.** `data/scenarios/line_follower.yml`  
**Assertions.** `robot.success`, `time < 150`, `cross_track.max < 0.08`, `laps >= 1`.

**Simulateur.** Menu *Line follower*. L’onglet Scenario montre les fixes, le cross-track et les tours.

**CLI (CI, carte vérité / estimée).**

```bash
./build/Robotik-LineFollower --seed 4 --laps 1 --save map.png
./build/Robotik-LineFollower --view          # fenêtres OpenCV
./build/Robotik-Headless data/scenarios/line_follower.yml --seed 4
```

| Option | Défaut | Rôle |
|--------|--------|------|
| `--seed N` | 1 | Départ, bruit odométrie, tags |
| `--laps X` | 1 | Distance à parcourir (tours de piste) |
| `--view` | off | Caméra + carte en direct |
| `--save map.png` | — | Écrit la carte (vert = vérité, rouge = estimée) |
| `--scenario path` | `data/scenarios/line_follower.yml` | YAML |

---

## Pick-and-place RL

Cette démo montre la **boucle** qu’utiliserait un vrai apprentissage par renforcement. Elle n’entraîne **aucun réseau**. Il n’y a pas de fichier de poids, pas de PPO, pas de SAC.

### La boucle, à chaque pas

```
observation  →  politique  →  action  →  physique  →  récompense
     ↑                                                    │
     └──────────── nouvel état (ou nouvel épisode) ───────┘
```

- **Observation** (16 nombres) : position de la ventouse, du cube, de la boîte, « est-ce que je tiens le cube ? », etc.
- **Action** (4 nombres) : déplacer la ventouse en x, y, z, et allumer / éteindre l’aspiration.
- **Récompense** : se rapprocher du cube ; +2 quand on saisit ; +10 quand le cube est dans la boîte. Le *return* d’un épisode est la somme de ces récompenses.
- **Épisode** : on part d’une pose de départ, on joue jusqu’à succès (cube dans la boîte) ou jusqu’à *max steps* (échec).

Deux programmes tournent **la même** boucle :

| Où | Ce que tu vois |
|----|----------------|
| Simulateur → menu **Pick-and-place RL**, onglet **RL** | Un seul bras, à l’écran |
| `./build/Robotik-PickAndPlaceRL` | N bras en parallèle, sans fenêtre, un tableau de scores |

`--envs 4` ce n’est pas « 4 robots qui s’entraînent et on garde le meilleur ». C’est 4 **copies identiques** de la même politique, pour aller plus vite et vérifier que le rejeu est bit-à-bit.

Ce n’est **pas** le pick-and-place à arbre de comportement (menu *Pick-and-place*, ou `Robotik-Headless`).

### Qui décide l’action ? (`--policy`)

| `--policy` | Qui calcule les 4 nombres | Résultat typique |
|------------|---------------------------|------------------|
| `converged` | Une **recette écrite en C++** (`convergedPolicy`) : va au-dessus du cube, descends, aspire, porte, lâche dans la boîte | Succès en ~50–60 pas, return ~41 |
| `random` | Un tirage au hasard dans `[-1, 1]` | Le bras gigote, échec, return négatif |

`converged` veut dire « déjà capable de finir la tâche », pas « un réseau qui a fini d’apprendre ». C’est le comportement qu’on aurait **après** un vrai entraînement.

### À quoi sert `--train` ?

`--train` **n’apprend rien**. Il ne met à jour aucun poids.

Il fait un **fondu** entre le hasard et la recette, pour qu’on voie le taux de succès monter comme pendant un entraînement.

L’action jouée est :

```
action = (1 − mix) × hasard  +  mix × recette
```

| Moment | `mix` | Ce que fait le bras |
|--------|-------|---------------------|
| Début | 0 | 100 % hasard, ça échoue |
| Après 4 épisodes finis | 0.5 | Moitié hasard, moitié recette |
| Après 8 épisodes finis | 1 | 100 % recette : même chose que `--policy converged` |

Huit, c’est `PICK_PLACE_TRAIN_EPISODES` dans le code.

**`--train` ne sert qu’avec `--policy random`.** Avec `--policy converged`, `mix` vaut déjà 1 : le flag ne change rien.

Sans `--train`, `--policy random` reste du hasard du premier au dernier épisode (baseline : « à quoi ressemble un agent qui ne sait rien »).

Exemple de sortie avec `--policy random --train --episodes 12` : les premiers épisodes `success no`, les derniers `success yes`. Le résumé `random→convergé` compte les succès sur **tout** le run, donc pas 12/12 : le début a échoué exprès.

Dans le Simulateur, **Random** fait toujours ce fondu (pas de bruit pur). La barre monte à chaque pas d’action (~400 pas pour arriver à 100 %). **Converged** = `--policy converged`. Le bruit pur sans fondu n’existe que sur le CLI : `--policy random` sans `--train`.

### Commandes CLI

Toujours depuis la racine du dépôt.

```bash
# Recette seule (déjà capable). Mesure le débit du pool.
./build/Robotik-PickAndPlaceRL --policy converged --envs 16 --threads 8 --episodes 32

# Hasard qui fond vers la recette. Les derniers épisodes réussissent.
./build/Robotik-PickAndPlaceRL --policy random --train --envs 4 --episodes 16 --seed 1

# Hasard seul, ça n’arrive jamais (baseline).
./build/Robotik-PickAndPlaceRL --policy random --envs 4 --episodes 8 --seed 1
```

| Option | Défaut | Rôle |
|--------|--------|------|
| `--policy converged\|random` | `converged` | Recette, ou hasard |
| `--train` | off | Si `random` : monter `mix` de 0 à 1 en 8 épisodes. Inutile si `converged`. |
| `--envs N` | 8 | Nombre de copies parallèles (même politique) |
| `--threads T` | 0 = auto | Threads du pool |
| `--episodes E` | 32 | Combien d’épisodes afficher dans le tableau |
| `--seed S` | 1 | Graine maître (même seed = même run) |
| `--scenario file.yml` | pick-and-place | Scène (l’arbre de comportement est coupé) |
| `--no-replay` | — | Ne pas revérifier le rejeu sur 1 thread |
| `--trace` | — | Afficher observation et action de la copie 0 |

Le pool doit donner le **même** tableau quel que soit `--threads`. Pour brancher un vrai trainer, on remplace `pickPlacePolicy` par le `forward()` du réseau.

---

## Tutoriel rapide

1. Compiler le cœur et le Simulateur (ci-dessus).
2. `./build/Robotik-Simulator` — la mission pick-and-place démarre.
3. Play / Pause / Step, *Replay* (même seed), *New seed*.
4. **EMERGENCY STOP** préempte le bras (`Stop`, priorité 1000).
5. Combo *Pick-and-place RL*, onglet **RL** : **Converged** = la recette (le cube arrive dans la boîte). **Random** = fondu hasard → recette (équivalent de `--train`). Le pool N copies est le CLI, pas la fenêtre.
6. Onglet **Scenario file** : le YAML chargé. *Load* relit le chemin de la barre.
7. Vérifier sans fenêtre : `./build/Robotik-Headless data/scenarios/pick_and_place.yml --seed 11` (codes : 0 = assertions OK, 2 = échec, 1 = erreur).
8. CI line follower : `./build/Robotik-LineFollower --seed 4 --save map.png`.
9. Mesure RL : `./build/Robotik-PickAndPlaceRL --policy converged --envs 8 --episodes 16`. Pour voir le fondu hasard → recette : `--policy random --train` (ce n’est pas un vrai entraînement).

Pour ajouter une mission : copier un YAML, implémenter `robotik::Mission` (`setup` / `reset` / `step` / `measure`), l’enregistrer dans le menu du Simulateur et dans `Robotik-Headless`.
