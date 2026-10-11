# Les plugins Robotik

Un plugin est une démonstration livrée à part du simulateur. Le C++ vit dans une bibliothèque partagée. Les fichiers YAML, à côté, décrivent le monde, les scénarios et les expériences qui utilisent ce code.

Le simulateur et l’exécuteur sans fenêtre chargent le même paquet. Ajouter une démo consiste à ajouter un dossier dans `plugins/`, puis à recompiler.

---

# 1. Pourquoi des plugins

Les démos partagent la fenêtre, le temps, les menus et le runtime. Leur logique, elle, change : perception, gestes, politique, métriques, panneaux.

Les compiler dans le simulateur oblige à modifier l’hôte à chaque nouvelle idée. Le plugin inverse cette dépendance. L’hôte découvre les paquets, charge la bibliothèque au moment où un scénario est choisi, et appelle le code de la démo aux instants prévus.

Un même plugin possède plusieurs scénarios. Le pick-and-place nominal et celui avec pannes partagent `PickPlaceMission`. La cinématique directe, la cinématique inverse et les limites articulaires partagent `ManipulatorMission`. Le code n’est pas dupliqué, et les fichiers restent ensemble, donc faciles à trouver et à distribuer.

```text
plugins/Painter/
├── plugin.yaml              manifeste du paquet
├── Makefile
├── src/                     le C++ de la bibliothèque
├── data/                    pack d’assets propre au paquet (le chevalet)
├── worlds/
│   └── painting_lab.yaml    ce qui existe physiquement
├── scenarios/
│   ├── painter.yml          ce qui se passe, et comment le juger
│   ├── painter.bt.yml       l’arbre de comportement
│   └── trajectories/        les traits, un fichier par couleur
└── experiments/
    └── painting_benchmark.yaml   cas, graines, évaluation
```

Le YAML décrit le monde, le scénario et l’expérience ([Scenario-et-Simulation.md](Scenario-et-Simulation.md)). Le C++ apporte ce que le YAML ne sait pas exprimer : les gestes, les détecteurs, les métriques propres à la démo.

---

# 2. Le plugin ne possède pas le monde

Le plugin apporte du code et des fichiers. Les entités créées pendant une simulation appartiennent au monde courant : la `Simulation` les construit à partir du fichier monde, et les détruit avec lui.

Le Painter en est l’exemple. Le chevalet et le tableau sont des entités de `painting_lab.yaml`. La mission ne les charge pas : elle les retrouve par leur identifiant (`p_simulation.entity("canvas")`, `p_simulation.pose("canvas")`) et n’apporte que les gestes. Un reset reconstruit l’état du monde sans décharger le plugin, et aucun pointeur vers un objet du monde ne survit dans le plugin.

---

# 3. Les deux hôtes

| Hôte | Dossier | Rôle |
|------|---------|------|
| `Robotik-Simulator` | `simulator/` | Fenêtre, menu **Demos**, panneaux, temps |
| `Robotik-Headless` | `headless/` | Un scénario ou une expérience, sans rendu |

Aucune recherche ne dépend du répertoire courant. Les deux hôtes parcourent, dans cet ordre :

1. le chemin passé à `scan`, s’il y en a un ;
2. chaque dossier de la variable `ROBOTIK_PLUGINS` (séparés par `:`) ;
3. `plugins/` à côté de l’exécutable (`build/plugins` dans l’arbre de compilation) ;
4. `lib/robotik/plugins` du préfixe d’installation de l’exécutable.

Un scénario donné par son chemin amène son paquet avec lui, où qu’il soit : `Robotik-Headless plugins/Painter/scenarios/painter.yml` fonctionne depuis les sources, la bibliothèque étant cherchée à côté du manifeste, puis à côté de l’exécutable.

Le catalogue lit `plugin.yaml` sans ouvrir la bibliothèque. La bibliothèque n’est chargée que pour le scénario choisi. Passer d’un scénario à un autre du même plugin garde la bibliothèque ouverte.

Après `make`, le paquet installé est :

```text
build/plugins/Painter/
├── plugin.yaml
├── librobotik_painter.so
├── data/
├── worlds/
├── scenarios/
└── experiments/
```

Lancer :

```bash
./build/Robotik-Simulator
./build/Robotik-Headless build/plugins/Painter/scenarios/painter.yml
./build/Robotik-Headless build/plugins/Painter/experiments/painting_benchmark.yaml \
    --report painting.json
```

Le menu **Demos** reprend le champ `plugin.name` du paquet, puis le `name` de chaque scénario.

---

# 4. Le manifeste

`plugin.yaml` décrit le paquet. Il ne le construit pas : le Makefile reste responsable de la compilation, du téléchargement et du cache des dépendances.

```yaml
plugin:
  id: robotik.painter
  name: Robotic painter
  version: 1.0.0
  api_version: 1
  category: Manipulation

library:
  entrypoint: librobotik_painter.so

resources:
  directory: data
  packs:
    - { id: robotik.robots, version: "1.0" }
    - { id: robotik.tools, version: "1.0" }

dependencies:
  plugins: []
  thirdparties: []

host:
  graphics: false
  clock: simulation
  view: { eye: [1.6, 1.2, 1.6], target: [0.35, 0.15, 0.0] }

scenarios:
  directory: scenarios
  entries:
    - { id: painter, name: Robotic painter, file: painter.yml }

experiments:
  directory: experiments
  entries:
    - { id: painting_benchmark, name: Painting benchmark, file: painting_benchmark.yaml }
```

| Champ | Rôle |
|-------|------|
| `plugin.id` | Identifiant stable. La bibliothèque doit renvoyer le même dans `describe()`. |
| `plugin.version` | `majeur.mineur.correctif`. |
| `plugin.api_version` | Version de l’ABI C pour laquelle la bibliothèque a été compilée. Un paquet d’une autre ABI est refusé avant tout `dlopen`. |
| `library.entrypoint` | Nom de fichier se terminant par `.so`, sans répertoire et sans `..`. |
| `resources.directory` | Pack d’assets propre au paquet (§5). |
| `resources.packs` | Packs nécessaires **à l’exécution**. Absents, ou d’une version incompatible : le scénario est refusé avec la raison. |
| `dependencies.plugins` | Paquets qui doivent être dans le catalogue pour que celui-ci s’exécute. |
| `dependencies.thirdparties` | Bibliothèques nécessaires **à la compilation**, résolues par le Makefile. Informatif. |
| `host.graphics` | `true` réserve la démo au simulateur. Headless la refuse. |
| `host.clock: plugin` | `update` fait avancer la simulation. L’hôte n’appelle plus `Simulation::step`. |
| `host.view` | Caméra du simulateur : `eye` et `target`. |
| `scenarios`, `experiments` | Fichiers exécutables. Chaque `file` reste dans le paquet. Sans `entries`, le catalogue prend tous les `*.yml` et `*.yaml` du dossier, arbres de comportement exceptés. |

Une version requise `"1.2"` accepte 1.2.0 et tout 1.x plus récent, pas 2.0.

---

# 5. Les packs d’assets

Un modèle de robot, d’outil ou d’objet n’est pas copié dans chaque paquet qui s’en sert. Il vit dans un **pack** : un dossier avec un `pack.yaml`, versionné.

```yaml
# data/packs/robotik.tools/pack.yaml
id: robotik.tools
version: 1.0.0
license: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
source: https://github.com/Lecrapouille/RobotIK
models:
  vacuum: tool_vacuum.urdf
  gripper: tool_gripper.urdf
```

Un fichier YAML désigne un modèle par `<id du pack>.<clé>` : `robotik.tools.vacuum`, `robotik.robots.robot_6axis`, `robotik.painter.easel`. Le chemin réel est trouvé par `AssetResolver`, jamais écrit dans un plugin. Le plus long identifiant de pack l’emporte : `robotik.robots.abb.irb2400` désigne le pack `robotik.robots.abb`.

| Niveau | Où | Exemple |
|--------|----|---------|
| Pack partagé | `data/packs/<id>/` | `robotik.robots`, `robotik.tools`, `robotik.robots.abb` |
| Pack d’un plugin | `plugins/<Nom>/data/` | `robotik.painter` (le chevalet) |

Les packs sont cherchés, dans l’ordre : dans `ROBOTIK_DATA`, dans `data/packs` à côté du dossier de l’exécutable, dans `share/robotik/packs` du préfixe d’installation, puis dans le pack de chaque paquet du catalogue. Le premier trouvé l’emporte ; un autre pack de même identifiant et d’une autre version est signalé comme conflit.

`license` est une expression SPDX. `NOASSERTION` signifie que personne n’a encore vérifié la licence des fichiers : c’est le cas des maillages tiers (ABB, T12, robots cartésien et SCARA), à vérifier avant toute redistribution.

---

# 6. Comment l’hôte appelle le plugin

La frontière binaire est en C, dans `include/Robotik/Plugin/PluginABI.h`. Aucune classe C++, aucun conteneur et aucune exception ne la traverse. Le développeur écrit du C++ : `include/Robotik/Plugin/PluginAPI.hpp` fournit la classe `robotik::Plugin`, l’objet `Host` et la macro `ROBOTIK_EXPORT_PLUGIN`.

`describe()` est statique. L’hôte l’appelle avant de créer l’instance, pour comparer l’`id` à celui du manifeste. Le reste de la description vient du manifeste.

```text
catalogue (plugin.yaml)      api_version, packs, dépendances vérifiés
        │
        ▼
   dlopen de la .so          seulement pour le scénario choisi
        │
        ▼
   setup(id du scénario)    crée la Mission et l’enregistre
        │
        ▼
   Simulation               construit le monde, reçoit cette Mission
        │
        ▼
   start
        │
        ▼
   step de la Simulation, puis update(dt)
        │
        ▼
   stop, shutdown           la .so peut rester chargée
```

`setup` a lieu avant que la simulation existe. `bindMission` transmet la mission que l’hôte passera au constructeur de `Simulation`. `update` arrive après chaque pas. Il sert aux comptes, aux événements et, avec `clock: plugin`, à piloter le temps soi-même.

Les méthodes que l’on n’a pas besoin de spécialiser ont un comportement vide et réussissent.

| Méthode | Moment |
|---------|--------|
| `setup` | Préparer la mission, les abonnements, le menu et le panneau |
| `start` | Le scénario démarre |
| `update(dt)` | Après chaque pas, ou à la place du pas si `clock: plugin` |
| `onEvent` | Touche, début ou fin de scénario, skill, assertion |
| `pause` | La barre **Pause** |
| `stop` | Fin de l’exécution, la simulation existe encore |
| `shutdown` | Libérer la mission. L’hôte a déjà détruit la simulation |

Les assertions sont jugées par le moteur du scénario, pas par le plugin : le plugin ne réimplémente ni `is_finished()` ni `assert_success()`. Il publie des métriques (`Mission::measure`) que les assertions peuvent utiliser. Une assertion en échec reste affichée en rouge ; elle n’arrête la simulation que si elle le déclare (`stop: true`).

Événements disponibles : touche (`H` dans le simulateur), scénario démarré ou arrêté, skill réussi ou échoué, assertion dont l’état change.

Le panneau est dessiné par l’hôte. Le plugin appelle `Canvas` : texte, bouton, case à cocher, curseur entier. Ces appels ne sont valables que pendant le dessin.

La mouche est un plugin comme les autres : son modèle de vol est le backend du robot (`FlyBackend`), son cerveau est l’agent que la mission fait tourner. Le monde (`worlds/fly_arena.yaml`) décrit la mouche, ses deux yeux, les blocs et la nourriture ; les faisceaux des yeux et la trace sont des décorations (`ecs::Decoration`), ignorées par la préhension, les capteurs de distance et la vérité terrain. Elle tourne donc aussi en headless.

---

# 7. Micro tutoriel

Le plugin **Hello** publie un compteur. Le scénario vérifie qu’il a bougé, puis la mission s’arrête au bout de deux secondes.

Arborescence :

```text
plugins/Hello/
├── Makefile
├── plugin.yaml
├── src/
│   └── Plugin.cpp
├── worlds/
│   └── arm.yaml
└── scenarios/
    └── hello.yml
```

`plugin.yaml` :

```yaml
plugin:
  id: robotik.hello
  name: Hello
  version: 1.0.0
  api_version: 1
  category: Demo

library:
  entrypoint: librobotik_hello.so

resources:
  packs:
    - { id: robotik.robots, version: "1.0" }

scenarios:
  directory: scenarios
  entries:
    - { id: hello, name: Hello, file: hello.yml }
```

`worlds/arm.yaml` :

```yaml
world:
  id: arm
  entities:
    - id: robot
      type: robot
      model: robotik.robots.robot_6axis
      actuators:
        arm: { type: joint_group }
```

`scenarios/hello.yml` :

```yaml
scenario:
  id: hello
  description: Count simulation steps from the plugin.
  world: ../worlds/arm.yaml
  seed: 1
  timeout: 30
  assertions:
    - hello.ticks > 0
    - time < 30
```

`src/Plugin.cpp` :

```cpp
#include "Robotik/Plugin/PluginAPI.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include <memory>
#include <string>

namespace
{

class HelloMission final: public robotik::Mission
{
public:

    void step(robotik::Simulation& /*p_simulation*/, Seconds /*p_dt*/) override
    {
        m_ticks += 1.0;
    }

    void measure(robotik::Simulation const& /*p_simulation*/,
                 robotik::Metrics& p_metrics) const override
    {
        p_metrics.set("hello.ticks", m_ticks);
    }

    robotik::Status status(robotik::Simulation const& p_simulation) const override
    {
        if (p_simulation.time() >= Seconds(2.0))
        {
            return robotik::Status::SUCCESS;
        }
        return robotik::Status::RUNNING;
    }

    [[nodiscard]] double ticks() const
    {
        return m_ticks;
    }

private:

    double m_ticks = 0.0;
};

class HelloPlugin final: public robotik::Plugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.hello" };
    }

    RobotikPluginStatus setup(robotik::Host& p_host, std::string_view p_scenario) override
    {
        if (p_scenario != "hello")
        {
            p_host.error("unknown hello scenario");
            return ROBOTIK_PLUGIN_ERR_SETUP;
        }
        m_mission = std::make_unique<HelloMission>();
        p_host.bindMission(*m_mission);
        p_host.addPanel("hello", "Hello", &onDraw, this);
        return ROBOTIK_PLUGIN_OK;
    }

    RobotikPluginStatus shutdown(robotik::Host& /*p_host*/) override
    {
        m_mission.reset();
        return ROBOTIK_PLUGIN_OK;
    }

    void draw(robotik::Canvas const& p_canvas) const
    {
        if (m_mission != nullptr)
        {
            p_canvas.text("ticks " + std::to_string(static_cast<int>(m_mission->ticks())));
        }
    }

private:

    static void onDraw(void* p_user, RobotikCanvas const* p_canvas)
    {
        static_cast<HelloPlugin*>(p_user)->draw(robotik::Canvas(p_canvas));
    }

    std::unique_ptr<HelloMission> m_mission;
};

} // namespace

ROBOTIK_EXPORT_PLUGIN(HelloPlugin)
```

`Makefile`, sur le modèle de `plugins/plugin.mk` :

```make
P := ../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := robotik_hello
TARGET_DESCRIPTION := Hello plugin
PLUGIN_NAME := Hello
PLUGIN_DIR := $(P)/plugins/Hello
PLUGIN_SRC := $(PLUGIN_DIR)/src
include $(M)/project/Makefile

LIB_FILES := $(PLUGIN_SRC)/Plugin.cpp

include $(P)/plugins/plugin.mk
```

`TARGET_NAME` produit `librobotik_hello.so`. Ce nom est celui de `library.entrypoint`. Le `make` à la racine compile chaque `plugins/*/Makefile` et recopie le manifeste, la bibliothèque et les dossiers `worlds/`, `scenarios/`, `experiments/` et `data/` dans `build/plugins/Hello/`.

```bash
make -C plugins/Hello -j"$(nproc)"
./build/Robotik-Headless build/plugins/Hello/scenarios/hello.yml
```

La sortie attendue contient `[PASS] hello.ticks > 0` et un temps d’environ 2 s. Dans le simulateur, le scénario apparaît sous **Demos → Hello**.

Un second fichier dans `scenarios/`, déclaré dans `plugin.yaml`, réutilise la même bibliothèque. `setup` reçoit l’`id` du scénario et choisit la mission.

---

# 8. Les paquets du dépôt

| Paquet | Scénarios | Expériences | Rôle |
|--------|-----------|-------------|------|
| `Painter` | portrait | `painting_benchmark` | Monde avec corps libres (chevalet, tableau), trajectoires en fichiers |
| `PickAndPlaceBT` | nominal, pannes | `pick_and_place_benchmark` | Mission à arbre de comportement, outils interchangeables |
| `PickAndPlaceRL` | politique | | `host.clock: plugin`, réutilise `PickPlaceControl.cpp` du paquet BT |
| `LineFollower` | suivi de ligne | | Mission, vision, panneau |
| `Manipulator` | directe, inverse, limites | | Une mission, trois scénarios, un monde |
| `FlyBrain` | évitement | | Le modèle de vol comme backend, le cerveau comme agent |
| `Probe` | alpha, beta | | Essai du cycle de vie pour les tests |

Line follower, le pool RL et la mouche ont aussi un exécutable dans `plugins/<Nom>/app` pour les usages hors du couple simulateur / headless. Le détail de ces démos est dans [Demos.md](Demos.md).
