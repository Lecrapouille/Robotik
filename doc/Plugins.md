# Les plugins Robotik

Un plugin est une démonstration livrée à part du simulateur. Le C++ vit dans une bibliothèque partagée. Les fichiers YAML, à côté, décrivent chacune des exécutions de ce même code.

Le simulateur et l’exécuteur sans fenêtre chargent le même paquet. Ajouter une démo consiste à ajouter un dossier dans `demos/`, puis à recompiler.

---

# 1. Pourquoi des plugins

Les démos partagent la fenêtre, le temps, les menus et le runtime. Leur logique, elle, change : perception, politique, métriques, panneaux.

Les compiler dans le simulateur oblige à modifier l’hôte à chaque nouvelle idée. Le plugin inverse cette dépendance. L’hôte découvre les paquets, charge la bibliothèque au moment où un scénario est choisi, et appelle le code de la démo aux instants prévus.

Un même plugin possède plusieurs scénarios. Le pick-and-place nominal et celui avec pannes partagent `PickPlaceMission`. La cinématique directe, la cinématique inverse et les limites articulaires partagent `ManipulatorMission`. Le code n’est pas dupliqué, et les fichiers restent ensemble, donc faciles à trouver et à distribuer.

```text
demos/Manipulator/
├── plugin.yaml
├── Plugin.cpp
├── ManipulatorMission.cpp
└── scenarios/
    ├── forward_kinematics.yml
    ├── inverse_kinematics.yml
    └── joint_limits.yml
```

Le YAML décrit le monde, le robot, la tâche et les assertions. Le C++ apporte ce que le YAML ne sait pas exprimer.

---

# 2. Les deux hôtes

| Hôte | Dossier | Rôle |
|------|---------|------|
| `Robotik-Simulator` | `simulator/` | Fenêtre, menu **Demos**, panneaux, temps |
| `Robotik-Headless` | `headless/` | Le même scénario, sans rendu |

Les deux parcourent, dans cet ordre :

1. le chemin passé à `scan`, s’il y en a un ;
2. la variable `ROBOTIK_PLUGINS` ;
3. `build/plugins` ;
4. `plugins` ;
5. `../build/plugins`.

Le catalogue lit `plugin.yaml` sans ouvrir la bibliothèque. La bibliothèque n’est chargée que pour le scénario choisi. Passer d’un scénario à un autre du même plugin garde la bibliothèque ouverte.

Après `make`, le paquet installé est :

```text
build/plugins/Manipulator/
├── plugin.yaml
├── librobotik_manipulator.so
└── scenarios/
    ├── forward_kinematics.yml
    ├── inverse_kinematics.yml
    └── joint_limits.yml
```

Lancer :

```bash
./build/Robotik-Simulator
./build/Robotik-Headless \
    build/plugins/Manipulator/scenarios/forward_kinematics.yml
```

Le menu **Demos** reprend le champ `name` du paquet, puis le `name` de chaque scénario.

---

# 3. Le paquet

`plugin.yaml` est le seul catalogue du plugin.

```yaml
id: robotik.hello
name: Hello
version: 1.0.0
library: librobotik_hello.so
category: Demo
graphics: false
scenarios:
  - id: hello
    name: Hello
    file: scenarios/hello.yml
```

| Champ | Rôle |
|-------|------|
| `id` | Identifiant stable. Il doit être celui renvoyé par `describe()`. |
| `name` | Libellé du sous-menu **Demos**. |
| `library` | Nom de fichier se terminant par `.so`, sans répertoire et sans `..`. |
| `scenarios` | Liste explicite. Chaque `file` reste dans le paquet. |
| `graphics` | `true` réserve la démo au simulateur. Headless la refuse. |
| `clock: plugin` | `update` fait avancer la simulation. L’hôte n’appelle plus `Simulation::step`. |
| `view` | Caméra optionnelle : `eye` et `target`, trois nombres chacun. |

Si `scenarios` est omis, le catalogue prend `scenarios/*.{yml,yaml}`.

Le scénario lui-même est un YAML Robotik ordinaire. Un nom de modèle sans slash, par exemple `robot_6axis.urdf`, est cherché en remontant jusqu’à `data/`. Le format est décrit dans [Scenario-et-Simulation.md](Scenario-et-Simulation.md).

---

# 4. Comment l’hôte appelle le plugin

La frontière binaire est en C, dans `include/Robotik/Plugin/PluginABI.h`. Aucune classe C++, aucun conteneur et aucune exception ne la traverse. Le développeur écrit du C++ : `include/Robotik/Plugin/PluginAPI.hpp` fournit la classe `robotik::Plugin`, l’objet `Host` et la macro `ROBOTIK_EXPORT_PLUGIN`.

`describe()` est statique. L’hôte l’appelle avant de créer l’instance, pour comparer l’`id` à celui du manifeste.

```text
catalogue (plugin.yaml)
        │
        ▼
   dlopen de la .so          seulement pour le scénario choisi
        │
        ▼
   setup(id du scénario)    crée la Mission et l’enregistre
        │
        ▼
   Simulation               reçoit cette Mission
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

Une assertion en échec reste affichée en rouge et peut repasser au vert. Elle ne termine pas la simulation. La fin vient du statut de la mission ou de l’arbre de comportement.

Événements disponibles : touche (`H` dans le simulateur), scénario démarré ou arrêté, skill réussi ou échoué, assertion dont le résultat change.

Le panneau est dessiné par l’hôte. Le plugin appelle `Canvas` : texte, bouton, case à cocher, curseur entier. Ces appels ne sont valables que pendant le dessin.

La mouche déclare `graphics: true`. Sa vue 3D est compilée dans le simulateur, parce que Compages est une archive statique. Le menu la liste quand même. Headless refuse ce paquet.

---

# 5. Micro tutoriel

Le plugin **Hello** publie un compteur. Le scénario vérifie qu’il a bougé, puis la mission s’arrête au bout de deux secondes.

Arborescence :

```text
demos/Hello/
├── Makefile
├── Plugin.cpp
├── plugin.yaml
└── scenarios/
    └── hello.yml
```

`plugin.yaml` :

```yaml
id: robotik.hello
name: Hello
version: 1.0.0
library: librobotik_hello.so
category: Demo
graphics: false
scenarios:
  - id: hello
    name: Hello
    file: scenarios/hello.yml
```

`scenarios/hello.yml` :

```yaml
scenario: hello
seed: 1

robot:
  model: robot_6axis.urdf
  actuators:
    arm: { type: joint_group }

execute:
  task: Count simulation steps from the plugin.

assert:
  - hello.ticks > 0
  - time < 30
```

`Plugin.cpp` :

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
        return { "robotik.hello", "Hello", "1.0.0", "Demo" };
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

`Makefile`, sur le modèle de `demos/plugin.mk` :

```make
P := ../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := robotik_hello
TARGET_DESCRIPTION := Hello plugin
PLUGIN_NAME := Hello
PLUGIN_DIR := $(P)/demos/Hello
include $(M)/project/Makefile

LIB_FILES := $(PLUGIN_DIR)/Plugin.cpp

include $(PLUGIN_DIR)/../plugin.mk
```

`TARGET_NAME` produit `librobotik_hello.so`. Ce nom est celui du champ `library`. Le `make` à la racine compile chaque `demos/*/Makefile` et recopie le manifeste, la bibliothèque et les scénarios dans `build/plugins/Hello/`.

```bash
make -C demos/Hello -j"$(nproc)"
./build/Robotik-Headless build/plugins/Hello/scenarios/hello.yml
```

La sortie attendue contient `[PASS] hello.ticks > 0` et un temps d’environ 2 s. Dans le simulateur, le scénario apparaît sous **Demos → Hello**.

Un second fichier dans `scenarios/`, déclaré dans `plugin.yaml`, réutilise la même bibliothèque. `setup` reçoit l’`id` du scénario et choisit la mission.

---

# 6. Les paquets du dépôt

| Paquet | Scénarios | Rôle |
|--------|-----------|------|
| `PickAndPlaceBT` | nominal, pannes | Mission à arbre de comportement |
| `PickAndPlaceRL` | politique | `clock: plugin`, réutilise `PickPlaceControl.cpp` du paquet BT |
| `LineFollower` | suivi de ligne | Mission, vision, panneau |
| `Manipulator` | directe, inverse, limites | Une mission, trois YAML |
| `FlyBrain` | évitement | `graphics: true`, vue dans le simulateur |
| `Probe` | alpha, beta | Essai du cycle de vie pour les tests |

Line follower, le pool RL et la mouche ont aussi un exécutable dans `demos/<Nom>/app` pour les usages hors du couple simulateur / headless. Le détail de ces démos est dans [Demos.md](Demos.md).
