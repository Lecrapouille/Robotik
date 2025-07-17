# RobotIK

Bibliothèque robotique en C++ pour la manipulation et la visualisation de bras robotiques.

## Fonctionnalités

- **Cinématique directe et inverse** : Calcul des positions et orientations
- **Graphe de scène** : Représentation hiérarchique des robots
- **Jacobien** : Calcul pour le contrôle en vitesse
- **Visualisation OpenGL** : Rendu 3D temps réel avec contrôles interactifs
- **Support multi-joints** : Joints rotatifs, prismatiques et fixes

## Prérequis

### Installation de base

```bash
sudo apt-get install libeigen3-dev
```

### Pour le visualiseur OpenGL

```bash
sudo apt-get install libgl1-mesa-dev libglew-dev libglfw3-dev
```

## Compilation

```bash
git clone https://github.com/Lecrapouille/Robotik --recurse
cd Robotik
make -j8
# Optional:
make tests -j8
sudo make install
```

Un dossier `build` a du être créé, il contient les libraries créées ainsi que les demos.

## Visualiseur

Permet de charger un nouveau robot depuis un fichier URDF, de le visualiser et de le contrôler.

```bash
./build/viewer-demo
```

**Contrôles du visualiseur :** (en cours)

- Souris : Rotation de la caméra
- Molette : Zoom
- W/S : Rapprocher/éloigner
- ESC : Quitter

## Structure du projet

```
Robotik/
├── include/Robotik/
│   ├── Robotik.hpp      # API principale du robot
│   ├── Parser.hpp       # Classe créant un robot via un fichier URDF
│   └── Viewer.hpp       # Visualiseur de robot en OpenGL
├── src/
│   ├── Robotik.cpp      # Implémentation robotique
│   ├── Parser.cpp       # Implémentation du parseur URDF
│   └── Viewer.cpp       # Implémentation OpenGL
└── demo/
    └── RobotViewer/     # Exemple OpenGL
```
