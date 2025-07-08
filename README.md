# Robotik

Une bibliothèque robotique moderne en C++ pour la simulation et la visualisation de bras robotiques.

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

## Exemples

### 1. Bras robotique 6DOF (Console)
```bash
cd demo/Arm6DOF
make
./Arm6DOF
```

### 2. Visualiseur OpenGL 3D (Nouveau!)
```bash
cd demo/RobotViewer
make install-deps  # Installer les dépendances OpenGL
make               # Compiler
./RobotViewerDemo  # Exécuter
```

**Contrôles du visualiseur :**
- Souris : Rotation de la caméra
- Molette : Zoom
- W/S : Rapprocher/éloigner
- ESC : Quitter

## Structure du projet

```
Robotik/
├── include/Robotik/
│   ├── Robotik.hpp      # API principale
│   └── RobotViewer.hpp  # Visualiseur OpenGL
├── src/
│   ├── Robotik.cpp      # Implémentation robotique
│   └── RobotViewer.cpp  # Implémentation OpenGL
└── demo/
    ├── Arm6DOF/         # Exemple console
    └── RobotViewer/     # Exemple OpenGL
```
