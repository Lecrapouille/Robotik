# Robotik Viewer Demo

Ce démonstrateur montre comment utiliser le viewer OpenGL de la bibliothèque Robotik pour visualiser un robot arm en 3D.

## Fonctionnalités

Le viewer OpenGL offre les fonctionnalités suivantes :

- **Rendu 3D simple** : Affichage de géométries basiques (boîtes, cylindres, sphères)
- **Grille de référence** : Grille au sol pour faciliter l'orientation
- **Vues caméra prédéfinies** : Perspective, vue de dessus, vue de face, vue de côté, vue isométrique
- **Visualisation de robot** : Affichage automatique de la structure du robot arm
- **Pas d'éclairage** : Rendu en couleurs plates pour la simplicité

## Dépendances

### Ubuntu/Debian
```bash
sudo apt-get install libglfw3-dev libglew-dev libgl1-mesa-dev
```

### Fedora
```bash
sudo dnf install glfw-devel glew-devel mesa-libGL-devel
```

### Arch Linux
```bash
sudo pacman -S glfw-x11 glew mesa
```

## Compilation

1. Compiler la bibliothèque Robotik principale :
```bash
cd ../..
make
```

2. Compiler la démonstration :
```bash
cd demos/ViewerDemo
make
```

## Utilisation

### Lancer la démonstration
```bash
make run
```

### Contrôles
- **Fermer la fenêtre** : Cliquer sur le bouton de fermeture
- **Changer de vue** : Utiliser les méthodes de la classe Viewer

## Code d'exemple

```cpp
#include "Robotik/Robotik.hpp"
#include "Robotik/Viewer.hpp"

int main()
{
    // Créer un robot arm
    robotik::Robot robot("MonRobot");

    // Configurer la structure du robot...

    // Créer le viewer
    robotik::Viewer viewer(1024, 768, "Mon Viewer");
    viewer.initialize();
    viewer.setRobot(&robot);

    // Boucle de rendu
    while (!viewer.shouldClose())
    {
        viewer.processInput();
        viewer.render();
    }

    return 0;
}
```

## Vues caméra disponibles

- `CameraView::PERSPECTIVE` : Vue perspective par défaut
- `CameraView::TOP` : Vue de dessus
- `CameraView::FRONT` : Vue de face
- `CameraView::SIDE` : Vue de côté
- `CameraView::ISOMETRIC` : Vue isométrique

## Structure du code

- `Viewer.hpp` : Déclaration de la classe Viewer
- `Viewer.cpp` : Implémentation du viewer OpenGL
- `ViewerDemo.cpp` : Exemple d'utilisation

## Limitations

- Pas de gestion de l'éclairage (rendu en couleurs plates)
- Géométries simplifiées (cylindres et sphères approximés par des boîtes)
- Pas de contrôles interactifs de la caméra
- Pas de gestion des textures

## Extensions possibles

- Ajout de contrôles de caméra interactifs
- Implémentation d'éclairage
- Support de géométries plus complexes
- Ajout de textures
- Gestion des collisions visuelles