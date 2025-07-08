# Arm6DOF Demo avec Visualisation OpenGL

Ce démo présente un bras robotique à 6 degrés de liberté avec calculs de cinématique ET visualisation OpenGL interactive.

## Fonctionnalités

### 1. Tests de cinématique (Console)
- **Cinématique directe** : Calcul de la position de l'effecteur final
- **Cinématique inverse** : Résolution pour atteindre une position cible
- **Affichage des transformations** : Matrices, positions et orientations

### 2. Visualisation OpenGL 3D
- **Affichage temps réel** du bras robotique
- **Contrôles interactifs** pour manipuler chaque joint
- **Animation automatique** avec mouvements sinusoïdaux
- **Affichage des repères** de coordonnées

## Compilation

```bash
# Installer les dépendances
sudo apt-get install libgl1-mesa-dev libglew-dev libglfw3-dev libeigen3-dev

# Compiler
make clean && make
```

## Exécution

```bash
# Depuis le répertoire racine du projet
./build/arm6dof-demo
```

## Utilisation

### Phase 1 : Tests de cinématique
Le programme commence par effectuer des tests de cinématique en mode console :
1. Position de repos (tous les joints à 0)
2. Position de test avec angles spécifiques
3. Test de cinématique inverse

### Phase 2 : Visualisation OpenGL
Une fois les tests terminés, le visualiseur OpenGL s'ouvre avec :

#### Contrôles de caméra
- **Souris** : Rotation de la caméra autour du robot
- **Molette** : Zoom avant/arrière
- **W/S** : Rapprocher/éloigner la caméra

#### Contrôles du robot
- **1-6** : Sélectionner le joint à contrôler (1=base, 6=poignet)
- **+/=** : Augmenter l'angle du joint sélectionné
- **-** : Diminuer l'angle du joint sélectionné
- **R** : Remettre à zéro (position repos)
- **A** : Activer/désactiver l'animation automatique
- **F** : Afficher/masquer les repères de coordonnées

#### Contrôles généraux
- **ESC** : Quitter l'application

## Configuration du robot

Le robot 6DOF est configuré avec :
- **6 joints rotatifs** avec limites d'angles
- **Géométrie réaliste** inspirée des robots industriels
- **Hiérarchie de nœuds** pour la cinématique
- **Offsets géométriques** entre les joints

### Dimensions (en mètres)
- Lien 1 : 0.2m (base vers joint 2)
- Lien 2 : 0.3m (joint 2 vers joint 3)
- Lien 3 : 0.3m (joint 3 vers joint 4)
- Lien 4 : 0.1m (joint 4 vers joint 5)
- Lien 5 : 0.1m (joint 5 vers joint 6)
- Lien 6 : 0.1m (joint 6 vers effecteur)
- Effecteur : 0.1m d'offset

## Exemple de sortie

```
=== Test of forward kinematics ===
Position de repos: [0, -0.3, 0.5]
Position de test 1: [0.153692, -0.140951, 0.479251]

=== Test of inverse kinematics ===
Target position: [0.153692, -0.140951, 0.479251]
Inverse kinematics succeeded!
Joint values: 0.785398 0.523599 -0.523599 0.785398 1.0472 -0.785398

=== Initializing OpenGL Viewer ===
OpenGL viewer initialized successfully!
```

## Résolution des problèmes

### Erreur d'initialisation OpenGL
Si le visualiseur ne s'initialise pas :
- Vérifiez que votre système supporte OpenGL 3.3+
- Installez les pilotes graphiques appropriés
- Assurez-vous que GLFW et GLEW sont installés

### Compilation échouée
- Vérifiez que toutes les dépendances sont installées
- Utilisez un compilateur compatible C++17
- Vérifiez que Eigen3 est trouvé par le système

## Extension

Ce démo peut être étendu pour :
- Ajouter des obstacles dans l'environnement
- Implémenter la planification de trajectoire
- Ajouter des capteurs virtuels
- Intégrer avec ROS (Robot Operating System)
- Ajouter des effets visuels (ombres, textures)

## Architecture

Le code combine :
- **Robotik::RobotArm** : Modèle cinématique
- **Robotik::RobotViewer** : Visualisation OpenGL
- **Robotik::Camera** : Contrôle de caméra 3D
- **GLFW** : Gestion des fenêtres
- **OpenGL Core 3.3** : Rendu graphique