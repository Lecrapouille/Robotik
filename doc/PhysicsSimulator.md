# PhysicsSimulator - Guide d'utilisation

Le `PhysicsSimulator` est un module qui permet de simuler la dynamique d'un robot en appliquant les forces de gravité, l'inertie, le frottement et l'amortissement sur les joints et les liens.

## Vue d'ensemble

Le simulateur implémente l'algorithme de Newton-Euler récursif pour calculer les dynamiques directes (forward dynamics) :
- **Entrée** : efforts/couples appliqués aux joints
- **Sortie** : accélérations, vitesses et positions des joints

### Caractéristiques

- ✅ Effet de la gravité sur les liens
- ✅ Inertie des liens
- ✅ Frottement et amortissement des joints
- ✅ Limites d'effort (couple/force maximum)
- ✅ Intégration semi-implicite d'Euler
- ✅ Support de tous les types de joints (revolute, prismatic, continuous, fixed)

## Installation

Le `PhysicsSimulator` est automatiquement compilé avec la bibliothèque `robotik-core`. Il suffit d'inclure :

```cpp
#include <Robotik/Robotik.hpp>
```

## Utilisation de base

### 1. Créer un simulateur

```cpp
#include <Robotik/Robotik.hpp>

// Créer un simulateur avec un pas de temps de 10ms
double dt = 0.01;  // 10 ms
Eigen::Vector3d gravity(0.0, 0.0, -9.81);  // Gravité terrestre
robotik::PhysicsSimulator simulator(dt, gravity);
```

### 2. Configurer les propriétés physiques des joints

```cpp
// Charger le robot
auto robot = robotik::loadRobot("mon_robot.urdf");

// Configurer les joints
for (const auto& name : robot->jointNames())
{
    auto& joint = const_cast<robotik::Joint&>(robot->joint(name));

    joint.damping(0.1);        // Coefficient d'amortissement
    joint.friction(0.05);      // Coefficient de frottement
    joint.effort_max(10.0);    // Couple/force maximum (N·m ou N)
}
```

### 3. Exécuter la simulation

```cpp
// Boucle de simulation
for (int i = 0; i < 1000; ++i)
{
    // Calculer et intégrer la dynamique
    simulator.step(*robot);

    // Observer l'état du robot
    auto positions = robot->jointPositions();
    // ... faire quelque chose avec les positions
}
```

## API Détaillée

### Constructeur

```cpp
PhysicsSimulator(double p_dt, Eigen::Vector3d const& p_gravity)
```

- `p_dt` : Pas de temps pour l'intégration (secondes)
- `p_gravity` : Vecteur de gravité (m/s²), par défaut `(0, 0, -9.81)`

### Méthodes

#### `void step(Robot& p_robot)`

Effectue un pas de simulation :
1. Calcule les dynamiques (accélérations) pour tous les joints
2. Intègre les vitesses : `v(t+dt) = v(t) + a(t) * dt`
3. Intègre les positions : `q(t+dt) = q(t) + v(t+dt) * dt`

#### `void setGravity(const Eigen::Vector3d& p_gravity)`

Modifie le vecteur de gravité.

```cpp
simulator.setGravity(Eigen::Vector3d(0.0, 0.0, -1.62)); // Gravité lunaire
```

#### `Eigen::Vector3d const& gravity() const`

Retourne le vecteur de gravité actuel.

#### `void dt(double p_dt)` et `double dt() const`

Modifie ou récupère le pas de temps.

## Propriétés des joints

Pour configurer les propriétés physiques des joints, utilisez les méthodes suivantes :

```cpp
auto& joint = const_cast<robotik::Joint&>(robot->joint("joint_name"));

// Propriétés dynamiques
joint.effort(5.0);           // Appliquer un couple/force (N·m ou N)
joint.damping(0.1);          // Amortissement (sans unité)
joint.friction(0.05);        // Frottement (sans unité)
joint.effort_max(10.0);      // Couple/force maximum (N·m ou N)

// Lecture de l'état
double pos = joint.position();      // Position (rad ou m)
double vel = joint.velocity();      // Vitesse (rad/s ou m/s)
double acc = joint.acceleration();  // Accélération (rad/s² ou m/s²)
```

## Exemple complet

```cpp
#include <Robotik/Robotik.hpp>
#include <iostream>

int main()
{
    // 1. Charger le robot
    auto robot = robotik::loadRobot("data/simple_revolute_robot.urdf");
    if (!robot)
    {
        std::cerr << "Erreur de chargement du robot" << std::endl;
        return 1;
    }

    // 2. Créer le simulateur
    robotik::PhysicsSimulator simulator(0.01);  // 10ms

    // 3. Configurer les joints
    for (const auto& name : robot->jointNames())
    {
        auto& joint = const_cast<robotik::Joint&>(robot->joint(name));
        joint.damping(0.1);
        joint.friction(0.05);
        joint.effort_max(10.0);
    }

    // 4. Position initiale
    robot->setNeutralPosition();

    // 5. Simuler pendant 2 secondes
    for (int i = 0; i < 200; ++i)  // 200 steps × 10ms = 2s
    {
        simulator.step(*robot);

        // Afficher tous les 50 steps (0.5s)
        if (i % 50 == 0)
        {
            auto positions = robot->jointPositions();
            std::cout << "t = " << i * 0.01 << "s, positions: ";
            for (double p : positions)
                std::cout << p << " ";
            std::cout << std::endl;
        }
    }

    // 6. Appliquer un couple et continuer
    auto& joint = const_cast<robotik::Joint&>(
        robot->joint(robot->jointNames()[0]));
    joint.effort(5.0);  // 5 N·m

    for (int i = 0; i < 100; ++i)
    {
        simulator.step(*robot);
    }

    std::cout << "Position finale: " << joint.position() << " rad" << std::endl;
    std::cout << "Vitesse finale: " << joint.velocity() << " rad/s" << std::endl;

    return 0;
}
```

## Détails d'implémentation

### Algorithme de Newton-Euler

Le simulateur utilise une version simplifiée de l'algorithme de Newton-Euler :

1. **Phase de propagation avant** (calcul des dynamiques) :
   - Pour chaque joint, calcul du couple/force de gravité
   - Application de l'équation : `τ_total = τ_applied + τ_gravity - b·ω - f·sign(ω)`
   - Calcul de l'accélération : `α = τ_total / I`

2. **Phase d'intégration** :
   - Intégration semi-implicite d'Euler
   - Mise à jour récursive de tous les joints

### Équation de dynamique

Pour un joint revolute :
```
α = (τ_effort + τ_gravity - damping·ω - friction·sign(ω)) / I
```

Où :
- `α` : accélération angulaire (rad/s²)
- `τ_effort` : couple appliqué (N·m)
- `τ_gravity` : couple dû à la gravité (N·m)
- `ω` : vitesse angulaire (rad/s)
- `I` : inertie équivalente (kg·m²)

### Inertie équivalente

L'inertie est approximée par :
```
I = m · L² / 3
```

Où :
- `m` : masse du lien (kg)
- `L` : longueur du lien (m)

## Notes importantes

1. **const_cast** : Pour modifier les joints pendant la simulation, il faut utiliser `const_cast` car `Robot::joint()` retourne une référence constante.

2. **Pas de temps** : Un pas de temps trop grand peut causer des instabilités. Recommandation : 0.001s à 0.01s (1ms à 10ms).

3. **Inertie** : L'implémentation actuelle utilise une approximation simplifiée de l'inertie. Pour une simulation plus précise, les propriétés inertielles complètes devraient être définies dans le fichier URDF.

4. **Limites** : Les limites de position et de vitesse des joints sont automatiquement respectées pendant l'intégration.

## Limitations actuelles

- L'algorithme ne gère pas les contraintes de boucle fermée (closed-loop mechanisms)
- Pas de détection de collision
- Pas de contact avec le sol
- Inertie simplifiée (approximation par point masse)

## Améliorations futures

- [ ] Détection de collision
- [ ] Contact avec le sol
- [ ] Intégrateur Runge-Kutta 4
- [ ] Algorithme complet de Newton-Euler avec matrices d'inertie complètes
- [ ] Support des contraintes
- [ ] Parallélisation pour les robots multi-branches

## Voir aussi

- [Joint.hpp](../include/Robotik/Core/Joint.hpp) - Documentation de la classe Joint
- [Link.hpp](../include/Robotik/Core/Link.hpp) - Documentation de la classe Link
- [Robot.hpp](../include/Robotik/Core/Robot.hpp) - Documentation de la classe Robot

