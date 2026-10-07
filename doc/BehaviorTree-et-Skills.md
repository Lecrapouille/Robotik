# Behavior Trees, Skills et Scheduler

Cette page explique une distinction fondamentale de Robotik :

> **Le Behavior Tree décide de la logique d'exécution.  
> Le scheduler décide si une skill peut s'exécuter.  
> La skill sait comment réaliser l'action.**

Ces trois notions sont volontairement séparées.

---

# 1. Une skill, c'est quoi ?

Une skill représente une **capacité du robot**.

Exemples :

```text
Home
MoveJoint
MoveJoints
MoveTCP
Stop
Detect
Approach
Reach
Grasp
Release
```

Une skill peut être très simple :

```text
MoveJoint(joint1, 0.5)
```

ou représenter une opération plus complexe :

```text
Pick(red_cube)
```

qui peut elle-même nécessiter :

```text
Detect
→ Approach
→ Reach
→ Grasp
→ Lift
```

Une skill n'est donc pas forcément un seul mouvement moteur.

---

# 2. Skill ≠ thread

Une erreur classique serait de penser :

```text
1 skill = 1 thread
```

Robotik ne fait pas cette hypothèse.

Une skill est une **opération asynchrone pilotée par le scheduler**.

Elle possède un état :

```text
Waiting
Running
Succeeded
Failed
Cancelled
```

et avance à chaque `tick`.

```cpp
skill.tick(context, dt);
```

Cela permet d'exécuter de nombreuses skills sans créer un thread pour chacune.

---

# 3. Le cycle d'une skill

Une skill suit conceptuellement :

```text
          request
             │
             ▼
         Waiting
             │
      ressources OK ?
         /       \
       non       oui
       │          │
       │          ▼
       │       Running
       │          │
       │       tick()
       │          │
       │     ┌────┴────┐
       │     │         │
       │   success    error
       │     │         │
       ▼     ▼         ▼
     attente SUCCESS FAILURE
```

Elle peut aussi être annulée :

```text
Running
   │
 cancel()
   ▼
Cancelled
```

L'annulation est coopérative : la skill doit remettre le matériel dans un état sûr.

---

# 4. Le scheduler

Le `SkillScheduler` est l'arbitre.

Supposons :

```text
Skill A = MoveArm
Skill B = Grasp
Skill C = Stop
```

Toutes les trois peuvent être demandées.

Le scheduler regarde :

```text
Skill
 ├── priorité
 ├── ressources
 ├── préconditions
 ├── état des ressources
 └── politique d'attente
```

Exemple :

```cpp
SkillDescription grasp;

grasp.name = "Grasp(red_cube)";
grasp.resources = {
    resources.require("gripper")
};
grasp.priority = 100;
```

Si le gripper est occupé :

```text
Grasp
 │
 ├── gripper busy
 │
 ├── priorité élevée
 │
 └── skill actuelle annulable ?
          │
         oui
          ▼
       préemption
```

---

# 5. Les ressources

Une ressource représente une partie du robot ou un moyen nécessaire à une opération.

Exemples :

```text
arm
gripper
wrist_camera
left_wheel
right_wheel
laser
```

Une skill peut demander :

```text
Grasp
 ├── arm      exclusif
 ├── gripper  exclusif
 └── camera   partagé
```

Deux skills peuvent donc fonctionner simultanément si elles n'ont pas de conflit.

Exemple :

```text
LookAround
    └── head_camera

MoveArm
    └── arm
```

Ces deux skills peuvent éventuellement coexister.

Mais :

```text
MoveArm
    └── arm

Grasp
    └── arm
```

sont en conflit.

---

# 6. Pourquoi ne pas laisser le Behavior Tree gérer cela ?

Parce que le Behavior Tree répond à une autre question.

Le BT dit :

> « Je veux maintenant exécuter `Grasp(red_cube)`. »

Le scheduler répond :

> « Le gripper est disponible, la caméra est disponible et les préconditions sont satisfaites : tu peux démarrer. »

Cela permet de garder le BT simple.

```text
Behavior Tree
    │
    │ "Grasp(red_cube)"
    ▼
SkillScheduler
    │
    ├── ressources ?
    ├── priorité ?
    ├── préconditions ?
    ├── panne ?
    └── préemption ?
         │
         ▼
       Skill
```

---

# 7. Le Behavior Tree

Le Behavior Tree représente la **logique de mission**.

Exemple :

```text
Sequence
├── Detect(red_cube)
├── Approach(red_cube)
├── Reach(red_cube)
├── Grasp(red_cube)
├── MoveTo(box)
└── Release(red_cube)
```

Chaque feuille est une action BlackThorn.

Mais cette action ne manipule pas directement la skill.

Elle demande la skill au scheduler.

---

# 8. Communication BT → Scheduler → Skill

```text
BlackThorn
    │
    ▼
Action("Grasp(red_cube)")
    │
    │ request()
    ▼
SkillScheduler
    │
    ├── acquire resources
    │
    ▼
GraspSkill
    │
    │ tick(context, dt)
    ▼
Robot
```

Puis :

```text
GraspSkill
    │
    ▼
Succeeded
    │
    ▼
SkillScheduler
    │
    ▼
Action
    │
    ▼
SUCCESS
```

Le Behavior Tree n'a donc pas besoin de connaître le fonctionnement interne du scheduler.

---

# 9. Une panne

Imaginons :

```text
wrist_camera = panne
```

Une skill `Detect` qui utilise cette caméra ne peut pas fonctionner.

Le scheduler peut produire :

```text
Detect(red_cube)
    ↓
Failed
ResourceUnavailable
```

Le Behavior Tree décide alors de la stratégie :

```text
Retry
   Detect
```

ou :

```text
Fallback
├── DetectWithWristCamera
└── DetectWithHeadCamera
```

La panne et la réaction sont donc deux responsabilités différentes.

---

# 10. Préconditions

Une skill peut déclarer :

```text
Grasp(red_cube)
```

avec :

```text
precondition:
    gripper is empty
    red_cube is reachable
```

Le scheduler peut vérifier ces conditions avant de lancer la skill.

Cela devient particulièrement intéressant pour la planification.

---

# 11. GOAP et PDDL

Une évolution prévue de Robotik est de permettre à un planificateur de construire automatiquement une séquence de skills.

Au lieu d'écrire directement :

```text
Detect
→ Approach
→ Reach
→ Grasp
→ Move
→ Release
```

on pourrait décrire :

```text
État initial :
    visible(red_cube)
    empty(gripper)

But :
    inside(red_cube, box)
```

Un planificateur GOAP ou PDDL pourrait produire :

```text
Detect(red_cube)
Approach(red_cube)
Reach(red_cube)
Grasp(red_cube)
MoveTo(box)
Release(red_cube)
```

Puis ce plan serait transformé en Behavior Tree.

```text
              Goal
                │
          GOAP / PDDL
                │
                ▼
             Plan
                │
                ▼
         Behavior Tree
                │
                ▼
             Skills
```

Le planificateur répond donc :

> « Quelles actions sont nécessaires ? »

Le Behavior Tree répond :

> « Comment exécuter et réagir à ces actions ? »

---

# 12. Pourquoi garder le Behavior Tree ?

Parce qu'un planificateur produit généralement un plan idéal.

Dans le monde réel :

```text
MoveTo(box)
```

peut échouer parce que :

- un objet bloque le chemin ;
- le robot n'est plus où prévu ;
- le capteur est en panne ;
- la préhension a échoué ;
- une personne est apparue.

Le Behavior Tree peut réagir :

```text
MoveTo(box)
    │
    ├── SUCCESS → Release
    │
    └── FAILURE
          │
          ├── Retry
          ├── Replan
          └── Abort
```

---

# 13. Prolog

Prolog peut compléter cette architecture avec de la logique symbolique.

Par exemple :

```prolog
inside(red_cube, box).
reachable(arm, red_cube).
empty(gripper).
```

On peut demander :

```prolog
can_grasp(arm, red_cube).
```

et utiliser la réponse pour :

- vérifier des préconditions ;
- enrichir le WorldModel ;
- choisir une skill ;
- aider un planificateur.

L'idée est donc :

```text
PDDL / GOAP
     │
     │ plan
     ▼
Behavior Tree
     │
     ▼
Skills
     │
     └──── Prolog
             │
          logique /
        connaissances
```

Prolog ne remplace pas le Behavior Tree.

---

# 14. LLM

À plus haut niveau, un LLM pourrait transformer une intention :

```text
« Prends le cube rouge et mets-le dans la boîte. »
```

en objectif structuré :

```text
goal:
    inside(red_cube, box)
```

Puis :

```text
LLM
 ↓
objectif
 ↓
PDDL / GOAP
 ↓
plan
 ↓
Behavior Tree
 ↓
Skills
```

L'intérêt est de ne pas laisser le LLM contrôler directement les moteurs.

Le LLM reste au niveau de l'intention.

---

# 15. Une skill peut être remplacée

Une autre idée importante est de ne pas considérer une skill comme une implémentation unique.

Par exemple :

```text
Reach(target)
```

peut être réalisé par :

```text
ReachSkill
 ├── Pinocchio + IK
 ├── motion planner
 ├── RL policy
 └── VLA policy
```

Le scénario reste :

```text
Reach(red_cube)
```

Le mécanisme choisi pour réaliser cette opération peut changer.

---

# 16. Tester une skill sans Behavior Tree

Le BT n'est pas nécessaire pour tester le scheduler.

On peut faire :

```cpp
auto id = scheduler.add<MoveJointSkill>(
    description,
    "joint1",
    target
);

scheduler.request(id);

while (!scheduler.finished(id))
{
    scheduler.update(context);
    robot.step(dt);
}
```

Cela facilite les tests unitaires.

Le Behavior Tree devient alors un **client du scheduler**, au même titre qu'une politique RL, une interface graphique ou un programme C++.

---

# 17. Résumé

Les responsabilités sont :

```text
GOAP / PDDL
    « quoi faire ? »
          │
          ▼
Behavior Tree
    « comment réagir ? »
          │
          ▼
SkillScheduler
    « puis-je le faire maintenant ? »
          │
          ▼
Skill
    « comment réaliser cette capacité ? »
          │
          ▼
Robot / Controller
    « quelles commandes envoyer ? »
```

Cette séparation permet à Robotik de rester simple tout en pouvant accueillir progressivement des mécanismes de planification plus avancés.
