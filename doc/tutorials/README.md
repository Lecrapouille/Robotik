# 🎓 Tutorials

[⏫ Documentation](../README.md)

A tutorial teaches by doing. Follow the steps in order: each one gives a result that you can see. You do not need any robotics background. If a word is new, look in the [glossary](../reference/glossary.md).

1. [Getting started](01-getting-started.md): build RobotIK, watch a robot in the simulator, run the same scenario without a window, break a test on purpose, run an experiment. About 20 minutes, most of it compilation.
2. [Three programs, one per layer](02-three-programs.md): move a joint, run a skill, then load a scenario, each time in a few lines of C++.
3. [A cube in a box](03-a-cube-in-a-box.md): add objects to a world, write an assertion, and see what a seed changes. YAML only.
4. [Your first plugin](04-your-first-plugin.md): what a plugin is, how a host loads it, and the smallest scenario logic. Change its C++ code and see the verdict change.
5. [Move the arm](05-move-the-arm.md): command one joint, then reach a point with inverse kinematics.
6. [A skill and a behavior tree](06-skills-and-a-behavior-tree.md): write a skill, register it with its resources, and give the order of the skills in a tree.
7. [Your own plugin](07-your-own-plugin.md): write a plugin from zero, build it, run it, and change its robot.
8. [A planned pick and place](08-a-planned-pick-and-place.md): give a goal instead of a behavior tree, read the PDDL domain, and watch the planner plan again after a fault.
9. [See and grasp](09-see-and-grasp.md): from the camera to the grasp. Add noise to the beliefs, and find the error that makes the grasp fail.

Tutorials 4 to 6 read the HelloWorld plugin, one lesson each. Tutorial 7 uses all of them to write a new plugin.

After the tutorials, the [guides](../guides/README.md) give the steps of one task, and the [concepts](../concepts/README.md) explain why RobotIK works the way it does.
