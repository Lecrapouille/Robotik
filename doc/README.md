# Robotik documentation

Read the chapters in order: each one uses the words of the previous ones.
Chapters 1 to 5 are for everybody, even without any robotics background.
Chapters 6 to 12 are references: open them when you need them.

## Learn

| # | Chapter | You will learn | Time |
|---|---------|----------------|------|
| 1 | [Introduction](01-Introduction.md) | Robotics words (joint, frame, TCP, kinematics, skill, behavior tree…) and what Robotik does | 10 min |
| 2 | [Getting started](02-Getting-Started.md) | Build, run the simulator and Headless, read a report, break a test on purpose | 20 min |
| 3 | [Worlds and scenarios](03-Worlds-and-Scenarios.md) | Describe a world, a task, its checks and its variants in YAML — no C++ | 30 min |
| 4 | [Your first plugin](04-Your-First-Plugin.md) | Write C++: the HelloWorld plugin lesson by lesson, then your own plugin | 45 min |
| 5 | [Skills and behavior trees](05-Skills-and-Behavior-Trees.md) | Skills, resources, the scheduler, trees, recovering from faults | 30 min |

## Reference

| # | Chapter | Contents |
|---|---------|----------|
| 6 | [Architecture](06-Architecture.md) | Layers, one simulation step, backends, truth vs belief, source layout |
| 7 | [API reference](07-API-Reference.md) | The classes a plugin uses, grouped by task |
| 8 | [Reinforcement learning](08-Reinforcement-Learning.md) | Environments, spaces, episode ends, parallel pool, episode log |
| 9 | [Demos](09-Demos.md) | Every plugin of the repository, and how to write a good demo |
| 10 | [Tools and frames](10-Tools-and-Frames.md) | Flange, `tool0`, `tool_mount`, TCP; mounting a tool |
| 11 | [The PDDL planner](11-PDDL-Planner.md) | A goal instead of a tree: domain, observers, bindings, replanning |
| 12 | [GRAFCET, GEMMA and workflows](12-GRAFCET-and-Workflows.md) | A GRAFCET instead of a tree, start and stop modes (GEMMA), cycle time and bottleneck of a cell |
| — | [Glossary](Glossary.md) | The words of RobotIK (asset, pack, skill, belief, fidelity...) in simple English |

## Quick paths

- *"I just want to see something move."* → [2. Getting started](02-Getting-Started.md)
- *"I want to change a test without compiling."* → [3. Worlds and scenarios](03-Worlds-and-Scenarios.md)
- *"I want to write my own robot program."* → [4. Your first plugin](04-Your-First-Plugin.md)
- *"Which method do I call?"* → [7. API reference](07-API-Reference.md)
- *"I want to change Robotik itself."* → [6. Architecture](06-Architecture.md), and [AGENTS.md](../AGENTS.md) for the conventions

## A note on the language

This documentation is written in simple English, with short sentences and
common words, so that readers whose first language is not English (French
readers in particular) can follow it easily.
