# 🧭 Guides

[⏫ Documentation](../README.md)

A guide gives the steps of one task in a plugin that already exists. The guides expect that you know the [tutorials](../tutorials/README.md). For the reasons behind a choice, read the linked [concepts](../concepts/README.md).

Robots and tools:

- [Mount a tool on a robot](mount-a-tool.md): the world file of a robot with a tool, and how to check the heights of its frames.
- [Share 3D models in an asset pack](share-3d-models.md): a versioned folder of URDF files and meshes that several plugins use, and how to read its errors.

Skills and trees:

- [Write a skill that moves to an object](write-a-skill.md): read where the robot believes the object is, and reuse `MoveTCPSkill` to go there.
- [Give arguments to a skill from the tree](write-tree-nodes.md): write a tree node that reads a value of the tree and gives it to a skill.
- [Inject a fault and recover from it](recover-from-a-fault.md): break a device during a run, and make the tree react.

Learning and demos:

- [Build reinforcement learning environments](reinforcement-learning.md): environments, spaces, the end of an episode, many environments at a time. RobotIK gives the environments; the learning algorithm is yours.
- [Write a good demo](write-a-good-demo.md): one question, one plugin, and assertions that answer it.
