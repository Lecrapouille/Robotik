# 🎲 Pick and place RL

[⏫ Case studies](README.md)

The classic learning loop on the pick-and-place cell: `Environment`, parallel
pool, reproducibility. Training a real PPO or SAC policy is **not** the goal
here: it validates the infrastructure.

```bash
./build/linux-gcc-debug/bin/Robotik-PickAndPlaceRL --policy converged --envs 16 --threads 8 --episodes 32 \
    --log episodes.json
```

This scenario lives in the PickAndPlace plugin, beside the behavior tree
ones (submenus of the **Demos** menu). Its entry in `plugin.yaml` has
`clock: plugin`: the plugin `update` advances the simulation. The pool and
the plugin share `PickPlaceControl.cpp`. Details in
[Build reinforcement learning environments](../guides/reinforcement-learning.md).
