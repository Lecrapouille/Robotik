# 🎬 Case studies: the demos

[⏫ Documentation](../README.md)

Each demo is a plugin in `demos/` (the tutorial plugin HelloWorld is in `doc/tutorials/HelloWorld/`). Each one shows **one idea** of robotics, and each one is also a test: its scenarios have assertions, and they run in Headless.

| Plugin | Idea | Scenarios | Experiment | Extra program |
|--------|------|-----------|------------|---------------|
| [HelloWorld](hello-world.md) | Learn the API, one lesson per scenario | hello, wave, reach, skills | `hello_experiment` | — |
| [PickAndPlace](pick-and-place.md) | Perception + skills + behavior tree or PDDL planner + faults | nominal, faults, pddl, pddl_faults, pabt | `pick_and_place_experiment` | — |
| [Painter](painter.md) | Free bodies, trajectories in files, "nothing moved", planned layers | painter, painter_pddl | `painter_experiment` | — |
| [LineFollower](line-follower.md) | Mobile robot, localisation, control loop | line_follower | — | `RobotIK-LineFollower` |
| [PickAndPlace](pick-and-place-rl.md) | RL environment, parallel pool | pick_and_place_rl | — | `RobotIK-PickAndPlaceRL` |
| [FlyBrain](fly-brain.md) | A biological brain as an agent | fly_obstacle_avoidance | — | `RobotIK-Fly` |
| [HumanoidWalk](humanoid-walk.md) | A walking policy learned elsewhere (ONNX), on a floating base | walk, walk_circle | — | — |
| [DualArm](dual-arm.md) | Two robots, one plan: steps in parallel, required concurrency | conveyor, tray | — | — |
| [MicroDuck](micro-duck.md) | A small walking robot in a room: a learned policy, real contacts | walk_forward, tour, drive | — | — |

All demos open from the simulator **Demos** menu, and run with:

```bash
./build/linux-gcc-debug/bin/RobotIK-Headless build/linux-gcc-debug/lib/robotik/plugins/<Plugin>/scenarios/<scenario>.scenario.yaml
```

To write a demo of your own, read [Write a good demo](../guides/write-a-good-demo.md).
