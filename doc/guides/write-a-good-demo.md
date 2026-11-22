# 🎬 Write a good demo

[⏫ Guides](README.md)

A good demo answers **one yes/no question**:

```text
Can I detect an object with a camera?
Can I localise a robot with AprilTags?
Can a skill fail cleanly?
Can I re-plan after a fault?
Can I compare two control strategies on identical runs?
```

And it is:

- **small**: one idea per demo;
- **reproducible**: a seed, assertions, a headless run;
- **readable**: YAML first, little C++;
- **observable**: in the simulator, or with `--trace` and reports;
- **testable**: explicit success criteria.

When a demo is stable, it runs both ways — `RobotIK-Simulator x.scenario.yaml` and `RobotIK-Headless x.scenario.yaml` — and it becomes a regression test.

## 💡 Ideas for new demos

- **A real SO-101 arm**: the same scenario, only the backend changes.
- **RGB-D perception**: camera → depth → Open3D / OpenCV → WorldModel → pick.
- **Planning**: goal → PDDL / GOAP → behavior tree → skills.
- **Reasoning**: WorldModel → Prolog facts and rules → decision.
- **Strategy comparison**: the same scenario solved by a BT, a planner and a policy, compared on time, collisions, failures and robustness to faults.
