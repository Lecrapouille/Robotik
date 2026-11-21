# 🎨 Painter

[⏫ Case studies](README.md)

An arm dips a brush in four paint pots, paints one stroke per colour on a
canvas standing on an easel, and rinses the brush between colours.

- The easel is a **free body**: pushed, it slides or tips over. The canvas is
  a free box resting on it. Both are entities of
  `plugins/Painter/worlds/painting_lab.world.yaml`; the scenario logic finds them by id and
  only brings the gestures.
- The strokes are **trajectories** in `scenarios/trajectories/`, written in
  the easel frame.
- Custom tree nodes (`Dip`, `Mark`, `Rinse`) give the colour to the skills.
- **Straight strokes.** `Mark` cuts each stroke into 2 mm Cartesian steps,
  each one an IK target, and the arm joints are stiff (`kp` 1500, `kd` 80).
  Gains alone are not enough: between two far targets each joint moves on
  its own, and the brush draws an arc (2 to 3 cm off the line). The plugin
  publishes `painter.stroke_deviation`, the largest distance between the
  inked points and the straight line (about 1.5 mm), and the scenario
  asserts it. All of it is in `parameters:` of the scenario:

  ```yaml
  parameters:
    stroke_step: 0.002
    stroke_tolerance: 0.02
    arm_gains: { kp: 1500, kd: 80 }
  ```

Zero collision does not prove the easel stayed in place, so the scenario
measures it:

```yaml
  - { condition: (< (pose_error easel) 0.002), phase: always }
  - { condition: (< (pose_error canvas) 0.005), phase: always }
  - { condition: (on canvas easel), phase: end }
  - name: brush loaded before the first stroke
    sequence: [ (skill_succeeded dip), (skill_started mark) ]
```

The experiment `painter_experiment` paints again on a slippery floor and with
a canvas four times heavier.

**With the PDDL planner** (`scenarios/painter_pddl.scenario.yaml`): the goal is the
four layers painted and a clean brush. The facts `(before red black)`… give
the order of the layers; the planner finds the twelve gestures (dip, mark,
rinse per colour). The robot cannot see the paint: the observers read what
the gestures did (`Stroke::brush`, `Stroke::painted` in
`src/PainterScene.hpp`). `painter_pabt.scenario.yaml` grows a tree from the
goal instead (PA-BT): it does not read the `forall` of the layer order, it
paints them in the order of the goal.
