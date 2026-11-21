# 🧠 A planned pick and place

[⏫ Tutorials](README.md)

In this tutorial a scenario gives a goal, "the red cube is in the box",
instead of a behavior tree. The planner finds the steps in the PDDL domain
of PickAndPlace, and plans again when a fault breaks a step. You need the
[first tutorials](README.md); the ideas are explained in
[The PDDL planner](../concepts/pddl-planner.md).

Run the scenario with its faults, in Headless:

```bash
./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/PickAndPlace/scenarios/pick_and_place_pddl_faults.scenario.yaml --trace
```

The camera is down at the start, back at 1 s, then down for good at 3 s. The trace (real output) ends with one line per plan:

```text
  0 s  Detect(red_cube)  FAILED (unavailable: wrist_camera)  (0 s)
  0.51 s  Detect(red_cube)  FAILED (unavailable: wrist_camera)  (0 s)
  1.02 s  Approach(red_cube)  SUCCEEDED  (2.37 s)
  ...
  6.01 s  Release  SUCCEEDED  (0.01 s)
  plan 0 at 0 s (start): solved (detect red_cube) (pick red_cube) (place red_cube box)
  plan 1 at 0.51 s (1. (detect red_cube): skill Detect(red_cube) FAILED (unavailable: wrist_camera)): solved (detect red_cube) (pick red_cube) (place red_cube box)
  plan 2 at 1.02 s (1. (detect red_cube): skill Detect(red_cube) FAILED (unavailable: wrist_camera)): solved (pick red_cube) (place red_cube box)
[PASS] controller.success  (controller.success = 1)
[PASS] (inside red_cube box)  ((inside red_cube box) = 1)
...
[PASS] (>= planner.replans 1)  (planner.replans = 2)
```

Read the plans:

1. At 0 s the robot has seen nothing: the plan starts with `detect`.
2. `Detect` fails (camera down). The executor waits 0.5 s of simulated time and plans again: same plan, same failure.
3. At 1.02 s the camera is back and has already seen the cube. The new plan has no `detect` at all.

In the simulator, open **Demos → Pick and place → PDDL planner**, press Play, and look at the **Planner** panel of the **Mission** workspace ([debugging](../reference/pddl-planner.md#-debugging)).

The tutorials end here. The [guides](../guides/README.md) give the steps of
other tasks, and the [concepts](../concepts/README.md) explain the choices
of RobotIK.
