# 🧊 A cube in a box

[⏫ Tutorials](README.md)

You will add two objects to the HelloWorld world and check a fact about them.
Work in the sources, then run `make` to copy them to `build/`.

**Step 1.** Open `plugins/HelloWorld/worlds/arm.world.yaml`. It has one entity, the
robot. Add a red cube and a box at the end of the `entities` list:

```yaml
    - id: red_cube
      type: prop
      shape: { type: box, size: [0.04, 0.04, 0.04] }    # 4 cm cube
      color: [0.9, 0.1, 0.1]                             # red, green, blue in [0, 1]
      pose: { position: [0.40, 0.20, 0.02] }

    - id: box
      type: prop
      shape: { type: container, size: [0.16, 0.16, 0.08] }   # an open box
      pose: { position: [0.40, -0.20, 0.04] }
```

A `prop` is an object that does not move by itself (no physics), unless the
robot grasps it. Positions are in metres, z up. The position is the **centre**
of the object: the cube is 4 cm high, so its centre is at z = 0.02.

**Step 2.** Open `plugins/HelloWorld/scenarios/hello.scenario.yaml` and add two
assertions:

```yaml
  assertions:
    - controller.success
    - (> hello.ticks 100)
    - (< time 3)
    - (< (distance red_cube box) 0.05)
    - (inside red_cube box)
```

**Step 3.** Build and run:

```bash
make robotik_hello_world
./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/HelloWorld/scenarios/hello.scenario.yaml
```

```text
[FAIL] (< (distance red_cube box) 0.05)  ((distance red_cube box) = 0.4005)
[FAIL] (inside red_cube box)  ((inside red_cube box) = 0)
```

Of course: the cube is 40 cm away from the box.

**Step 4.** "Cheat": put the cube in the box directly. The box floor is 5 mm
thick, so the cube centre is at 0.005 + 0.02 = 0.025:

```yaml
      pose: { position: [0.40, -0.20, 0.025] }
```

```text
[PASS] (< (distance red_cube box) 0.05)  ((distance red_cube box) = 0.015)
[PASS] (inside red_cube box)  ((inside red_cube box) = 1)
hello: 1/1 runs passed
```

**Step 5.** Open it in the simulator to see the scene:

```bash
./build/linux-gcc-debug/bin/Robotik-Simulator build/linux-gcc-debug/lib/robotik/plugins/HelloWorld/scenarios/hello.scenario.yaml
```

Of course, a real mission would ask the robot to *move* the cube. This is
what `plugins/PickAndPlace` does, with the same world entities and the same
`(inside red_cube box)` assertion. When you are done, put the files back with
`git checkout plugins/HelloWorld`.

## 🎲 Randomness and seeds

Real objects are never exactly where we expect. Ask Robotik to move the cube
a little at each run:

```yaml
    - id: red_cube
      type: prop
      shape: { type: box, size: [0.04, 0.04, 0.04] }
      pose: { position: [0.40, 0.20, 0.02] }
      randomize: { x: [-0.02, 0.02], y: [-0.02, 0.02] }   # ± 2 cm
```

The random numbers come from the **seed** of the scenario (`seed: 1`). Same
seed, same run, always. Another seed, another layout:

```bash
./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/HelloWorld/scenarios/hello.scenario.yaml --seed 7
```

The robot knows only the *nominal* position (the one written in the file).
The *true* position, used by the assertions, is the shifted one. This is how
you test that a robot uses its camera instead of the file.

A failed run in CI prints its seed: run it again with `--seed` to see exactly
the same failure.

Next: [Your first plugin](04-your-first-plugin.md).
