# Line follower

[⏫ Case studies](README.md)

A mobile robot with two wheels, a camera, AprilTags on the floor, and a line
to follow for one lap. [Video](https://youtu.be/T0Oop_cmlPw?si=UKg83e5xunciYAZ1).

```text
camera → perception (AprilTag, line) → localisation → controller → differential robot
```

The simulated **odometry is wrong on purpose**: it drifts. A perfect
odometry would prove nothing about localisation. The tags correct the
estimate, and you can draw the truth against the estimate:

```bash
./build/linux-gcc-debug/bin/Robotik-LineFollower --seed 4 --laps 1 --save map.png   # green: truth, red: estimate
./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/LineFollower/scenarios/line_follower.scenario.yaml --seed 4
```

To watch the robot, open the simulator (**Demos → Line follower**). The
**Cameras** panel shows what the robot sees, with the tags and the line it
detects; the **Line follower** panel gives the estimate against the truth.
`Robotik-LineFollower` has no window: it runs the scenario and draws the map.

Tree: `Localize`, then `UntilSuccess` (3 attempts) of `ReachLine` +
`FollowLine`. Assertions: `(>= laps 1)`, `(< cross_track.max 0.08)`, `(< time 150)`.

Needs OpenCV (the `core` and `imgproc` modules only) and AprilTag at build
time.
