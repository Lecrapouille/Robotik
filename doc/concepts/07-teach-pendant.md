# 🕹️ The teach pendant

[⏫ Concepts](README.md)

This page explains what a teach pendant is, the words that come with it (jog, waypoint, smooth move), and what the RobotIK teach pendant does and does not do. You do not need to know robotics. Read [Robotics in ten minutes](01-robotics-in-ten-minutes.md) first if *joint*, *tool* and *frame* are new words.

## 🏭 The problem: give a path to an arm without code

In a factory, the operator who sets up a robot is often not a programmer. To teach the robot a motion, they hold a box with buttons and a screen, connected to the robot by a cable. This box is the **teach pendant**. With it, the operator:

1. pushes the arm, step by step, to a good position;
2. saves this position;
3. does this again for each position of the task;
4. asks the robot to replay the saved positions, in order.

This way of programming is called *teaching by showing*. It is slow for long paths, but anyone can do it, and the robot then goes exactly where the operator saw it go.

RobotIK gives the same tool in the simulator (the **Teach Pendant** panel) and in C++ (the `robotik::TeachPendant` class, `include/RobotIK/Robot/TeachPendant.hpp`). Use it to make a motion before you write a skill, to check that the robot can reach a place, or to find the joint values of a pose for a world file.

## 👆 Jog: move by small steps

To **jog** is to move the robot by one small step, each time you push a button. There are two kinds.

A **joint jog** turns one joint (or slides it, for a joint that slides) by a fixed step, for example 0.05 rad. It is simple and always possible, but the tool then moves along a curve that is not easy to guess.

A **tool jog** moves the tool along a straight line (1 cm up) or turns it about an axis, in the frame of the robot base: X, Y and Z, then Rx, Ry and Rz for the turns. The operator thinks about the tool, not about the joints. RobotIK must then find the joint values that put the tool at the new place. This calculation is the inverse kinematics (IK, see the [glossary](../reference/glossary.md)). It can fail: the place can be out of reach, or the joints cannot move the tool in this direction.

The moved part is the tool by default, but you can choose another link: the foot of a walking robot, or the elbow of an arm. A link with fewer than six joints above it cannot go everywhere. For example, the first link of an arm only turns about Z. The panel greys out the buttons of the directions that cannot work.

## 📍 Waypoint: a saved position

A **waypoint** is a position of the robot that the operator saved. RobotIK saves the value of every joint: an angle for a joint that turns, a length for a joint that slides. These values together are sometimes called the *configuration* of the robot.

Why the joints and not only the place of the tool? An arm can put its tool at the same place with different shapes: elbow up or elbow down, for example. Only the joint values say which shape. The waypoint also keeps the place of the tool, to show it to the operator, and its name ("above", "wp 2").

Each waypoint has a **duration**: the time the replay takes to come to this waypoint from the one before. A longer duration gives a slower, softer move.

## 🎬 Replay and the smooth move

During the replay, the robot goes from one waypoint to the next. If each joint moved at a constant speed, the arm would start and stop with a jerk, which is bad for the motors and for what the robot holds. The pendant uses a **smooth move**: each joint follows a curve that starts at zero speed and ends at zero speed, with no sudden change of acceleration. Engineers call this curve a *quintic* (a polynomial of degree 5) or a *rest-to-rest* move.

Each joint moves on its own curve, and all the joints arrive at the same time. The tool does not follow a straight line between two waypoints. If you need a straight line, add more waypoints, or use a cartesian trajectory of the scenario (see [the scenario file](../reference/scenario-file.md)).

## 🖥️ In the simulator

Open the **Robot** workspace, then the **Teach Pendant** panel. The first button you push takes the robot from the mission: the simulation is paused, the Inspector shows **Manual**, and the mission stops writing targets, so the arm obeys you.

- **Move a frame**: choose the moved link, then push X, Y, Z (straight moves) or Rx, Ry, Rz (turns). The steps are set under the buttons. **Axes** shows the moved frame in the 3D view (X red, Y green, Z blue).
- **Waypoints**: type a label and a duration, push **Record**. Each line has **Go** (go to this waypoint) and **Delete**. **Play waypoints** replays the list; **Loop** starts again after the last one; **Stop** stops at once; **Clear all** removes the list.
- The red line under the buttons tells why the last command did not work.

## 💻 In C++

The pendant only writes the joint targets: the positions that the motors must reach. The arm moves when the simulation steps. So, during a replay, call `update` and then step the robot, at each step:

```cpp
using namespace robotik::literals; // 0.01_m, 0.2_rad, 2.0_s

robotik::TeachPendant pendant;

// Lift the tool by 1 cm and save this position.
pendant.jogTool(robot, { 0.0_m, 0.0_m, 0.01_m }, robotik::Vector3rad::ZERO);
pendant.record(robot, "above", 2.0_s); // reached in 2 s during replay

// Turn the wrist by 0.2 rad and save it too.
pendant.jogJoint(robot, wrist, 0.2_rad);
pendant.record(robot, "turned", 1.0_s);

// Replay the two positions, again and again.
pendant.play(robot, true);
while (pendant.isPlaying()) {
    pendant.update(robot, dt);   // next targets of the move
    robot.step(dt);              // the physics moves the arm
}
```

The test `TeachPendant.HeaderExampleTeachesAndReplaysTwoPositions` in `tests/Robot/TestTeachPendant.cpp` runs this code (it lets the arm settle after each jog, because `record` saves where the arm really is, not the target).

## ⚠️ Limits

- The replay moves in joint space: no straight line of the tool between waypoints.
- A waypoint saved on a robot with another number of joints is refused.
- The pendant and a mission cannot drive the robot together. In C++, pause the mission first (`RoboticCell::suspend`); the panel does it for you.
- Waypoints are not saved in a file yet; they are lost when the scenario is loaded again.
