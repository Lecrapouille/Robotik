**WARNING: This project is currently in at its beginning age and is currently instable. Use it with care!**

See my [Youtube video](https://www.youtube.com/watch?v=BgFjewCz328)

# 🤖 RobotIK

C++ robotics library for simulation and visualization of robot (libraries and stand-alone applications).

- **Compages** : monde, hiérarchie spatiale, articulations légères, rendu.
- **Pinocchio** : cinématique directe, jacobiennes, cinématique inverse.
- **MuJoCo** : simulation dynamique, contacts, actionneurs.
- **Skills** : Home, MoveJoint, MoveJoints, MoveTCP, gripper. Les mêmes skills tournent sur MuJoCo et, plus tard, sur le robot réel.

## ⚙️ Compilation

### Prerequisites

Debian / Ubuntu:

```bash
sudo apt-get install build-essential cmake git \
    libeigen3-dev libgl1-mesa-dev libglew-dev libglfw3-dev \
    libsfml-dev swi-prolog-dev swi-prolog

# Optional, for `make tests`:
sudo apt-get install libgtest-dev libgmock-dev
```

Fedora:

```bash
sudo dnf install gcc-c++ make cmake git curl \
    eigen3-devel libglvnd-devel glew-devel glfw-devel \
    SFML-devel swi-prolog-core pkgconf-pkg-config

# Optional, for `make tests`:
sudo dnf install gtest-devel gmock-devel
```

Compages is also installed on the system (`pkg-config --exists Compages`) or
built from the clone listed in `external/manifest`. Pinocchio and MuJoCo are
fetched into `external/forge` by `make compile-external-libs` (conda-forge,
no root required). The project is C++20.

`cmake` is not used to build Robotik itself: it builds rapidyaml, the YAML
backend that [BlackThorn](https://github.com/Lecrapouille/BlackThorn) pulls in.
SFML is only needed for its network module, which carries the behavior tree
state to the Oakular visualizer.

> **Note:** the build resolves these through `pkg-config`, which fails as a
> whole as soon as one module of the list is missing. A single absent package
> therefore drops the flags of all the others, and the failure surfaces later as
> unrelated missing headers or undefined OpenGL symbols. Check the whole set
> with `pkg-config --exists eigen3 gl glew glfw3 sfml-network swipl` before
> digging further.

### Building

```bash
git clone https://github.com/Lecrapouille/Robotik --recurse
cd Robotik
make download-external-libs
make compile-external-libs
make -j8
make applications -j8
sudo make install

# Optional:
make tests -j8
```

`make download-external-libs` clones the third-party projects listed in
[external/manifest](external/manifest), and is only needed on a fresh clone or
after the manifest changes. `make compile-external-libs` runs
[external/compilation](external/compilation), which builds rapidyaml and, when they
are missing, installs Pinocchio and MuJoCo into `external/forge`. The build
triggers it when the archives it produces are missing.

A `build` folder holds `librobotik-core.so` and the applications
`Robotik-Headless` and `Robotik-Simulator`.

## Applications

Headless, no window. Drives one joint through MuJoCo and checks Pinocchio:

```bash
./build/Robotik-Headless data/simple_revolute_robot.urdf
```

Simulator. Compages draws the URDF, the right mouse button orbits:

```bash
./build/Robotik-Simulator data/simple_revolute_robot.urdf
```

## Architecture

Compages owns the world. Pinocchio is the analytical service (forward
kinematics, Jacobians, inverse kinematics). MuJoCo is the simulator. Robotik
keeps the robotic components, the name mapping, the controllers and the skills.
A skill reads `JointState` and writes `JointCommand`. It does not know whether
the backend is MuJoCo or, later, a real robot.

```cpp
compages::world::World world;
robotik::RobotRuntime runtime(world, "robot.urdf");
robotik::MoveJointSkill skill("revolute_joint", 0.5);

robotik::RobotContext context = runtime.context();
if (skill.tick(context, 0.002) != robotik::Status::Failure)
{
    runtime.step(0.002);
}
```

Skills: `Home`, `MoveJoint`, `MoveJoints`, `MoveTCP`, `OpenGripper`,
`CloseGripper`. BlackThorn wrappers are `registerSkillNodes()`.

## References

- [Compages Matrix Convention (`M * x`)](doc/MathMatrices.md)
- [Compages](https://github.com/Lecrapouille/Compages)
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
- [MuJoCo](https://github.com/google-deepmind/mujoco)
- [Robot course by Jacques Gangloff](https://www.youtube.com/playlist?list=PLMXdciyMZwAAUlCQ_9mVs_CqQ9YaRTptX)
