# RobotIK

C++ robotics library for manipulation and visualization of robotic arms.

## Features

- **Forward and inverse kinematics** : Position and orientation calculations
- **Scene graph** : Hierarchical representation of robots
- **Jacobian** : Calculation for velocity control
- **OpenGL visualization** : Real-time 3D rendering with interactive controls
- **Multi-joint support** : Rotary, prismatic and fixed joints

## Prerequisites

### Basic installation

```bash
sudo apt-get install libeigen3-dev
```

### For OpenGL viewer

```bash
sudo apt-get install libgl1-mesa-dev libglew-dev libglfw3-dev
```

## Compilation

```bash
git clone https://github.com/Lecrapouille/Robotik --recurse
cd Robotik
make -j8
# Optional:
make tests -j8
sudo make install
```

A `build` folder should have been created, it contains the created libraries as well as the demos.

## Viewer

Allows loading a new robot from a URDF file, visualizing it and controlling it.

```bash
./build/viewer-demo <path/to/your/robot/file.urdf>
```

The application expected to have a URDF file to load. This project contains some files in the [data](data) folder.

**Viewer controls:** (in progress)

- Mouse : Camera rotation
- Scroll wheel : Zoom
- W/S : Move closer/farther
- ESC : Quit

## Project structure

```bash
Robotik/
├── include/Robotik/
│   ├── Robot.hpp        # Main robot API
│   ├── Parser.hpp       # Class creating a robot from a URDF file
│   └── Viewer.hpp       # OpenGL robot viewer
├── src/
│   ├── Robotik.cpp      # Robotics implementation
│   ├── Parser.cpp       # URDF parser implementation
│   └── Viewer.cpp       # OpenGL implementation
└── demos/
    └── RobotViewer/     # OpenGL example
```

## API

Namespace is `robotik`.

### Quick Robot Creation

You need the class `URDFParser` just the time to create a new `std::unique_ptr<Robot>`. You can use it as a local variable.

```cpp
std::string urdf_file = "xxxx.urdf";
robotik::URDFParser parser;

auto robot = parser.load(urdf_file);
if (!robot)
{
    std::cerr << "Failed to load robot from '" << urdf_file
                << "': " << parser.getError() << std::endl;
    return nullptr;
}
```

### Forward kinematic

(work in progress API)
- Par nom de joints
- Utiliser les contraintes.

```cpp
std::vector<double> joint_values = p_robot->jointValues();
robot->setJointValues(joint_values);
```