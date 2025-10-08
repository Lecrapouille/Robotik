# RobotIK 🤖

C++ robotics library for manipulation and visualization of robotic arms.

## Features ✨

- **Forward and inverse kinematics** : Position and orientation calculations
- **Scene graph** : Hierarchical representation of robots
- **Jacobian** : Calculation for velocity control
- **OpenGL visualization** : Real-time 3D rendering with interactive controls
- **Multi-joint support** : Rotary, prismatic and fixed joints

## Prerequisites 📋

### Basic installation

```bash
sudo apt-get install libeigen3-dev
```

### For OpenGL viewer

```bash
sudo apt-get install libgl1-mesa-dev libglew-dev libglfw3-dev
```

## Compilation ⚙️

```bash
git clone https://github.com/Lecrapouille/Robotik --recurse
cd Robotik
make -j8
# Optional:
make tests -j8
sudo make install
```

A `build` folder should have been created, it contains the created libraries as well as the demos.

## Viewer 👁️

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

## Project structure 🏗️

```bash
Robotik/
├── 📚 include/Robotik/
│   ├── Robotik.hpp                    # Main entry point
│   ├── Core/                          # Core robotics functionality
│   │   ├── Robot.hpp                  # Main robot control API
│   │   ├── URDFParser.hpp             # URDF file parser
│   │   ├── IKSolver.hpp               # Inverse kinematics solvers
│   │   ├── Joint.hpp                  # Joint representation
│   │   ├── Link.hpp                   # Link representation
│   │   ├── Geometry.hpp               # Geometric primitives
│   │   ├── SceneNode.hpp              # Scene graph node
│   │   ├── Types.hpp                  # Common types
│   │   └── ...
│   └── Viewer/                        # OpenGL viewer
│       ├── Application.hpp            # Base application class
│       ├── Camera.hpp                 # 3D camera
│       ├── OpenGLWindow.hpp           # Window management
│       ├── ShaderManager.hpp          # Shader management
│       ├── MeshManager.hpp            # Mesh loading/caching
│       ├── GeometryRenderer.hpp       # Primitive rendering
│       ├── RobotManager.hpp           # Robot rendering
│       ├── STLLoader.hpp              # STL file loader
│       └── Path.hpp                   # File path utilities
├── 🔧 src/
│   ├── Robotik/                       # Core implementation
│   │   ├── Robot.cpp
│   │   ├── URDFParser.cpp
│   │   ├── IKSolver.cpp
│   │   ├── Joint.cpp
│   │   ├── Link.cpp
│   │   ├── Geometry.cpp
│   │   ├── SceneNode.cpp
│   │   └── ...
│   └── Viewer/                        # Viewer implementation
│       ├── Application.cpp
│       ├── Camera.cpp
│       ├── OpenGLWindow.cpp
│       ├── ShaderManager.cpp
│       ├── MeshManager.cpp
│       ├── GeometryRenderer.cpp
│       ├── RobotManager.cpp
│       ├── STLLoader.cpp
│       └── Path.cpp
├── 🧪 tests/                          # Unit tests
│   ├── TestRobot.cpp
│   ├── TestJoint.cpp
│   ├── TestUrdfParser.cpp
│   └── ...
├── 📊 data/                           # Example URDF files
│   ├── cartesian_robot.urdf
│   ├── scara_robot.urdf
│   └── meshes/                        # 3D mesh files (STL)
└── 📖 doc/                            # Documentation
    └── demos/viewer/                  # Viewer demo application
```

## Architecture 🏛️

### Core Module (Robotics)

```
┌─────────────────────────────────────────────────────────────────────┐
│                          robotik::Robot                              │
│                   (Complete robotic system)                          │
└──────────────────┬──────────────────────────────────────────────────┘
                   │ uses
                   ▼
┌─────────────────────────────────────────────────────────────────────┐
│                     robotik::scene::Node                             │
│                   (Scene graph hierarchy)                            │
└──────────────────┬──────────────────────────────────────────────────┘
                   │ inherits
          ┌────────┴────────┬────────────────┐
          ▼                 ▼                ▼
    ┌─────────┐      ┌─────────┐      ┌──────────┐
    │  Joint  │      │  Link   │      │ Geometry │
    └─────────┘      └─────────┘      └──────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                      robotik::URDFParser                             │
│                   (Creates Robot from URDF)                          │
└──────────────────────────────────────────────────────────────────────┘
                                │ creates
                                ▼
                          ┌─────────┐
                          │  Robot  │
                          └─────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                      robotik::IKSolver                               │
│                 (Inverse Kinematics solver)                          │
└──────────────────┬──────────────────────────────────────────────────┘
                   │ inherits
                   ▼
          ┌───────────────────┐
          │ JacobianIKSolver  │
          └───────────────────┘
```

### Viewer Module (3D Visualization)

```
┌─────────────────────────────────────────────────────────────────────┐
│                 robotik::viewer::Application                         │
│                      (Base app class)                                │
└──────────────────────────────────────────────────────────────────────┘
                   │ inherits
                   ▼
          ┌────────────────────┐
          │  RobotViewerApp    │
          │  (User's app)      │
          └────────┬───────────┘
                   │ uses
      ┌────────────┼────────────┬────────────┬───────────┐
      ▼            ▼            ▼            ▼           ▼
┌──────────┐ ┌──────────┐ ┌────────┐ ┌──────────┐ ┌──────────┐
│ OpenGL   │ │  Camera  │ │ Shader │ │   Mesh   │ │  Robot   │
│ Window   │ │          │ │ Manager│ │  Manager │ │  Manager │
└──────────┘ └──────────┘ └────┬───┘ └────┬─────┘ └────┬─────┘
                                │          │            │
                                │          │ uses       │
                                ▼          ▼            │
                           ┌──────────┐ ┌──────────┐   │
                           │ Geometry │ │   STL    │   │
                           │ Renderer │ │  Loader  │   │
                           └──────────┘ └──────────┘   │
                                                        │ uses
                                                        ▼
                                                   ┌─────────┐
                                                   │  Robot  │
                                                   │ (Core)  │
                                                   └─────────┘
```

### Class Descriptions 📝

#### Core Module (namespace `robotik`)

**🤖 Robot**
- Complete robotic manipulator system
- Manages kinematic chain (joints + links)
- Forward kinematics, inverse kinematics, Jacobian computation
- Joint configuration management

**🔗 Joint**
- Robotic joint representation (revolute, prismatic, fixed, continuous)
- Motion axis, position value, limits
- Transform propagation in kinematic chain

**🔲 Link**
- Rigid body connecting joints
- Visual and collision geometry
- Part of the scene graph hierarchy

**📐 Geometry**
- Geometric primitives (box, cylinder, sphere, mesh)
- Used for visualization and collision detection
- Supports STL mesh files

**🌳 scene::Node**
- Scene graph node with hierarchical transforms
- Parent-child relationships
- Local and world transformations

**📄 URDFParser**
- Parses URDF files (Unified Robot Description Format)
- Automatically builds complete robot from XML description
- Creates scene graph with joints and links

**🎯 IKSolver / JacobianIKSolver**
- Inverse kinematics solvers
- Jacobian-based iterative method with damping
- Computes joint values for desired end-effector pose

#### Viewer Module (namespace `robotik::viewer`)

**🖼️ Application**
- Base application class with main loop
- Handles rendering (FPS control) and physics threads
- Abstract interface for setup, draw, update callbacks

**📷 Camera**
- 3D camera management
- Multiple view types (perspective, top, front, side, isometric)
- View and projection matrices

**🪟 OpenGLWindow**
- OpenGL window creation and management
- GLFW/GLEW initialization
- Input callbacks (keyboard, mouse, scroll)

**🎨 ShaderManager**
- Compiles and manages OpenGL shaders
- Program switching and uniform management
- Vertex and fragment shader handling

**🗂️ MeshManager**
- Loads and caches 3D meshes
- STL file support (ASCII and binary)
- OpenGL buffer management (VAO/VBO/EBO)

**🎭 GeometryRenderer**
- Renders basic 3D primitives
- Box, cylinder, sphere, grid, coordinate axes
- Manages geometry buffers

**🦾 RobotManager**
- Manages multiple robot instances
- Robot visualization and control
- Animation and inverse kinematics modes

**📦 STLLoader**
- Loads STL mesh files
- Supports ASCII and binary formats
- Extracts vertices, normals, and indices

## API 🔌

Namespace is `robotik`.

### Quick Robot Creation 🚀

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

### Forward Kinematics ⚡

(work in progress API)
- Par nom de joints
- Utiliser les contraintes.

```cpp
std::vector<double> joint_values = p_robot->jointValues();
robot->setJointValues(joint_values);
```