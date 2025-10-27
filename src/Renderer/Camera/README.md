# Camera System

## Overview

This folder contains a complete game-engine-style camera system designed for robotics visualization. The architecture provides flexibility for different viewing modes and interaction styles.

## Architecture

### Camera Blueprint

```
Camera (Abstract Base Class)
├── PerspectiveCamera
└── OrthographicCamera
```

**Camera** - Abstract base class
- Common properties: position, target, up vector, aspect ratio
- View matrix computation
- Pure virtual `projectionMatrix()` method

**PerspectiveCamera** - Standard 3D perspective projection
- Field of view (FOV)
- Near/far clipping planes
- Best for realistic 3D visualization

**OrthographicCamera** - Parallel projection
- Orthographic bounds
- No perspective distortion
- Perfect for technical views (TOP, FRONT, SIDE)

### Camera Controllers

```
CameraController (Interface)
├── OrbitController ⭐ (Primary for robotics)
├── FlyController
└── DragController
```

Controllers handle user input and update camera state:

**OrbitController** ⭐ - **Recommended for robot visualization**
- Orbits around a fixed target point
- Mouse drag to rotate around target
- Right/middle mouse drag to pan
- Scroll to zoom in/out
- Perfect for inspecting robots from all angles

**FlyController** - Free-flight FPS-style camera
- WASD/arrow keys for movement
- Mouse drag to look around
- Scroll to adjust speed
- Shift for faster movement
- Good for navigating large environments

**DragController** - Simple 2D-like navigation
- Mouse drag to pan
- Scroll to zoom
- Minimal controls for simplicity

## Usage Examples

### Basic Setup with OrbitController (Recommended)

```cpp
#include "Renderer/Camera/PerspectiveCamera.hpp"
#include "Renderer/Camera/OrbitController.hpp"

// Create perspective camera
auto camera = std::make_unique<PerspectiveCamera>(
    45.0f,      // FOV in degrees
    16.0f/9.0f, // Aspect ratio
    0.1f,       // Near plane
    100.0f      // Far plane
);

// Create orbit controller
auto controller = std::make_unique<OrbitController>(
    *camera,
    Eigen::Vector3f(0.0f, 0.0f, 0.5f), // Target point (robot center)
    5.0f                                 // Initial distance
);

// In your update loop
controller->update(dt);

// Forward input events
controller->handleMouseMove(xpos, ypos);
controller->handleMouseButton(button, action, mods);
controller->handleScroll(xoffset, yoffset);

// Use camera matrices for rendering
shader.setMatrix("view", camera->viewMatrix());
shader.setMatrix("projection", camera->projectionMatrix());
```

### Orthographic Camera for Technical Views

```cpp
#include "Renderer/Camera/OrthographicCamera.hpp"

// Create orthographic camera (for TOP/FRONT/SIDE views)
auto camera = std::make_unique<OrthographicCamera>(
    5.0f,   // Size (half-width of view)
    1.0f,   // Aspect ratio
    0.1f,   // Near plane
    100.0f  // Far plane
);

// Position for top-down view
camera->lookAt(
    Eigen::Vector3f(0.0f, 0.0f, 10.0f), // Camera position (above)
    Eigen::Vector3f(0.0f, 0.0f, 0.0f),  // Target (origin)
    Eigen::Vector3f(1.0f, 0.0f, 0.0f)   // Up (X-axis points up)
);
```

### Fly Controller for Scene Exploration

```cpp
#include "Renderer/Camera/FlyController.hpp"

auto camera = std::make_unique<PerspectiveCamera>(45.0f, aspect, 0.1f, 100.0f);
auto controller = std::make_unique<FlyController>(*camera, 3.0f);

// Controls:
// - W/S: Forward/backward
// - A/D: Left/right
// - E/Space: Up
// - Q/Ctrl: Down
// - Right mouse drag: Look around
// - Shift: Move faster
// - Scroll: Adjust speed
```

## Controller Features

### OrbitController Settings

```cpp
controller->setTarget(Eigen::Vector3f(x, y, z));  // Set orbit center
controller->setDistance(5.0f);                     // Set zoom distance
controller->setSensitivity(0.3f);                  // Rotation sensitivity
controller->setZoomSensitivity(0.1f);              // Zoom sensitivity
controller->setPanSensitivity(0.01f);              // Pan sensitivity
controller->reset();                                // Reset to defaults
```

### FlyController Settings

```cpp
controller->setMoveSpeed(3.0f);          // Movement speed
controller->setLookSensitivity(0.1f);    // Look sensitivity
```

### DragController Settings

```cpp
controller->setPanSensitivity(0.01f);    // Pan sensitivity
controller->setZoomSensitivity(0.1f);    // Zoom sensitivity
controller->setTarget(target);           // Set look-at point
```

## Integration with Application

In your application class:

```cpp
class MyApplication : public OpenGLApplication
{
private:
    std::unique_ptr<PerspectiveCamera> m_camera;
    std::unique_ptr<OrbitController> m_controller;

    bool onSetup() override {
        m_camera = std::make_unique<PerspectiveCamera>(45.0f, aspect, 0.1f, 100.0f);
        m_controller = std::make_unique<OrbitController>(*m_camera);
        return true;
    }

    void onUpdate(float dt) override {
        m_controller->update(dt);
    }

    void onMouseButton(int button, int action, int mods) override {
        m_controller->handleMouseButton(button, action, mods);
    }

    void onCursorPos(double xpos, double ypos) override {
        m_controller->handleMouseMove(xpos, ypos);
    }

    void onScroll(double xoffset, double yoffset) override {
        m_controller->handleScroll(xoffset, yoffset);
    }

    void onKeyInput(int key, int scancode, int action, int mods) override {
        m_controller->handleKeyboard(key, action, mods);
    }
};
```

## Design Philosophy

This camera system is designed specifically for **robotics visualization**:

1. **OrbitController is primary** - Most users want to inspect a robot from all angles while keeping it centered
2. **Eigen-based** - Uses Eigen for all vector/matrix math (no GLM)
3. **Modular** - Easy to add new camera types or controllers
4. **Extensible** - Base classes for custom implementations
5. **Game-engine style** - Familiar patterns from Unity/Unreal/Godot

## Future Enhancements

Potential additions for robotics-specific features:

- **Focus on link/joint** - Automatically center camera on specific robot parts
- **Smooth transitions** - Interpolate between camera positions
- **Preset views** - Quick switch between common viewpoints (front, top, side, isometric)
- **Follow mode** - Camera tracks robot motion during animation
- **Collision detection** - Prevent camera from clipping through geometry
- **Multi-viewport** - Multiple cameras for split-screen technical views

## Files

```
Camera/
├── Camera.hpp/.cpp              # Abstract base camera
├── PerspectiveCamera.hpp/.cpp   # Perspective projection
├── OrthographicCamera.hpp/.cpp  # Orthographic projection
├── CameraController.hpp        # Controller interface
├── OrbitController.hpp/.cpp     # Orbit controller ⭐
├── FlyController.hpp/.cpp       # Fly controller
├── DragController.hpp/.cpp      # Drag controller
└── README.md                    # This file
```

## Notes

- All classes use **Eigen** for vector/matrix operations
- Cameras use **right-handed coordinate system**
- Up vector is configurable (default Y-up, but Z-up is common in robotics/URDF)
- Controllers are independent of rendering - they only update camera state
- Thread-safe: Each camera/controller is independent

---

For examples, see `src/RobotViewerApplication.cpp`

