# 👁 Perception and beliefs

[⏫ API reference](README.md)

Headers: `RobotIK/Task/Perception/Detector.hpp`, `Detection.hpp`, `WorldModel.hpp`, `RobotIK/Robot/Sensors/Camera.hpp`.

**Write a detector:**

```cpp
class RedBlobDetector final: public robotik::Detector
{
public:
    void detect(robotik::CameraFrame const& p_frame, robotik::Detections& p_out) override
    {
        // read p_frame.rgb, p_frame.depth, p_frame.intrinsics, p_frame.pose; push what you find:
        robotik::Detection d;
        d.label = "red_cube";
        d.confidence = 0.9f;
        d.center = { 120.0f, 80.0f };          // pixels
        // d.position = ... (optical frame, metres) if you know the depth
        p_out.items.push_back(d);
    }
};

// in ScenarioLogic::setup:
p_cell.perception().add<RedBlobDetector>();
```

`Detection`: `label`, `id` (fiducial id or -1), `confidence`, `box` (pixels), `center` (pixels), `position` (optional, optical frame), `pose` (optional).

**Read beliefs** (in a skill, through `p_context.world_model`):

| `WorldModel` | Does |
|--------------|------|
| `find(name)` | `WorldObject*` or null |
| `objects()` | All beliefs |
| `observe(name, point, confidence, stamp)` | Update a belief |
| `add(name, position, size)`, `place(...)` | Declare / move a belief |

`WorldObject`: `name`, `position`, `size`, `confidence`, `seen` (time), `observations`, `observed()`, `top()`.

Other sensors (`RobotIK/Robot/Sensors/`): `Imu`, `RangeScanner`, `ForceTorqueSensor`. All derive from `Sensor` (`name()`, `frequency()`, `samples()`, `stamp()`), and their readings derive from `Measurement` (time stamp, frame, sequence, validity). A skill reads them through `Robot::getSensors()`, read only; the runtime adds them (`RobotRuntime::addSensor`) and sets their image source and noise seed.
