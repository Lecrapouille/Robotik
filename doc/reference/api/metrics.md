# 📊 Metrics

[⏫ API reference](README.md)

Header: `RobotIK/Task/Metrics.hpp`.

```cpp
void measure(robotik::RoboticCell const&, robotik::Metrics& p_metrics) const override
{
    p_metrics.set("laps", m_laps);
    p_metrics.set("cross_track.max", m_max_error);
}
```

Any published name can be used in an assertion (`(>= laps 1)`) or in `evaluation.metrics` of an experiment. Use a prefix per plugin (`hello.ticks`, `reach.error`) to avoid clashes.
