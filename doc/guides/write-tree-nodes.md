# 🌿 Write your own tree nodes

[⏫ Guides](README.md)

By default, actions are named after the skills. A plugin can also register
its own BlackThorn nodes, for example to give arguments to a skill before
requesting it. The Painter does this (`painter::registerNodes`):

```cpp
p_factory.registerNode("Dip", [&p_scheduler]() {
    return std::make_unique<GestureNode>(p_scheduler, p_scheduler.find("dip"), Gesture::Dip);
});
```

Call it from `ScenarioLogic::setup` with `p_cell.factory()`. Include
`Robotik/Task/Skills/SkillNodes.hpp` first: it is the only RobotIK header
that brings BlackThorn. The rest of the API names the tree types without
their definition, and `RoboticCell::status()` answers a `robotik::Status`
(`IDLE`, `RUNNING`, `SUCCESS`, `FAILURE`). An application that does not
write nodes never sees BlackThorn.
