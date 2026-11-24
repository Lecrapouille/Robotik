# 🌿 Give arguments to a skill from the tree

[⏫ Guides](README.md)

In a behavior tree, an `Action` node runs the skill that has its name, and nothing more: it cannot give an argument to the skill. Often the tree knows something that the skill needs. The Painter demo paints a portrait in four colours. The skills `dip` (put the brush in a pot of paint) and `mark` (paint a line) are the same for each colour, but the tree must tell them which colour. In this guide you write a tree node that reads a value in the tree, gives it to the skill, and runs the skill.

You need to know how a tree runs skills ([A skill and a behavior tree](../tutorials/06-skills-and-a-behavior-tree.md)).

## 📖 The words

- BlackThorn is the behavior tree library of RobotIK. A node of the tree is a C++ class of BlackThorn. The node factory (`bt::NodeFactory`) creates the nodes from their names in the `.bt.yaml` file.
- The blackboard is a table of named values that the nodes of a tree share. A `SubTree` node can give parameters to its subtree: they go on the blackboard of the subtree.
- `RobotIK/Task/Skills/SkillNodes.hpp` is the only RobotIK header that includes BlackThorn. The other headers only name the tree types. A plugin that writes no node never includes BlackThorn.

## 🌳 1. Give the value in the tree

`plugins/Painter/scenarios/painter.bt.yaml` writes the steps of one colour once, in the subtree `Layer`. Each `SubTree` node runs `Layer` with its own `color` parameter:

```yaml
SubTrees:
  Layer:
    Sequence:
      name: Layer
      children:
        - Action:
            name: Dip
        - Action:
            name: Home
        - Action:
            name: Mark
        - Action:
            name: Home
        - Action:
            name: Rinse
        - Action:
            name: Home

BehaviorTree:
  Sequence:
    name: Portrait
    children:
      - Action:
          name: Home
      - SubTree:
          name: Red
          reference: Layer
          parameters:
            color: red
      - SubTree:
          name: Black
          reference: Layer
          parameters:
            color: black
      # ... then blue and gold
```

`Dip`, `Mark` and `Rinse` are not the names of skills (the skills are `dip`, `mark` and `rinse`). They are the names of nodes that the plugin registers, in step 3.

## 🧩 2. Write the node

The node of Painter is a BlackThorn leaf. At its first tick, it reads `color` on the blackboard, gives it to the skill, and requests the skill. At the next ticks, it answers the state of the skill (`plugins/Painter/src/PainterSkills.cpp`, shortened):

```cpp
bt::Status onTick()
{
    if (m_run == robotik::NO_RUN)
    {
        auto const color = blackboard()
                               ? blackboard()->get<std::string>("color")
                               : std::nullopt;
        if (!color || color->empty())
        {
            return bt::Status::FAILURE;
        }
        dynamic_cast<DipSkill&>(m_scheduler.skill(m_id)).use(*color);
        m_run = m_scheduler.request(m_id);
        return bt::Status::RUNNING;
    }
    switch (m_scheduler.state(m_run))
    {
        case robotik::SkillState::Idle:
        case robotik::SkillState::Waiting:
        case robotik::SkillState::Running:
        case robotik::SkillState::Stopping:
            return bt::Status::RUNNING;
        case robotik::SkillState::Succeeded:
            m_run = robotik::NO_RUN;
            return bt::Status::SUCCESS;
        case robotik::SkillState::Failed:
        case robotik::SkillState::Cancelled:
            m_run = robotik::NO_RUN;
            return bt::Status::FAILURE;
    }
    return bt::Status::FAILURE;
}
```

The node does not move the robot: it requests the skill, and the scheduler runs it with its resources, as for an `Action`. The node keeps the ticket (`RunId`) that `request` returns, and reads the state of its own run with it. When the tree stops the node (`on_reset`), the node cancels its own run with `m_scheduler.cancel(m_run)`: another node of the same skill keeps its run.

## 📝 3. Register the node

`registerNodes` gives each node name a function that creates the node:

```cpp
void registerNodes(bt::NodeFactory& p_factory,
                   robotik::SkillScheduler& p_scheduler)
{
    auto bind = [&](char const* p_node, char const* p_skill, Gesture p_gesture)
    {
        p_factory.registerNode(
            p_node,
            [&p_scheduler, name = std::string(p_skill), p_gesture]()
            {
                return std::make_unique<GestureNode>(
                    p_scheduler, p_scheduler.find(name), p_gesture);
            });
    };
    bind("Dip", "dip", Gesture::Dip);
    bind("Mark", "mark", Gesture::Mark);
    bind("Rinse", "rinse", Gesture::Rinse);
}
```

The scenario logic calls it in `setup`, after it adds the skills, with the factory of the cell:

```cpp
painter::registerNodes(p_cell.factory(), skills);
```

The cell builds the tree after `setup`, so the factory knows the nodes when it reads the `.bt.yaml` file.

## ✅ 4. Check the result

Run the Painter scenario with `--trace`:

```bash
./build/linux-gcc-debug/bin/RobotIK-Headless build/linux-gcc-debug/lib/robotik/plugins/Painter/scenarios/painter.scenario.yaml --trace
```

```text
  0 s  Home  SUCCEEDED  (0 s)
  0.01 s  dip  SUCCEEDED  (1.84 s)
  1.86 s  Home  SUCCEEDED  (0.66 s)
  2.53 s  mark  SUCCEEDED  (7.11 s)
  9.65 s  Home  SUCCEEDED  (0.77 s)
  10.43 s  rinse  SUCCEEDED  (3.83 s)
  14.27 s  Home  SUCCEEDED  (0.99 s)
  15.27 s  dip  SUCCEEDED  (1.99 s)
  ...
```

The same skills `dip` and `mark` run once per colour, with the colour of their subtree. If a node finds no `color` on its blackboard, it fails, and the `Sequence` above it fails too.

## 🔗 See also

- [The Painter case study](../case-studies/painter.md): the whole demo.
- [Skills, the scheduler and behavior trees](../concepts/04-skills-and-behavior-trees.md): the other nodes of BlackThorn.
