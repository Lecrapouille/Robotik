---
name: robotik-docs
description: Edit or extend the RobotIK documentation in doc/ (numbered chapters 01–11, README index). Use when documenting a new feature, plugin, assertion, API change, or when asked to fix or rewrite docs.
---

# RobotIK documentation

Audience: people with no robotics background, often French. Write simple
English: short sentences, present tense, common words, no idioms or phrasal
verbs when a plain verb exists, define each robotics word once
(01-Introduction.md §2 is the glossary).

## Map (general → specific)

| File | Holds |
|------|-------|
| doc/README.md | index, reading order, quick paths |
| 01-Introduction.md | robotics glossary, what RobotIK is |
| 02-Getting-Started.md | build, simulator panels, headless output, exit codes |
| 03-Worlds-and-Scenarios.md | YAML tutorials + full reference (entities, assertions, experiments) |
| 04-Your-First-Plugin.md | HelloWorld lessons, plugin class, lifecycle, new plugin, packs, plugin.yaml |
| 05-Skills-and-Behavior-Trees.md | skills, scheduler, resources, BT nodes, fault tutorial |
| 06-Architecture.md | layers, step order, backends, truth/description/belief, source layout |
| 07-API-Reference.md | classes by task |
| 08-Reinforcement-Learning.md | environments, pool, log |
| 09-Demos.md | one section per plugin |
| 10-Tools-and-Frames.md | flange/tool0/tool_mount/tcp |
| 11-PDDL-Planner.md | planner controller, domain/observers/bindings, replanning, Planner panel |

Put a fact in one place and link to it (`[text](03-Worlds-and-Scenarios.md#6-assertions)`).
Root README.md has a doc table: keep it in sync when a chapter is added or renamed.

## Rules

- Every tutorial step and every quoted output must be run for real
  (`./build/Robotik-Headless ... --trace`) and pasted, not invented.
  Experiment on the `build/plugins/...` copy, then `make -C plugins/<Name>`
  restores it.
- Code excerpts come from real files (prefer HelloWorld); keep them in sync
  when the code changes.
- Each chapter: what you will learn → tutorial(s) → reference tables → next.
- Text diagrams in ```text blocks; tables for references.
- Do not document what does not exist; mark future work as such.
