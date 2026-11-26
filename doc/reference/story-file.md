# 🎭 The story of a scenario

[⏫ Reference](README.md)

A robot that works among people is tested with people around it. The **story** says what these people do during a run: a visitor walks to the desk, turns to the robot, says why she is here, waits, then goes to the meeting room. A story is data in the scenario file, like the assertions: no C++ moves the people. The idea comes from the storyboard of [ASAM OpenSCENARIO](https://www.asam.net/standards/detail/openscenario/), which tests cars in traffic.

The people are **actors**: entities of type `actor` in the [world file](world-file.md#-actors). The story moves them in simulated time. It runs headless too: without a view, an actor is a point that moves, and every condition and assertion works the same. The demo [Reception](../../demos/Reception/scenarios/welcome.scenario.yaml) uses most of the keywords of this page.

```yaml
scenario:
  id: welcome
  world: worlds/hall.world.yaml
  story:
    places:
      counter: [1.6, 0.0]
      meeting_room: [2.0, 6.0]
    variables:
      registered: 0
    events:
      - id: arrive
        actor: woman
        start: (> time 1)
        actions:
          - walk_to: counter
          - turn_to: robot
          - say: { id: hello, text: "Hello, I have a meeting at ten.", to: robot }
      - id: to_meeting
        actor: woman
        after: arrive
        actions:
          - add: { registered: 1 }
          - walk_to: meeting_room
          - hide: true
  assertions:
    - { condition: (said woman hello), eventually: true }
```

## 🗂 The story section

| Key | Value | Default |
|-----|-------|---------|
| `places` | Named points of the floor, `name: [x, y]` or `[x, y, z]` (m, world frame). A target may name them | none |
| `variables` | Story variables and their value at the start, `name: number`. Actions `set` and `add` change them; conditions read them with `(story_var name)` | none |
| `events` | The list of events, in the order they are checked at each step | none |

## 🎬 Events

An event is a list of actions that one actor plays when a condition becomes true.

| Key | Value | Default |
|-----|-------|---------|
| `id` | Unique name, without space, `(` or `.`. Conditions name it: `(event_done arrive)` | required |
| `actor` | The actor that plays it. Without an actor, the event may only use `wait`, `wait_until`, `set`, `add`, `start`, `stop`, `choose` | none |
| `start` | The condition that starts it ([conditions](scenario-file.md#-conditions)) | none: it starts at once |
| `after` | An event or a list of events that must be done (once at least) before it starts | none |
| `delay` | Seconds between the start and the first action | `0` |
| `repeat` | How many times it may run: a number, or `forever` | `1` |
| `interrupt` | `true` cancels the event its actor plays, instead of waiting for it | `false` |
| `stop_when` | A condition that cancels it while it runs | none |
| `actions` | The actions, played one after the other ([actions](#-actions)) | required |

### ⏱ When an event starts

An event starts on the **rising edge** of `start`: the condition was false at the step before, and it is true now. A condition that is true at the first step counts as a rising edge. An event without `start` starts at the first step.

A rising edge arms the event. The event then starts as soon as every event of `after` is done and its actor is free. It does not need a new edge for that: `start: (> time 1)` with `after: check_in` starts when `check_in` ends, even at 20 s.

When the event ends and `repeat` allows another run, it waits for the next rising edge of `start`. An event without `start` runs again at once. For example, `repeat: 3` without `start` plays the actions three times in a row.

### 👤 One event per actor

An actor plays one event at a time. When a second event of the same actor starts, it waits until the first one ends (the phase is `pending`). With `interrupt: true`, the second event cancels the first one and starts at once. A cancelled event counts as failed: `(event_failed e)` becomes true.

### 🔚 How an event ends

| End | What happens | Counts in |
|-----|--------------|-----------|
| Done | The last action ended | `(event_done e)`, `(event_count e)` |
| Failed | An action passed its `timeout` | `(event_failed e)` |
| Cancelled | `stop_when` became true, another event used `stop`, or an event with `interrupt` took the actor | `(event_failed e)` |

A failed event does not stop the run. If a failure must fail the test, write an assertion: `{ condition: (not (event_failed man_queues)), phase: always }`.

## 🏃 Actions

An action is a map with one keyword. Most keywords have a short form (a value) and a long form (a map).

```yaml
- walk_to: counter                                   # short form
- walk_to: { target: counter, speed: 0.8, timeout: 20 }   # long form
```

A **target** is a place of the story, an entity (`robot`, `desk`, `woman`), a frame of an entity (`robot.tool0`, `woman.head`), or a point `[x, y]`. The story reads the true pose of an entity at each step, so a moving target is followed.

### 🚶 Moves

| Keyword | Short form | Long form keys | Ends when |
|---------|------------|----------------|-----------|
| `walk_to` | a target | `target`, `speed`, `gait`, `distance`, `timeout` | The actor is `distance` from the target. Without `distance`: 0.1 m from a place or a point, 0.8 m from an entity |
| `run_to` | a target | the same; the gait is `run` | the same |
| `follow_path` | a list of targets | `points`, `loop`, `speed`, `gait`, `timeout` | The last point is reached (0.15 m). With `loop: true`, it starts again at the first point and does not end |
| `follow` | a target | `target`, `distance` (default 1.0 m), `until`, `speed`, `gait`, `timeout` | The condition `until` is true, or `timeout` passed. It walks when the target is 0.25 m farther than `distance`, and stops at `distance` |
| `turn_to` | a target | `target`, or `heading` (rad, about Z, 0 faces +X) | The actor faces the target within 0.03 rad |
| `teleport` | none | `position: [x, y, z]`, `rpy: [r, p, y]` (or `quaternion`) | At once. Only the yaw of the rotation is used |

`gait` is `walk` (the default) or `run`. The speed is `walk_speed` or `run_speed` of the actor, unless `speed` gives it in m/s.

An actor walks on arcs. It turns at most at `turn_rate`, and on a circle of `turn_radius` at least. While it turns, it slows down to 45 % of its speed, never less. A person does not spin on the spot, and the feet of the model do not slide. Near its goal (closer than two turn radii), it turns at `turn_rate` so that it does not circle round the goal.

For `walk_to`, `run_to` and `follow_path`, a `timeout` passed fails the event. For `follow`, a `timeout` ends the action normally.

### ⏳ Time

| Keyword | Short form | Long form keys | Ends when |
|---------|------------|----------------|-----------|
| `wait` | seconds, or `[min, max]` | none | The time passed. `[min, max]` draws a time from the seed at each run |
| `wait_until` | a condition | `condition`, `timeout` | The condition is true. A `timeout` passed fails the event |

### 💬 Speech, clips and face

| Keyword | Short form | Long form keys | Ends when |
|---------|------------|----------------|-----------|
| `say` | the text | `id`, `text`, `to`, `duration` | `duration` passed. Without it: one second for 2.5 words, one second at least |
| `play` | a clip role | `clip`, `duration` (default 2 s), `loop` | `duration` passed |
| `expression` | a name | `name`, `weight` in [0, 1] (default 1), `duration` | At once. The face keeps the expression for `duration`, or until the next one when there is no `duration` |

`say` records an **utterance**: who said it, its `id`, the text, whom it was said to, and when. Without an `id`, the id is `<event>_<number of the action>`. While it speaks, the actor plays its `talk` clip and turns to `to`. A view writes the text above the head. `(said woman hello)` is true from the start of the sentence. No robot hears it yet: see [ROADMAP-HRI.md](../../ROADMAP-HRI.md).

A **clip role** names a clip of the model: `play: wave` plays the clip that the `clips:` map of the actor gives for `wave`, or the clip named `wave` when the map has no entry. The story itself chooses the roles `idle`, `walk`, `run` and `talk`. A clip that the model does not have is written once in the terminal, and the actor plays nothing.

An `expression` needs a model with morph targets. The renderer does not read them yet, so a view shows nothing; the query `(expression a name)` works headless.

### 🎚 The story itself

| Keyword | Value | What it does |
|---------|-------|--------------|
| `show` | `true` or `false` | The actor enters or leaves the world. A view hides it; `(shown a)` reads it |
| `hide` | `true` or `false` | The opposite of `show` |
| `set` | `{ variable: number }` | Gives a value to a declared variable |
| `add` | `{ variable: number }` | Adds a number (negative to subtract) to a declared variable |
| `start` | an event or a list | Starts waiting events now, without their `start` condition and without `after`. Their actor must still be free |
| `stop` | an event or a list | Cancels running or pending events, and disarms waiting ones |
| `choose` | a list of `{ weight, actions }` | Draws one branch from the seed, with a chance in proportion to its `weight` (default 1), and plays its actions |

Every action of this table ends at once, and the next action starts in the same step.

## ❓ Queries of the story

Conditions of the story, `stop_when` and the assertions may ask these queries. The arguments are checked at load: a wrong event, actor or variable stops the load with its name.

| Query | Answer |
|-------|--------|
| `(event_started e)` | 1 when `e` started once at least |
| `(event_running e)` | 1 while `e` waits its `delay` or plays its actions |
| `(event_done e)` | 1 when `e` played all its actions once at least |
| `(event_failed e)` | 1 when `e` failed or was cancelled once at least |
| `(event_count e)` | How many times `e` played all its actions |
| `(story_var name)` | The value of a story variable |
| `(said who id)` | 1 when the entity `who` said the sentence `id` |
| `(speaking a)` | 1 while the actor `a` speaks |
| `(shown a)` | 1 while the actor `a` is in the world |
| `(actor_speed a)` | The speed of the actor `a` now (m/s) |
| `(expression a name)` | The weight of the expression `name` on the actor `a`, 0 when it has another one |
| `(at a target [tolerance])` | 1 when the entity `a` is within `tolerance` (default 0.5 m) of the target, measured on the floor. The target may be a place |
| `(heading_error a target)` | The angle (rad, in [0, π]) between where the entity `a` faces (its +X axis) and the direction of the target |

`(distance a b)` and the other queries of the world work with actors too ([conditions](scenario-file.md#-conditions)).

## 🎲 Seeds

The same seed gives the same story, step for step. The seed draws the times of `wait: [min, max]`, the branches of `choose`, and the `randomize` offsets of the actors in the world file. An experiment with several seeds therefore plays several variants of one story ([The experiment file](experiment-file.md)).

## 📋 What the run reports

`RobotIK-Headless --trace` prints the log of the story before the verdicts. The JSON report (`--report`) has it as a `story` list.

```text
  story  1.00 s arrive: started
  story  1.00 s child_follows: started
  story  3.01 s man_queues: started
  story  6.29 s woman says 'Hello, I have a meeting with Ms Martin at ten.' to robot
  story  10.30 s arrive: done
  story  10.30 s check_in: started
  ...
  story  19.23 s man_leaves: chose branch 1
```

From C++, `RoboticCell::story()` gives the `Story`: `log()`, `utterances()`, and the answers of the queries above.

## 🐞 Load errors

Every error names the event and the action, as in `story, event 'a', action 1 (jump): unknown action 'jump' (known: walk_to, run_to, ...)`.

| Message | Cause |
|---------|-------|
| `unknown action 'x' (known: ...)` | A keyword that is not in the tables above |
| `unknown key 'strat' (did you mean 'start'?)` | A key of an event or of a long form that does not exist |
| `actor 'desk' is no entity of type actor` | `actor:` names an entity that is not an actor |
| `'nowhere' is no place nor entity` | A target that is neither a place nor an entity of the world |
| `this action needs the 'actor:' of the event` | A move, a `say` or a `play` in an event without actor |
| `variable 'x' is not declared in story.variables` | `set` or `add` on an unknown variable |
| `story, event 'a', start: ...` | The condition does not compile (unknown metric, query or entity) |

## 🚧 Limits

- An actor walks in a straight line to its goal: there is no path planning, so it walks through walls, desks and robots.
- An actor has no physics: a robot passes through it, and a lidar or a contact sensor does not see it.
- No robot hears what an actor says, and no robot can speak yet.
- The `expression` action has no effect on the picture until the renderer reads morph targets.

[ROADMAP-HRI.md](../../ROADMAP-HRI.md) lists what is planned.

## 🔗 See also

[The world file: actors](world-file.md#-actors) · [The scenario file](scenario-file.md) · [Glossary](glossary.md)
