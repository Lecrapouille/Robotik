# 8. Reinforcement learning

Robotik does **not** train policies. It gives stable, reproducible,
window-free **environments** that a learning algorithm (in C++, or in another
process) can drive. The same worlds and scenarios as the demos are used.

If the words *observation*, *action*, *reward*, *episode* are new, read
[01-Introduction.md §2](01-Introduction.md#reinforcement-learning-rl) first.

---

## 1. The loop

```text
          ┌──────────── observation ◄──────────┐
          ▼                                    │
       policy ──► action ──► simulation ──► reward
```

`robotik::Environment` (`include/Robotik/Task/Learning/Environment.hpp`):

| Operation | Does |
|-----------|------|
| `reset(seed, observation)` | Starts a reproducible episode, writes the first observation |
| `step(action, observation)` | Applies the action for a fixed time, advances the simulation, writes the next observation, returns the reward and the end, if any |
| `observationSpace()`, `actionSpace()` | Layout of the vectors: name, bounds, unit of each dimension |

Observations and actions are flat `float` vectors, written in place: a step
allocates nothing.

### Spaces

```cpp
robotik::Space observation;
observation.add3("tip", -1.5f, 1.5f, "m")            // tip.x, tip.y, tip.z
           .add("holding", 0.0f, 1.0f)
           .add("delivered", 0.0f, 1.0f, "", true);  // privileged
```

A **privileged** dimension comes from the simulator ground truth: a real robot
could not observe it. A policy that uses it is not deployable as is.
`Space::privileged()` says so, and the log records it.

### How an episode ends

`StepResult::end` has four values, because learning and evaluation treat them
differently:

| `EpisodeEnd` | Meaning | Terminal state? |
|--------------|---------|-----------------|
| `Success` | The task is done | yes |
| `Failure` | The task cannot be done any more | yes |
| `TimeLimit` | The step budget is used up | no: the value of the next state still counts |
| `Aborted` | Technical stop (solver, invalid state) | no: the episode says nothing about the policy |

`terminated()` groups the first two, `truncated()` the last two.

---

## 2. An environment from a scenario

`robotik::SimulationEnvironment` turns a robot cell into an environment (a
scenario file, or a `CellDescription` written in C++), from three swappable
parts:

```text
           action                          observation
              │                                 ▲
              ▼                                 │
       ActionAdapter                    ObservationBuilder
   (vector → commands)             (sensors, beliefs → vector)
              │                                 ▲
              ▼                                 │
           Cell ── held for control_period ──┘
              │
              ▼
       RewardEvaluator ── reward, Success / Failure
```

| Part | Role |
|------|------|
| `ObservationBuilder` | What the agent may know. Prefer sensors and the `WorldModel`; flag what comes from the ground truth |
| `ActionAdapter` | Turns the vector into commands (joint targets, gripper...). Does not advance time |
| `RewardEvaluator` | Scores the step from the simulation only, never from the display. Says when the task is over |

Each part has `setup` (once) and `reset` (each episode). Changing the
observation (joints only, or camera) touches neither the action nor the
reward.

```cpp
robotik::SimulationEnvironment::Parts parts;
parts.observation = std::make_unique<PickPlaceObservation>();
parts.action = std::make_unique<PickPlaceAction>();
parts.reward = std::make_unique<PickPlaceReward>();

robotik::EnvironmentTiming timing;
timing.physics_step = Seconds(0.01);
timing.control_period = Seconds(0.1);   // one action held for 10 physics steps
timing.max_steps = 100;                 // then TimeLimit

robotik::SimulationEnvironment environment(
    robotik::Scenario::load(path, assets), std::move(parts), timing);
```

The scenario keeps its world, its faults and its perception policy. Its
controller, `timeout`, `stop_when` and assertions are removed: the agent
drives, the reward judges. An exception in a part or in the simulation ends
the episode as `Aborted`; the reason is in `abortReason()`.

`reset(seed)` rebuilds the full initial state: joints, base, sensors (random
streams included), resources, faults, object layout.

The three pick-and-place parts live in `plugins/PickAndPlace/src/RlMission.*`
and are shared by the command-line pool and the simulator plugin.

---

## 3. Many environments

`robotik::EnvironmentPool` runs N environments on threads, behind contiguous
buffers ready for a learner. Episode `k` of environment `i` uses the seed
`master.derive(i).derive(k)`: any episode can be replayed alone, and the
result does not depend on the number of threads.

```bash
./build/Robotik-PickAndPlaceRL --policy converged --envs 16 --threads 8 --episodes 32 \
    --log episodes.json
```

The command then replays the same batch on one thread and checks that the
results are identical, bit for bit. The last column gives the end of each
episode: `success`, `failure`, `time limit` or `aborted`.

### The log

`robotik::EpisodeLog` writes, in JSON, what is needed to compare and replay:
Robotik version and commit, scenario, policy, master seed, control period,
step budget, both spaces (privileged dimensions included), then each episode:
environment, index, seed, return, number of steps, end.

---

## 4. Environments and agents

Two different ideas:

| | What | Why |
|--|------|-----|
| **Many environments** (`env0` … `envN`) | Independent simulations, each with its world and seed | Collect faster, compare conditions, statistics |
| **Many agents** in one environment | Entities acting in the same world | Coordination, collective behaviour |

An agent is not always learned: an existing neural model (the fly brain), a
classic controller or a trained policy all share the same surface,
`robotik::Agent` (`include/Robotik/Task/Agents/Agent.hpp`): it receives an
observation and returns an action.

Today `SimulationEnvironment` drives one robot. Several agents in one shared
world are still to do.

---

## 5. Checklist

| Question | Answer in Robotik |
|----------|-------------------|
| Does a reset put every state back? | Yes: `Cell::reset` (robot, sensors, resources, faults, objects), then `reset` of each part. Checked by a replay test |
| Explicit observation and action spaces? | `Space`: name, bounds, unit per dimension |
| Observations consistent with the available sensors? | Pick-and-place reads the robot beliefs; the only ground-truth dimension is flagged privileged |
| How long is an action applied? | `EnvironmentTiming::control_period`, a whole multiple of the physics step |
| Reward independent of the display? | `RewardEvaluator` sees only the simulation; no environment has a window |
| Natural end distinct from truncation and technical stop? | `EpisodeEnd`: `Success`, `Failure`, `TimeLimit`, `Aborted` |
| Experiments without GUI? | Yes: pool, headless, tests |
| Compare seeds and policies? | Seeds derived per environment and per episode; `EpisodeLog` in JSON |

### Still to do

- A stable bridge to a Python process (C API over the pool, or bindings).
- Several agents in one environment.
