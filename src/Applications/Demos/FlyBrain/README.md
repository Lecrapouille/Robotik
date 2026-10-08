# Fly Brain demo

A fly (URDF scaled ×100) flies toward a food source while avoiding obstacles. The brain never sees MuJoCo, Compages, or the URDF: it reads an observation and returns an action (`forward`, `turn`, `lift`).

```text
sensors → observation → brain → action → controller → body
```

At startup, the simulator and `Robotik-Fly` build the 13-neuron circuit
(`FlyBrain::buildEmbedded`). The synapses (400, −650, 700, …) are written in
the code. The simulator's "Fly network" panel switches to the FlyWire
connectome. `Robotik-Fly` does the same with `--edges` and `--binding`.

The neuron is the one from Shiu et al. 2024
([Drosophila_brain_model](https://github.com/philshiu/Drosophila_brain_model)):
the same equations as `model.py`, the same constants, Poisson activation, and
silencing. The graph is FlyWire v783, extracted into `data/flywire/` by
`make compile-external-libs`. Those tables are not versioned.

`--brain reflex` has no neurons. It is a formula, used to check the
observation → action loop.

## From sensor to action

On every step (10 ms):

1. The three eyes, the altitude, and the food direction become an observation of 9 numbers.
2. The brain turns that observation into firing rates (hertz) on input neurons.
3. The network integrates those rates for 10 ms, in steps of 0.1 ms.
4. Spikes of the output neurons are smoothed, then mapped to an action: `forward` in [0, 1], `turn` and `lift` in [−1, 1].
5. The kinematic controller applies the action to the thorax and the wings. The network knows neither the joints nor the URDF.

A positive `turn` yaws left (+y). A zero `lift` holds altitude.

## The neuron

Each cell is a leaky integrate-and-fire neuron, with the constants from `model.py`:

| Constant | Value | Role |
|----------|-------|------|
| `v_rest`, `v_reset` | −52 mV | Resting potential, and the potential just after a spike |
| `v_th` | −45 mV | Threshold: above it, the cell spikes |
| `t_mbr` | 20 ms | Membrane leak |
| `tau` | 5 ms | Synaptic conductance leak |
| `t_rfc` | 2.2 ms | Refractory period after a spike |
| `t_dly` | 1.8 ms | Delay before a spike reaches its targets (18 steps of 0.1 ms) |
| `w_syn` | 0.275 mV | Weight of one synapse whose connectivity is 1 |

Two equations between spikes:

```text
dv/dt = (v_rest − v + g) / t_mbr
dg/dt = −g / tau
```

A spike resets `v` to −52 mV and `g` to 0, then adds `connectivity × w_syn` to the conductance of each target, 1.8 ms later. Positive connectivity is excitatory, negative connectivity is inhibitory. Shiu's column is named "Excitatory x Connectivity": that number, not a learned weight.

One presynaptic spike with a connectivity of about 400 is enough to push the target over threshold.

## Inputs and outputs

The brain talks to the body through thirteen roles. Eight are inputs, five are outputs. A role index is a neuron number: 0…12 in the embedded circuit, or a row of `Completeness_783.csv` when the connectome is loaded.

| Role | Direction | What drives it, or what is read from it |
|------|-----------|-----------------------------------------|
| `visual_left` | input | Left eye, 0…360 Hz |
| `visual_center` | input | Center eye, 0…360 Hz |
| `visual_right` | input | Right eye, 0…360 Hz |
| `seek_left` | input | Food to the left, up to 300 Hz |
| `seek_right` | input | Food to the right, up to 300 Hz |
| `too_low` | input | Below the food, ×220 Hz/m |
| `too_high` | input | Above the food, ×220 Hz/m |
| `bias` | input | 180 Hz all the time: cruise |
| `forward` | output | Smoothed rate / 140 → `forward` |
| `turn_left` | output | Compared with `turn_right` |
| `turn_right` | output | `(left − right) / 100` → `turn` |
| `lift_up` | output | Compared with `lift_down` |
| `lift_down` | output | `(up − down) / 80` → `lift` |

Input rates are capped at 400 Hz. Outputs go through an exponential average with a 50 ms time constant, so one 10 ms step cannot swing the action all the way.

In the embedded circuit the synapses are written by hand:

- `bias` excites `forward` (400), `visual_center` inhibits it (−650): cruise, unless something is straight ahead.
- The left eye excites `turn_right` (700) and inhibits `turn_left` (−300), and the right eye does the opposite: turn away from whatever is seen.
- `seek_left` / `seek_right` turn toward the food (420).
- `visual_center` also excites `turn_left` a little (220): head-on, with no winning side, the fly commits left.
- `too_low` climbs (400), `too_high` descends (400).

## Poisson

An input is not a constant current. It is a Poisson process, as in the optogenetic activation of Shiu's model.

On every 0.1 ms step, for an input neuron of rate `r` hertz, the number of kicks is drawn from a Poisson law of parameter `r × 0.0001`. Each kick adds `w_syn × f_poi` to the potential, with `f_poi = 250` (the constant from `model.py`, not the rate being commanded). One kick is therefore 0.275 × 250 = 68.75 mV before the membrane leak: more than enough to cross the 7 mV between rest and threshold. The rate `r` sets how many kicks arrive, not the size of one kick.

The scenario seed fixes those draws. Two runs with the same seed produce the same spikes.

Input neurons have no refractory period: they are stimulations, not cells that just fired. The others have one, of 2.2 ms.

## Activation and silencing

`model.py` changes a neuron in only two ways. The demo uses those two operations and no others.

**Activation.** The neuron is marked as a Poisson input (`addDrive`) and given a rate (`setRate`). This is the activation experiment: the cell is forced to receive random kicks. With no rate, an activated input does nothing. With a rate, it spikes and drives everything downstream in the graph.

That is what the demo uses for the eight input roles, including the embedded circuit. `bias` is a permanent activation at 180 Hz. The eyes, the food, and the altitude are activations whose rate follows the sensor.

**Silencing.** The neuron is held at rest. It no longer integrates, it does not spike, and it adds nothing to the conductance of its targets. This is the silencing experiment: the cell is removed from the circuit, and its targets no longer receive its spikes.

The mechanism is there (`silence INDEX` in the role file). The embedded circuit silences nobody: thirteen neurons, all of them used. Silencing is only for the connectome, to remove cells that should not fire while the inputs are activated.

Reading output rates is not a third manipulation. It is the measurement, as in the model: count the spikes of the neurons chosen as outputs.

## FlyWire connectome

The connectome is the FlyWire v783 synapse list: for each edge, the presynaptic index, the postsynaptic index, and "Excitatory x Connectivity". A neuron index is its row number in `Completeness_783.csv`, as in `model.py`.

The repository is listed in `external/manifest`. Extraction is in `external/compilation`:

```bash
make download-external-libs
make compile-external-libs
```

That writes `data/flywire/edges.csv` and `data/flywire/completeness.csv`. Both are in `.gitignore`: the parquet is about 100 MB, and the edge list is larger. `data/flywire/binding.txt` is versioned. It is the role file: which indices receive activation, which are read, and which are silenced.

The shipped numbers are real synapses from the graph, chosen because the weight is large enough for an activation to make the target fire. `bias` excites `forward`, so the thorax moves forward. The left eye excites `turn_right`, the right eye excites `turn_left`. `seek_left` and `seek_right` are a second synapse on the same turn: after an obstacle, the food direction steers. A repeated motor role sums the spikes. The completeness table does not name eyes or wings: these are not those cells. A role missing from the file is not wired.

A line `silence 50` removes neuron 50. Several such lines are allowed. A role absent from the file is simply not wired: no activation, or an output that stays at zero.

```bash
./build/Robotik-Fly \
    --edges data/flywire/edges.csv \
    --binding data/flywire/binding.txt \
    --headless
```

`--edges` without `--binding` is refused. Named inputs become Poisson activations. Named outputs are read. Neurons listed with `silence` are held quiet. The rest of the graph evolves on its own, with the model's equations.

## Running

With no option, the program is headless: one episode, the Shiu brain, the
scenario seed (123456). The exit code is 0 if the food was reached. The seed
fixes obstacle jitter, sensor noise, and the Poisson draws. Two runs with the
same seed play the same episode.

```bash
./build/Robotik-Fly --brain reflex
```

Same headless episode, but the explicit rule replaces the neural network. Use
it to check that the sensors → action → body loop reaches the food before
blaming the model.

```bash
./build/Robotik-Fly --view
```

Opens a window. The camera follows the thorax. Obstacles are brown, the food
is the gold sphere, the white crumbs are the path. Escape closes the window.
The brain stays Shiu, unless `--brain reflex` is added. Bottom left is the
left eye; bottom right is the right eye.

```bash
./build/Robotik-Fly --headless --envs 8 --episodes 8 --seed 123456
```

Eight worlds in parallel, no window, seed fixed. The program stops when eight
episodes have finished in total (not eight per world): as soon as one world
finishes, it starts another. Each world has a seed derived from 123456, so
the set is reproducible. The final line reports how many of those episodes
reached the food.

```bash
./build/Robotik-Fly --record fly_run.json
```

Plays one headless episode and writes `fly_run.json`: the seed, the time
step, then each observation (9 numbers) and the action that followed
(`forward`, `turn`, `lift`).

```bash
./build/Robotik-Fly --replay fly_run.json --view
```

Replays that file instead of querying the brain: the recorded actions are
applied in order, and the window shows the flight. One world. The file's seed
puts the obstacles back in the same place, so the body repeats the same path.

The graphical simulator shows the same scene, with orbit, pause, and
step-by-step. It starts on the 13-neuron circuit. The "Fly network" panel
switches to the connectome (`data/flywire/edges.csv` and `binding.txt`) and
restarts the episode. Both eyes are in the "Robot camera" panel.

```bash
./build/Robotik-Simulator data/scenarios/fly_obstacle_avoidance.yml
```

## Colored rays

Three horizontal rays start at the head, not at the center of the thorax.
Each one is 4 m long. The value sent to the brain is 1 if an obstacle touches
the eye, 0 if the ray reaches its end without a hit.

| Color  | Eye    | Direction                                      |
|--------|--------|------------------------------------------------|
| Yellow | left   | 0.55 rad left of the heading (+y)              |
| Red    | front  | straight ahead, between the two eyes           |
| Blue   | right  | 0.55 rad right of the heading                  |

The sphere is the start, sitting on the eye. The cone tip is the end: the
contact point on the obstacle, or the end of the 4 m if nothing is hit.
The closer the eye sees something, the thicker the cone and the larger the
end sphere.

## Eye cameras

`eye_L` and `eye_R` in the URDF have local +x as their optical axis (already
yawed outward by the joint). A camera is parented to each of those links,
4 cm in front of the eye, with a 70° vertical field of view. It follows the
head when the neck turns. In `--view`, the images are the two insets at the
bottom of the window. In the simulator, the "Robot camera" panel stacks them:
left eye, then right eye.
