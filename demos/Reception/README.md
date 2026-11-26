# 🛎 Reception

People walk, queue and talk in the hall of a reception robot, as the story of the scenario says ([The story of a scenario](../../doc/reference/story-file.md)). The robot, a LimX HU_D04 humanoid of the plugin HumanoidWalk, stands behind the desk and does nothing yet: [ROADMAP-HRI.md](../../ROADMAP-HRI.md) lists what it still needs.

## ▶ Run

```bash
make robotik_reception
./build/linux-gcc-debug/bin/RobotIK-Headless build/linux-gcc-debug/lib/robotik/plugins/Reception/scenarios/welcome.scenario.yaml --trace
./build/linux-gcc-debug/bin/RobotIK-Simulator build/linux-gcc-debug/lib/robotik/plugins/Reception/scenarios/welcome.scenario.yaml   # Space: play
```

## 🧍 The people

The woman, the man and the child are made with [MakeHuman](https://static.makehumancommunity.org) in Blender, by the add-on MPFB. Their bodies, skins, hair and clothes come from the asset packs of MakeHuman, under CC0. The files are not in git. Without them, the build copies `Soldier.glb` in their place.

To make them, install Blender and MPFB (Edit > Preferences > Get Extensions, search MPFB), then run:

```bash
make reception-people
```

The first run downloads the asset packs of MakeHuman (about 640 MB) into `~/.cache/robotik/mpfb-packs` and installs them in MPFB. It then writes `data/people/woman.glb`, `man.glb` and `child.glb`, in about 10 seconds each.

[people/people.json](people/people.json) describes each person: the sliders of MakeHuman (gender, age, height, weight, origin), the skin, the eyes, the eyebrows, the hair and the clothes, by the name of their folder in the packs. Each GLB has:

- the Mixamo skeleton of MPFB (52 joints);
- the 52 face units of ARKit (`eyeBlinkLeft`, `mouthSmileLeft`...) and the visemes as morph targets, for the expressions of a face;
- the clips Idle, Walk and Run of `Soldier.glb`, moved onto the new body by [people/make_people.py](people/make_people.py). Each bone turns in the world as its Mixamo bone does, from a rest where it points the same way.

## 🚧 Limits

- The clips come from a Mixamo character, so their license is not checked. The Idle clip is a soldier on guard, not a visitor who waits.
- The renderer does not show the morph targets yet: the faces keep the neutral expression.
