# 🤝 Roadmap: a reception robot among people

What is missing for the demo of a humanoid reception robot: it greets the
visitors, registers them and guides them. One line per step: tick it, or
remove it, in the commit that does it. Done work is in the git history and
the documentation, not here.

The rule for every item: the core stays agnostic. A person detector, a
speech channel or a dialogue skill is a generic concept; the hall, the desk
and the meeting rooms stay in `demos/Reception/`. Each perception step
comes first as a mock of the detector (`perception.fidelity: truth`, said
in the run report), then as the real detector on the camera images.

## 🎭 The people of the simulation

Done: actors in the world file, the story of a scenario
(doc/reference/story-file.md), the demo `demos/Reception`, clips that fade
and feet that do not slide (Compages `play(model, clip, fade)`,
`groundSpeed`).

- [x] Models of people made with MakeHuman (MPFB 2 in Blender, CC0):
      `demos/Reception/people/make_people.py` makes a woman, a man and a
      child with clothes, the Mixamo rig of MPFB (52 joints), the 52 face
      units of ARKit and the visemes as morph targets, and moves the clips
      of Soldier.glb onto them (`make reception-people`). Skin, hair and
      clothes come from the system assets of MakeHuman only (280 MB)
- [x] Compages: cut the transparent parts of a glTF material (cornea,
      hair, brows and lashes)
- [x] Compages: keep the materials and skins of each glTF file apart (two
      MPFB people both have `Human.body` and `Human.rig`)
- [ ] Clips under a free license, for a visitor: the Idle of Soldier.glb is
      a soldier on guard, its license is not checked. The Universal
      Animation Library of Quaternius (CC0) retargeted by the same script
      (idle, walk, run, talk, wave, point, sit)
- [ ] More people drawn from a seed: random sliders, skins, hair and
      clothes in people.json (the character builder of MPFB). More clothes
      and skins: the packs dropped into `RECEPTION_PACKS` are installed too
- [ ] Compages: the shadows and the transparent parts (a hair card casts
      a full rectangle today)
- [ ] Compages: morph targets (glTF loader, vertex shader, weights driven by
      the clips and by code): the `expression` action shows on the face
- [ ] Compages: a visibility switch on an entity (today `hide` scales the
      model to zero)
- [ ] Compages: clips in layers (wave with the arms while the legs walk),
      a turn on the spot, the head that looks at a target (`look_at` action)
- [ ] Actors in the physics: a kinematic capsule per actor in MuJoCo, so
      that a lidar sees them and a contact with the robot is a collision
- [ ] Actors that go round obstacles: a path on a grid of the floor, then
      the distances people keep between them (a queue, a crowd)
- [ ] A crowd made by drawing: N visitors from a template event, each with
      its own seed (`spawn:` in the story)

## 👀 What the robot perceives

- [ ] A robot file (`hu_d04_01.robot.yaml`, a new file role) with all the
      sensors and actuators of the real robot, as in its datasheet: the
      two D435 (head, waist), the IMU, the joint encoders, the microphones,
      the speaker, the joint groups. A world takes it by its path
      (`robot: robots/hu_d04_01.robot.yaml`) and changes only what differs
      (a sensor off, a noise). Today each world copies the sensors of its
      robot. A robot file is generic: the core reads it, the demos keep
      their own robots
- [x] The head camera of the real robot: the RealSense D435 of the LimX
      URDF (`head_camera_joint`), RGB-D, 69×42°, 1280×720 at 30 Hz, in the
      optical frame
- [ ] The second camera of the real robot: the D435 of the waist
      (`waist_camera_joint`, 35° down), for the floor and the near people
- [ ] Noise of the D435 depth: the error grows with the square of the
      distance (about 2 % at 2 m), holes on dark and shiny surfaces
- [ ] Check the face resolution: with 720 px over 42°, a face 15 cm wide
      is 64 px at 2.2 m and 18 px at the door (8 m). Detect the people far
      away, the faces only near the desk; the head turns (`head_pitch`,
      `head_yaw`) to look at a person
- [ ] A fixed camera of the hall (a corner, high up), for the view of the
      operator and the labels of the training images. It is not a sensor
      of the robot: its images never go into the beliefs
- [ ] People in the beliefs: the `WorldModel` holds persons (id of track,
      pose, speed, last time seen). First from the truth (`fidelity:
      truth`, `noisy_truth` with missed detections), then from a detector
- [ ] A person detector on the camera images (YOLO, ONNX Runtime as in
      HumanoidWalk), with the true boxes of the actors as labels: the
      simulator makes its own training data
- [ ] Faces: detection, an embedding to tell one person from another
      (ArcFace), the expression (a classifier). Needs the MakeHuman models
      with their face units, rendered at a resolution where a face is 64
      pixels wide at least
- [ ] A microphone sensor: it hears the utterances said within its range.
      First a perfect transcript (a mock of speech recognition), then noise,
      missed words, then whisper.cpp on synthetic speech
- [ ] Assertions on what the robot believes of the people
      (`(belief_error woman)`, `(believed (registered woman))`)

## 🧰 The software stack

Each block is behind a sensor, an actuator or a skill, with a mock first,
so that a test runs the same at each run and without the block installed.
A library used by the demo only goes in `dependencies:` of
`demos/Reception/plugin.yaml`; a generic block (speech channel, LLM
client) goes in the core only when a second plugin needs it.

- [ ] Speech to text: whisper.cpp (MIT), behind the microphone sensor.
      Mock: the transcript of the story
- [ ] Text to speech: Piper (MIT, ONNX voices, offline), behind the
      speaker actuator. Mock: the text in the list of utterances
- [ ] LLM: the local Qwen, served by llama.cpp (`llama-server`, an
      OpenAI-compatible HTTP API with tool calls). The client and the
      model name are in a file of the scenario, not in C++. Mock: a
      scripted LLM (questions and answers in YAML). Check that the Qwen
      model answers tool calls in the format of llama.cpp
- [ ] JEV: check its SDK contract first (what it takes, what it gives,
      its license), then choose where it plugs in
- [ ] The person detector and the faces: ONNX Runtime, as in HumanoidWalk
      (YOLO, ArcFace)

## 🗣 What the robot says and does

- [ ] A speaker actuator and a `Say` skill: the robot speaks into the same
      list of utterances as the actors, so that `(said robot follow_me)`
      can start an event of the story
- [ ] A dialogue skill with an LLM and function calling. The LLM proposes,
      the skills act: each tool it may call is a skill (look up a meeting,
      register a visitor, guide to a room, call the host). A scripted LLM
      (questions and answers in YAML) keeps the tests the same at each run
- [ ] A calendar and a list of visitors as a service of the plugin, with a
      file for the tests
- [ ] Navigation of the robot: a mobile base first (path on a map of the
      hall), then the walking humanoid (the policy of HumanoidWalk)
- [ ] Guide a visitor: the robot walks to a room and checks in its beliefs
      that the person follows (the actor uses `follow: robot`)
- [ ] A PDDL domain of the reception: greet, identify, register, guide,
      notify, with observers on the beliefs of the people
- [ ] Metrics of good manners: distance kept to people, waiting time of a
      visitor, time to answer; assertions on them in the scenarios

## 🎬 The demo, step by step

- [x] People come, queue, talk and leave (story only, the robot does
      nothing): `demos/Reception/scenarios/welcome.scenario.yaml`
- [ ] The robot turns to the person who comes in (people in the beliefs,
      a skill that turns the base)
- [ ] The robot says hello and the visitor answers (speaker, microphone,
      a scripted dialogue)
- [ ] The robot registers the visitor from the calendar (dialogue skill,
      function calling)
- [ ] The robot guides the visitor to the meeting room
- [ ] The same, with the camera images and the real detectors instead of
      the mocks
