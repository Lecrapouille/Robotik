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

- [ ] Models of people made with MakeHuman (MPFB 2 in Blender, CC0): a
      script that makes a man, a woman and a child with clothes, the
      "game engine" rig (53 joints) and the ARKit face units as shape keys,
      then exports a GLB. Replaces `Soldier.glb` in `demos/Reception`
- [ ] Clips for these models under a free license: the Universal Animation
      Library of Quaternius (CC0) or the CMU motion capture, retargeted in
      Blender (idle, walk, run, talk, wave, point, sit)
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

## 🗣 What the robot says and does

- [ ] A speaker actuator and a `Say` skill: the robot speaks into the same
      list of utterances as the actors, so that `(said robot follow_me)`
      can start an event of the story
- [ ] Text to speech on the real robot (Piper), behind the same actuator
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
