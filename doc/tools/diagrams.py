# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
"""Draws the diagrams of the documentation, in one style.

Run it from anywhere; it writes the SVG files next to the pages:

    python3 doc/tools/diagrams.py

Change a diagram here, run the script, then look at the result (for
example with `magick file.svg file.png`). The drawings of
doc/tutorials/images/{plugin-load,scenario-logic-step,tree-scheduler-skill,
planner-loop,robot-joint-loop,task-skill-run,package-scenario-verdict}.svg,
doc/guides/images/tool-mount.svg and the three first drawings of
doc/concepts/images are written by hand: edit those files directly.
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from svgkit import Svg  # noqa: E402

R = str(pathlib.Path(__file__).resolve().parents[2])
G = f"{R}/doc/guides/images"
C = f"{R}/doc/concepts/images"
F = f"{R}/doc/reference/images"
T = f"{R}/doc/tutorials/images"
K = f"{R}/doc/case-studies/images"

# ---- group 1 ----

# README: what you write -> RobotIK -> where it runs
s = Svg(900, 400, "From your files to four ways of running",
        "You write a world file, a scenario file and skills in C++. RobotIK runs them in the 3D simulator, as a headless test in CI, as a reinforcement learning environment, and later on a real robot.",
        "Write the mission once, run it four ways")
s.box(32, 70, 250, 80, "world.yaml", ["robot, cube, box"], "file")
s.box(325, 70, 250, 80, "scenario.yaml", ["goal, faults, assertions"], "file")
s.box(618, 70, 250, 80, "skills (C++)", ["Detect, Grasp, Release…"], "yours")
s.box(325, 190, 250, 56, "RobotIK", [], "box", center=True)
for x in (157, 450, 743):
    s.arrow([(x, 150), (x, 170), (450, 170), (450, 185)] if x != 450 else [(450, 150), (450, 185)])
s.box(32, 290, 190, 80, "3D simulator", ["see and debug"], "box")
s.box(250, 290, 190, 80, "headless test", ["PASS / FAIL in CI"], "box")
s.box(468, 290, 190, 80, "RL environment", ["N in parallel"], "box")
s.box(686, 290, 182, 80, "real robot", ["planned"], "future")
for x in (127, 345, 563, 777):
    s.arrow([(450, 246), (450, 268), (x, 268), (x, 285)] if x != 450 else [(450, 246), (450, 285)])
s.legend(["file", "yours", "future"])
s.save(f"{R}/doc/images/mission-four-ways.svg")

# Decision layers: intent -> planner -> tree -> skills -> robot (README + concept 04)
s = Svg(900, 520, "Who decides what",
        "An intent such as put the red cube in the box becomes a goal. A planner finds which actions are needed. A behavior tree runs them in order and reacts to failures. Skills and the scheduler do each action on this robot. The robot API sends the commands to the physics, and later to a real robot.",
        "From an intent to the motors")
rows = [
    ("Intent", '"Put the red cube in the box"', "a user, later a language model", "future"),
    ("Goal", "(inside red_cube box)", "a structured goal", "file"),
    ("PDDL planner", "which actions are needed?", "GOAP: planned", "box"),
    ("Behavior tree", "in which order, with which recovery?", "", "box"),
    ("Skills + scheduler", "how, on this robot; can it run now?", "", "yours"),
    ("Robot API", "MuJoCo now; a real robot: planned", "", "box"),
]
y = 64
for i, (t, a, b, k) in enumerate(rows):
    s.box(32, y, 520, 58, None, [], k)
    s.text(52, y + 25, t)
    s.text(52, y + 46, a, "small")
    if b:
        s.text(575, y + 35, b, "note")
    if i < len(rows) - 1:
        s.arrow([(292, y + 58), (292, y + 75)])
    y += 76
s.save(f"{R}/doc/images/decision-layers.svg")

# Concept 01: sense, think, act
s = Svg(900, 300, "Sensors, program, actuators",
        "Sensors measure the world, the program decides, the actuators act. The program uses a model of the body of the robot.",
        "A robot, for a program")
s.box(32, 90, 200, 80, "Sensors", ["it sees, it measures"], "box")
s.box(330, 90, 240, 80, "The program", ["the brain: it decides"], "yours")
s.box(668, 90, 200, 80, "Actuators", ["it acts: motors"], "box")
s.arrow([(232, 130), (325, 130)])
s.arrow([(570, 130), (663, 130)])
s.box(330, 210, 240, 64, "A model", ['"what is my body?"'], "file")
s.arrow([(450, 210), (450, 175)])
s.save(f"{R}/doc/concepts/images/sense-think-act.svg")

# Concept 01: example behavior tree
s = Svg(900, 400, "A small behavior tree",
        "A Sequence runs Home, then a Selector, then Grasp. The Selector tries Detect(red_cube); if it fails, it tries Home.",
        "A behavior tree: Sequence and Selector")
s.box(370, 64, 160, 56, "Sequence", [], "file", center=True)
s.text(545, 98, "do the children one after the other", "note")
s.path("M450 120V150M150 150H750M150 150V170M450 150V170M750 150V170", "line")
s.box(70, 175, 160, 56, "Home", [], "yours", center=True)
s.text(70, 252, "a skill", "note")
s.box(370, 175, 160, 56, "Selector", [], "file", center=True)
s.box(670, 175, 160, 56, "Grasp(red_cube)", [], "yours", center=True)
s.path("M450 231V260M330 260H570M330 260V280M570 260V280", "line")
s.box(240, 285, 180, 56, "Detect(red_cube)", [], "yours", center=True)
s.box(480, 285, 180, 56, "Home", [], "yours", center=True)
s.text(32, 375, "Selector (a fallback): try the first child; if it fails, try the next one.", "note")
s.legend(["file", "yours"])
s.save(f"{R}/doc/concepts/images/behavior-tree-example.svg")

# Concept 01: RobotIK connects libraries
s = Svg(900, 360, "RobotIK connects existing libraries",
        "Your program, a plugin, uses RobotIK: scenarios, skills, planner, behavior trees, perception and tests. RobotIK uses MuJoCo for physics, Pinocchio for kinematics, Kairos and BlackThorn for planning and behavior trees, and Compages for the 3D world and the display.",
        "RobotIK connects good libraries")
s.box(300, 64, 300, 56, "Your program (a plugin)", [], "yours", center=True)
s.arrow([(450, 120), (450, 145)])
s.box(32, 150, 836, 70, "RobotIK", ["scenarios · skills · planner · behavior trees · perception · tests"], "box", center=True)
libs = [("MuJoCo", "physics"), ("Pinocchio", "kinematics"), ("Kairos · BlackThorn", "planner · behavior trees"), ("Compages", "3D world, display")]
x = 32
for t, l in libs:
    s.box(x, 260, 194, 70, t, [l], "lib", center=True)
    s.arrow([(x + 97, 220), (x + 97, 255)])
    x += 214
s.legend(["yours", "lib"])
s.save(f"{R}/doc/concepts/images/robotik-connects.svg")

# Concept 02: experiment uses scenario uses world
s = Svg(900, 200, "Experiment, scenario, world",
        "An experiment uses scenarios with variants. A scenario uses a world, and adds the task and the checks. A world lists the things.",
        "Three files, each one uses the next")
s.box(32, 80, 240, 80, "experiment", ["variants, seeds"], "file")
s.box(330, 80, 240, 80, "scenario", ["task + assertions"], "file")
s.box(628, 80, 240, 80, "world", ["the things"], "file")
s.arrow([(272, 120), (325, 120)]); s.text(282, 112, "uses", "small")
s.arrow([(570, 120), (623, 120)]); s.text(580, 112, "uses", "small")
s.save(f"{R}/doc/concepts/images/experiment-scenario-world.svg")
# Concept 01: the control loop
s = Svg(900, 330, "The control loop",
        "The program reads the sensors, decides, sends commands to the actuators, waits a little, and starts again, many times per second.",
        "The loop of a robot program, many times per second")
pts = [(60, 90, "Read the sensors"), (500, 90, "Decide"), (500, 220, "Send commands"), (60, 220, "Wait a little")]
for x, y, t in pts:
    s.box(x, y, 340, 64, t, [], "box", center=True)
s.arrow([(400, 122), (495, 122)])
s.arrow([(670, 154), (670, 215)])
s.arrow([(500, 252), (405, 252)])
s.arrow([(230, 220), (230, 159)])
s.text(240, 195, "again", "note")
s.save(f"{R}/doc/concepts/images/control-loop.svg")


# ---- group 2 ----

# Concept 04: tree -> scheduler -> skill -> robot API
s = Svg(900, 470, "Who decides, who allows, who does",
        "The behavior tree requests Grasp(red_cube). The skill scheduler checks that the gripper and the arm are free and that the preconditions hold, then starts the skill or makes it wait. At each step it ticks the skill, which moves the arm a little toward the cube through the robot API.",
        "Who decides, who allows, who does")
rows = [("Behavior tree", '"I want Grasp(red_cube) now."', "file", "request"),
        ("SkillScheduler", '"The gripper and the arm are free, the preconditions hold: start." (or: wait, or: failed)', "box", "tick"),
        ("Skill", '"I move the arm a little toward the cube."', "yours", "commands"),
        ("Robot API", "joints, actuators, sensors", "box", None)]
y = 64
for t, l, k, a in rows:
    s.box(32, y, 836, 70, None, [], k)
    s.text(52, y + 30, t)
    s.text(52, y + 54, l, "small")
    if a:
        s.arrow([(200, y + 70), (200, y + 98)])
        s.text(212, y + 90, a, "small")
    y += 100
s.legend(["file", "yours"])
s.save(f"{C}/tree-scheduler-skill-robot.svg")

# Concept 04: resources of two skills
s = Svg(900, 300, "Resources of two skills",
        "Grasp needs the arm and the gripper, exclusive, and the camera, shared. LookAround needs the head camera, shared. Two skills can run together when their exclusive resources differ.",
        "What each skill reserves")
s.box(32, 70, 400, 200, "Grasp", [], "yours")
for i, (r, a) in enumerate([("arm", "exclusive"), ("gripper", "exclusive"), ("camera", "shared")]):
    s.box(60, 120 + i * 48, 340, 40, None, [], "box" if a == "exclusive" else "ok")
    s.text(76, 146 + i * 48, r); s.text(260, 146 + i * 48, a, "small")
s.box(468, 70, 400, 120, "LookAround", [], "yours")
s.box(496, 120, 340, 40, None, [], "ok"); s.text(512, 146, "head_camera"); s.text(696, 146, "shared", "small")
s.text(468, 230, "exclusive: one skill at a time", "note")
s.text(468, 252, "shared: several skills together", "note")
s.save(f"{C}/skill-resources.svg")

# Concept 04: states of a requested skill
s = Svg(900, 560, "The states of a skill",
        "request() gives a ticket and makes the run Waiting; a second request of the same skill waits in a queue. The run stays Waiting while a resource is busy or a precondition is false. When the resources are free and the preconditions true, it is Running and ticked at each step. SUCCESS gives Succeeded, FAILURE gives Failed. A cancel or a preemption makes it Stopping: it keeps its resources until the skill is safe, then it is Cancelled, or Waiting again when it was preempted and may wait. A lost resource ends it Failed at once.",
        "The states of a requested skill")
s.box(370, 64, 160, 50, "request()", [], "file", center=True)
s.text(545, 86, "gives a ticket (RunId); a second request", "note")
s.text(545, 106, "of the same skill waits in a queue", "note")
s.arrow([(450, 114), (450, 140)])
s.box(370, 145, 160, 56, "Waiting", [], "box", center=True)
s.text(355, 170, "a resource is busy, or a precondition", "note", anchor="end")
s.text(355, 190, "is false (wait = true)", "note", anchor="end")
s.arrow([(450, 201), (450, 240)])
s.text(462, 226, "resources free and preconditions true", "small")
s.box(370, 245, 160, 56, "Running", [], "box", center=True)
s.text(545, 278, "tick() at each step", "note")
s.path("M450 301V325M170 325H730M170 325V355M450 325V355M730 325V355", "line")
s.text(180, 372, "SUCCESS", "small")
s.text(460, 372, "FAILURE", "small")
s.text(450, 452, "or a lost resource", "note", anchor="middle")
s.text(720, 372, "cancel or preempted", "small", anchor="end")
s.arrow([(170, 355), (170, 380)]); s.arrow([(450, 355), (450, 380)]); s.arrow([(730, 355), (730, 380)])
s.box(85, 385, 170, 50, "Succeeded", [], "ok", center=True)
s.box(365, 385, 170, 50, "Failed", [], "fail", center=True)
s.box(645, 385, 170, 50, "Stopping", [], "box", center=True)
s.text(730, 452, "resources kept until stopping() is safe", "note", anchor="middle")
s.arrow([(730, 460), (730, 485)])
s.box(645, 490, 170, 50, "Cancelled", [], "fail", center=True)
s.path("M815 410H860V173H535", "line")
s.arrow([(540, 173), (532, 173)])
s.text(852, 300, "preempted,", "note", anchor="end")
s.text(852, 318, "wait = true", "note", anchor="end")
s.save(f"{C}/skill-states.svg")

# Concept 06: RobotIK and its libraries
s = Svg(900, 420, "RobotIK and its backends",
        "RobotIK has three parts: the robot API, the autonomy (skills, behavior trees, scheduler) and the scenarios. All three reach the world through backends: Pinocchio for kinematics, MuJoCo for physics, Compages for the 3D world, and later a hardware backend.",
        "RobotIK: three parts on top of backends")
s.box(300, 64, 300, 50, "RobotIK", [], "box", center=True)
s.path("M450 114V134M150 134H750M150 134V154M450 134V154M750 134V154", "line")
s.box(40, 159, 220, 70, "Robot API", ["joints, sensors"], "box", center=True)
s.box(340, 159, 220, 70, "Autonomy", ["skills · trees · scheduler"], "box", center=True)
s.box(640, 159, 220, 70, "Scenarios", ["files, verdicts"], "box", center=True)
for x in (150, 450, 750):
    s.arrow([(x, 229), (x, 259)])
s.box(32, 264, 836, 46, "Backends", [], "box", center=True)
libs = [("Pinocchio", "kinematics", "lib"), ("MuJoCo", "physics", "lib"), ("Compages", "3D world", "lib"), ("Hardware", "SO-101…, planned", "future")]
x = 32
for t, l, k in libs:
    s.arrow([(x + 97, 310), (x + 97, 335)])
    s.box(x, 340, 194, 64, t, [l], k, center=True)
    x += 214
s.legend(["lib", "future"])
s.save(f"{C}/robotik-backends.svg")

# Concept 06: the layers
s = Svg(900, 500, "The four layers of RobotIK",
        "Package: files and plugins, Simulation, RunReport, the YAML files, assertions, plugin ABI. Task: a job in C++, RoboticCell, ScenarioLogic, SkillScheduler, skills, WorldModel, planning, grasp, learning. Robot: one robot, Robot, RobotRuntime, joints, sensors, actuators, resources, backends, asset packs. Common: units, poses, Result, Signal, Config, Random. Each layer uses only the layers below it.",
        "Each layer uses only the layers below it")
layers = [("Package", "files and plugins", "Simulation (scenario = cell + verdict), RunReport, World / Scenario / Experiment YAML, Assertions, plugin ABI"),
          ("Task", "a job in C++", "RoboticCell (step loop, controllers, truth), ScenarioLogic, SkillScheduler, skills, WorldModel, Planning (PDDL), Grasp, Learning"),
          ("Robot", "one robot", "Robot, RobotRuntime, StandaloneRobot, joints, sensors, actuators, resources, backends (MuJoCo, Pinocchio), asset packs"),
          ("Common", "small tools", "units and poses (from Compages), Result, Signal, Config, Random")]
y = 64
for i, (t, sub, l) in enumerate(layers):
    s.box(32, y, 836, 90, None, [], "box")
    s.text(52, y + 32, t, None); s.text(150, y + 32, sub, "note")
    words = l.split(" "); line = ""; ly = y + 58
    out = []
    for w in words:
        if len(line) + len(w) > 95:
            out.append(line); line = w
        else:
            line = (line + " " + w).strip()
    out.append(line)
    for k, o in enumerate(out[:2]):
        s.text(52, y + 58 + k * 18, o, "small")
    if i < 3:
        s.arrow([(450, y + 90), (450, y + 104)])
    y += 106
s.save(f"{C}/layers.svg")

# Concept 06: backends of a session
s = Svg(900, 340, "The backend of a robot session",
        "A RobotRuntime uses a RobotBackend. The MuJoCo backend reads and writes the joint positions and velocities of the simulation. A hardware backend, planned, will talk to the servos on their bus.",
        "Simulated or real: the same session")
s.box(330, 64, 240, 50, "RobotRuntime", [], "box", center=True)
s.arrow([(450, 114), (450, 140)])
s.box(330, 145, 240, 50, "RobotBackend", [], "box", center=True)
s.arrow([(450, 195), (450, 215), (230, 215), (230, 235)])
s.arrow([(450, 215), (670, 215), (670, 235)])
s.box(110, 240, 240, 70, "MujocoBackend", ["qpos / qvel"], "box", center=True)
s.box(550, 240, 240, 70, "HardwareBackend", ["servo / bus · planned"], "future", center=True)
s.save(f"{C}/robot-backends.svg")

# Concept 06: perception pipeline
s = Svg(900, 330, "The perception pipeline",
        "The camera gives a CameraFrame. The PerceptionPipeline runs its detectors (color, AprilTag, depth) and gives detections. The detections update the WorldModel: the beliefs.",
        "From an image to a belief")
s.box(32, 70, 140, 60, "Camera", [], "box", center=True)
s.box(206, 70, 160, 60, "CameraFrame", [], "box", center=True)
s.box(400, 70, 200, 60, "PerceptionPipeline", [], "box", center=True)
s.box(634, 70, 120, 60, "Detections", [], "box", center=True)
s.box(728, 160, 140, 60, "WorldModel", [], "box", center=True)
s.arrow([(172, 100), (201, 100)]); s.arrow([(366, 100), (395, 100)]); s.arrow([(600, 100), (629, 100)])
s.arrow([(754, 100), (798, 100), (798, 155)])
for i, (d, k) in enumerate([("ColorDetector (plugin)", "yours"), ("AprilTagDetector (plugin)", "yours"), ("DepthEstimator", "box")]):
    s.box(420, 165 + i * 50, 260, 40, d, [], k, center=True)
s.path("M410 130V275M410 185H420M410 235H420M410 275H420", "line")
s.text(728, 245, "the beliefs", "note")
s.legend(["yours", "box"])
s.save(f"{C}/perception-pipeline.svg")

# Concept 06: truth to belief to skill
s = Svg(900, 210, "From the truth to a skill",
        "The truth of the simulation is seen by the camera; perception turns the image into a belief; the skill reads only the belief.",
        "A skill never reads the truth")
chain = [("truth", "ok"), ("simulation", "box"), ("camera", "box"), ("perception", "box"), ("belief", "box"), ("skill", "yours")]
x = 32
for i, (t, k) in enumerate(chain):
    s.box(x, 90, 120, 56, t, [], k, center=True)
    if i < len(chain) - 1:
        s.arrow([(x + 120, 118), (x + 143, 118)])
    x += 143
s.text(32, 180, "Assertions, metrics and RL rewards read the truth. Skills and observers read the beliefs.", "note")
s.save(f"{C}/truth-to-belief.svg")

# Concept 06: rendering
s = Svg(900, 190, "Rendering is separate",
        "RobotIK draws through a SceneView, which Compages implements. Without a window, the SceneView is null and nothing is drawn.",
        "RobotIK draws through a SceneView")
s.box(32, 80, 220, 60, "RobotIK", [], "box", center=True)
s.box(340, 80, 220, 60, "SceneView", ["null when headless"], "box", center=True)
s.box(648, 80, 220, 60, "Compages", ["3D rendering"], "lib", center=True)
s.arrow([(252, 110), (335, 110)]); s.arrow([(560, 110), (643, 110)])
s.save(f"{C}/rendering.svg")

# Concept 06: hosts and plugins
s = Svg(900, 355, "Hosts and plugins",
        "RobotIK-Simulator and RobotIK-Headless are the two hosts. A PluginSession reads the catalog and the packs, loads the plugin library with dlopen, and runs its lifecycle. The plugin binds its scenario logic to the Simulation of librobotik-core.",
        "Two hosts, one plugin border")
s.box(150, 64, 260, 56, "RobotIK-Simulator", [], "box", center=True)
s.box(490, 64, 260, 56, "RobotIK-Headless", [], "box", center=True)
s.text(770, 96, "the two hosts", "note")
s.arrow([(280, 120), (280, 140), (450, 140), (450, 170)])
s.arrow([(620, 120), (620, 140), (450, 140)], "line")
s.text(465, 160, "PluginSession: catalog, packs, dlopen, lifecycle", "small")
s.box(300, 175, 300, 56, "librobotik_<name>.so", [], "yours", center=True)
s.text(620, 208, "a plugin (C ABI border)", "note")
s.arrow([(450, 231), (450, 270)])
s.text(465, 256, "bindLogic", "small")
s.box(270, 275, 360, 56, "Simulation + ScenarioLogic", [], "box", center=True)
s.text(650, 308, "librobotik-core", "note")
s.legend(["yours"])
s.save(f"{C}/hosts-and-plugins.svg")


# ---- group 3 ----


def chain(path, title, desc, heading, items, w=900):
    n = len(items)
    gap = 26
    bw = (w - 64 - gap * (n - 1)) / n
    s = Svg(w, 170, title, desc, heading)
    x = 32
    for i, (t, k) in enumerate(items):
        lines = t.split("\n")
        s.box(round(x), 80, round(bw), 64, lines[0], lines[1:], k, center=True)
        if i < n - 1:
            s.arrow([(round(x + bw), 112), (round(x + bw + gap - 4), 112)])
        x += bw + gap
    s.save(path)


# RL: the loop
s = Svg(900, 260, "The reinforcement learning loop",
        "The policy receives an observation and chooses an action. The simulation applies the action and gives a reward and the next observation.",
        "Learning by trial: observation, action, reward")
s.box(60, 100, 200, 64, "Policy", ["your agent"], "yours", center=True)
s.box(350, 100, 200, 64, "Action", ["numbers"], "box", center=True)
s.box(640, 100, 200, 64, "Simulation", ["RobotIK"], "box", center=True)
s.arrow([(260, 132), (345, 132)]); s.arrow([(550, 132), (635, 132)])
s.arrow([(740, 164), (740, 215), (160, 215), (160, 169)])
s.text(330, 207, "observation + reward", "small")
s.legend(["yours"])
s.save(f"{G}/rl-loop.svg")

# RL: inside the environment
s = Svg(900, 440, "Inside a RobotIK environment",
        "The action vector goes through the ActionAdapter, which turns it into commands for the RoboticCell. The cell runs for one control period. The ObservationBuilder turns sensors and beliefs into the observation vector. The RewardEvaluator gives the reward and Success or Failure.",
        "Inside a RobotIK environment")
s.box(60, 70, 260, 50, "action (vector)", [], "box", center=True)
s.box(580, 70, 260, 50, "observation (vector)", [], "box", center=True)
s.box(60, 160, 260, 70, "ActionAdapter", ["vector → commands"], "yours", center=True)
s.box(580, 160, 260, 70, "ObservationBuilder", ["sensors, beliefs → vector"], "yours", center=True)
s.box(320, 280, 260, 64, "RoboticCell", ["held for control_period"], "box", center=True)
s.box(320, 370, 260, 56, "RewardEvaluator", ["reward, Success / Failure"], "yours", center=True)
s.arrow([(190, 120), (190, 155)])
s.arrow([(190, 230), (190, 312), (315, 312)])
s.arrow([(580, 312), (710, 312), (710, 235)])
s.arrow([(710, 160), (710, 125)])
s.arrow([(450, 344), (450, 365)])
s.legend(["yours"])
s.save(f"{G}/rl-environment.svg")

# Architecture: one step
s = Svg(900, 640, "One simulation step",
        "Simulation::step steps the RoboticCell, then decides if the run ends, then updates the assertions. RoboticCell::step updates the faults, ticks the controller, updates the scheduler, steps every robot session, updates the grasps, the perception, the ground truth, then calls the scenario logic.",
        "Simulation::step(dt), in this order")
steps = [
    ("FaultInjector::update", "faults on / off: resources available or not"),
    ("controller tick", "behavior tree or plan: requests and cancels skills"),
    ("SkillScheduler::update", "admit, preempt, tick the skills"),
    ("RobotRuntime prepare / advance / sense", "every robot: actuators, one physics step, sensors"),
    ("GraspModel::update", "what the grippers hold"),
    ("observe()", "perception → WorldModel (or the truth: fidelity)"),
    ("track / syncBodies", "ground truth of the bodies, contacts"),
    ("ScenarioLogic act / simulate / observe", "then status, if there is no controller"),
]
s.box(32, 64, 836, 420, "RoboticCell::step(dt)", [], "box")
y = 108
for i, (a, b) in enumerate(steps):
    k = "yours" if a.startswith("ScenarioLogic") else "box"
    s.box(56, y, 788, 40, None, [], k)
    s.text(72, y + 26, a); s.text(450, y + 26, b, "small")
    y += 46
s.box(32, 500, 836, 50, None, [], "box"); s.text(52, 531, "end of run?"); s.text(300, 531, "controller, stop_when, timeout, stop: true", "small")
s.box(32, 566, 836, 50, None, [], "box"); s.text(52, 597, "AssertionEngine::update"); s.text(300, 597, "verdicts, on the ground truth", "small")
s.arrow([(450, 484), (450, 497)]); s.arrow([(450, 550), (450, 563)])
s.legend(["yours"])
s.save(f"{C}/simulation-step.svg")

# Reference: one seed, many streams
s = Svg(900, 300, "One seed feeds every random draw",
        "The seed of the scenario gives one random stream to the world layout, one to the faults, one to each sensor by its name, and one to the scenario logic.",
        "One seed, one stream per user")
s.box(370, 64, 160, 50, "seed 42", [], "file", center=True)
s.path("M450 114V140M125 140H775M125 140V160M341 140V160M558 140V160M775 140V160", "line")
for x, t, l in [(32, "world", "layout of the props"), (248, "faults", "random faults"), (465, "<sensor>", "one per sensor name"), (682, "scenario logic", "an RL policy…")]:
    s.box(x, 165, 186, 70, t, [l], "box", center=True)
s.text(32, 270, "Same seed, same run.", "note")
s.save(f"{F}/seed-streams.svg")

# Reference: plugin lifecycle
s = Svg(900, 760, "The life of a plugin",
        "The host reads plugin.yaml and checks api_version, packs and dependencies. It loads the library only when a scenario of this plugin is chosen, checks the build id, calls setup with the scenario id, builds the Simulation, starts it, steps it, stops it, and shuts it down.",
        "The life of a plugin, from the host")
st = [
    ("plugin.yaml read", "the host checks api_version, packs, dependencies"),
    ("dlopen(.so)", "only when a scenario of this plugin is chosen"),
    ("build id", "robotik_plugin_build_id must be the text of the host"),
    ("setup(scenario id)", "create the ScenarioLogic (ScenarioPlugin), menus, panels"),
    ("Simulation built", "world + scenario; ScenarioLogic::setup, then reset"),
    ("start", ""),
    ("loop", "RoboticCell::step → act, simulate, observe → Plugin::update(dt)"),
    ("stop", "the simulation still exists"),
    ("shutdown", "the simulation is destroyed: free the scenario logic; the .so can stay loaded"),
]
y = 64
for i, (a, b) in enumerate(st):
    k = "yours" if a.startswith("setup") or a == "loop" else "box"
    s.box(32, y, 836, 56, None, [], k)
    s.text(52, y + 34, a); s.text(250, y + 34, b, "small")
    if i < len(st) - 1:
        s.arrow([(140, y + 56), (140, y + 76)])
    y += 76
s.legend(["yours"])
s.save(f"{F}/plugin-lifecycle.svg")

# Tutorial 07: the names that must match
s = Svg(900, 430, "The names that must match",
        "The target robotik_my_robot of MyRobot.mk builds librobotik_my_robot.so, which is the library.entrypoint of plugin.yaml. The plugin.id of plugin.yaml is the id returned by describe in Plugin.cpp. The scenario id nod is the same in plugin.yaml, in nod.scenario.yaml and in the table of Plugin.cpp.",
        "The same name in two or three files")
groups = [
    ("librobotik_my_robot.so", [("MyRobot.mk", "TARGETS += robotik_my_robot"), ("plugin.yaml", "library.entrypoint: librobotik_my_robot.so")]),
    ("robotik.my_robot", [("plugin.yaml", "plugin.id: robotik.my_robot"), ("Plugin.cpp", 'describe() { .id = "robotik.my_robot" }')]),
    ("nod", [("plugin.yaml", "scenarios.entries: { id: nod, … }"), ("nod.scenario.yaml", "id: nod"), ("Plugin.cpp", '{ "nod", robotik::logic<NodLogic>() }')]),
]
y = 64
for name, rows in groups:
    h = 34 * len(rows) + 16
    s.box(32, y, 200, h, None, [], "ok")
    s.text(132, y + h / 2 + 6, name, "small", anchor="middle")
    for j, (f, l) in enumerate(rows):
        ry = y + 8 + j * 34
        k = "yours" if f.endswith(".cpp") else "file"
        s.box(270, ry, 598, 28, None, [], k)
        s.text(282, ry + 20, f, "small"); s.text(440, ry + 20, l, "small")
        s.path(f"M232 {y + h / 2}L265 {ry + 14}", "line")
    y += h + 24
s.legend(["file", "yours"])
s.save(f"{T}/plugin-names.svg")

# Concept 05: how a plan runs
s = Svg(900, 580, "How a plan runs",
        "At the first step, or after a problem, the observers, the facts, the objects and the goal make a PDDL problem; Kairos finds a plan; the plan becomes a behavior tree with one subtree per action. The tree is ticked. A failed skill, an effect seen false, or a goal still false at the end makes the executor wait replan.delay and plan again. The run succeeds when the goal is not false; it fails when replan.max is reached or the problem is not valid.",
        "How a plan runs")
s.box(32, 64, 836, 50, None, [], "box"); s.text(52, 95, "observers (beliefs) + facts + objects + goal  →  problem.pddl")
s.box(32, 130, 836, 50, None, [], "lib"); s.text(52, 161, "Kairos  →  plan")
s.box(32, 196, 836, 70, None, [], "box"); s.text(52, 225, "plan  →  behavior tree, one subtree per action:"); s.text(52, 251, 'the nodes of its binding, then "Check <n>. <action>"', "small")
s.box(32, 282, 836, 50, None, [], "box"); s.text(52, 313, "tick the tree")
for a, b in [(114, 125), (180, 191), (266, 277)]:
    s.arrow([(450, a), (450, b)])
outs = [("a skill fails", "cancel, wait replan.delay, plan again", "fail"),
        ("Check: an effect is false", "cancel, wait replan.delay, plan again", "fail"),
        ("tree done, goal false", "wait replan.delay, plan again", "fail"),
        ("tree done, goal not false", "success", "ok"),
        ("replan.max reached, or no valid problem", "failure", "fail")]
y = 352
for a, b, k in outs:
    s.box(80, y, 788, 30, None, [], k)
    s.text(96, y + 21, a, "small"); s.text(470, y + 21, b, "small")
    y += 38
s.path("M56 332V543", "line")
for i in range(5):
    s.arrow([(56, 367 + i * 38), (76, 367 + i * 38)])
s.path("M868 367H884V20H450V60", "darrow")
s.text(700, 16, "", "small")
s.text(868, 562, "dashed: plan again from the first box", "note", anchor="end")
s.save(f"{C}/plan-run.svg")

# Case studies: chains
chain(f"{K}/fly-brain-chain.svg", "FlyBrain: from the senses to the body",
      "Sensors give an observation; the brain model chooses an action; a controller turns it into commands of the body.",
      "FlyBrain: the brain is the policy",
      [("sensors", "box"), ("observation", "box"), ("brain", "yours"), ("action", "box"), ("controller", "box"), ("body", "box")])
chain(f"{K}/line-follower-chain.svg", "LineFollower: from the camera to the wheels",
      "The camera image goes to perception (AprilTag and line detectors), then to localisation, then to the controller, which drives the differential robot.",
      "LineFollower: from the camera to the wheels",
      [("camera", "box"), ("perception\nAprilTag, line", "yours"), ("localisation", "yours"), ("controller", "yours"), ("differential\nrobot", "box")])
s = Svg(900, 250, "PickAndPlace: the skills in order",
        "Home, Detect, Approach, Reach, Grasp, Approach, Transit to the box, Approach the box, Release, Home.",
        "PickAndPlace: the skills of one pick and place")
names = ["Home", "Detect", "Approach", "Reach", "Grasp", "Approach", "Transit(box)", "Approach(box)", "Release", "Home"]
x, y = 32, 80
for i, n in enumerate(names):
    w = 146
    if i == 5:
        x, y = 32, 160
    s.box(x, y, w, 50, n, [], "yours", center=True)
    if i not in (4, 9):
        s.arrow([(x + w, y + 25), (x + w + 22, y + 25)])
    x += w + 26
gx = 32 + 4 * 172 + 73
s.arrow([(gx, 130), (gx, 145), (105, 145), (105, 155)])
s.save(f"{K}/pick-and-place-chain.svg")


# ---- group 4 ----

s = Svg(900, 430, "The scene of the tutorial, from above and from the side",
        "Seen from above: the robot base is at the origin, x points forward and y to the left. The red cube is at x = 0.40, y = 0.20 and the box at x = 0.40, y = -0.20, 40 cm apart. Seen from the side: the centre of the cube on the floor is at z = 0.02; in the box, on its 5 mm floor, it is at z = 0.025; the centre of the box is at z = 0.04.",
        "Where the cube and the box are")
# top view: 1 m = 500 px; origin at (110, 230); x to the right, y up
ox, oy, k = 110, 245, 500
s.text(32, 76, "From above", "note")
s.path(f"M{ox} {oy}H{ox + 300}", "arrow"); s.text(ox + 305, oy + 5, "x", "small")
s.path(f"M{ox} {oy}V{oy - 140}", "arrow"); s.text(ox + 10, oy - 132, "y", "small")
s.circle(ox, oy, 16, "#397baf", "#20334b"); s.text(ox - 40, oy + 40, "robot base (0, 0)", "small")
cx, cy = ox + 0.40 * k, oy - 0.20 * k
s.parts.append(f'<rect x="{cx - 10}" y="{cy - 10}" width="20" height="20" fill="#d9363e" stroke="#20334b"/>')
s.text(cx + 18, cy + 5, "red_cube (0.40, 0.20)", "small")
bx, by = ox + 0.40 * k, oy + 0.20 * k
s.parts.append(f'<rect x="{bx - 40}" y="{by - 40}" width="80" height="80" fill="none" stroke="#397baf" stroke-width="3"/>')
s.text(bx + 48, by + 5, "box (0.40, -0.20)", "small")
s.path(f"M{cx} {cy + 14}V{by - 44}", "dashed"); s.text(cx + 10, oy - 8, "0.40 m apart", "note")

# side view: z, 1 m = 1500 px; floor at y=380
fx, fy, kz = 470, 360, 1000
s.text(470, 76, "From the side (z up)", "note")
s.path(f"M{fx} {fy}H{fx + 300}", "line")
s.text(fx, fy + 22, "floor", "note")
# cube on floor
s.parts.append(f'<rect x="{fx + 20}" y="{fy - 0.04 * kz}" width="{0.04 * kz}" height="{0.04 * kz}" fill="#d9363e" stroke="#20334b"/>')
s.circle(fx + 20 + 0.02 * kz, fy - 0.02 * kz, 4, "#fff", "#20334b")
s.text(fx + 10, fy - 0.04 * kz - 12, "z = 0.02", "small")
# box: 0.16 wide, 0.08 high, walls; floor 5 mm
bx0, bw, bh = fx + 110, 0.16 * kz, 0.08 * kz
s.parts.append(f'<path d="M{bx0} {fy - bh}V{fy}H{bx0 + bw}V{fy - bh}" fill="none" stroke="#397baf" stroke-width="4"/>')
s.parts.append(f'<rect x="{bx0}" y="{fy - 0.005 * kz}" width="{bw}" height="{0.005 * kz}" fill="#397baf"/>')
cube_y = fy - 0.005 * kz - 0.04 * kz
s.parts.append(f'<rect x="{bx0 + bw / 2 - 0.02 * kz}" y="{cube_y}" width="{0.04 * kz}" height="{0.04 * kz}" fill="#d9363e" stroke="#20334b"/>')
s.circle(bx0 + bw / 2, fy - 0.025 * kz, 4, "#fff", "#20334b")
s.text(bx0 + bw + 10, fy - 0.025 * kz + 12, "z = 0.025 (cube)", "small")
s.text(bx0 + bw + 10, fy - 0.04 * kz + 1, "z = 0.04 (box)", "small")
s.path(f"M{bx0 + bw + 6} {fy - 0.04 * kz - 4}H{bx0 + bw / 2 + 24}", "dashed")
s.text(bx0 - 10, fy + 22, "5 mm floor", "note")
s.text(32, 410, "A position is the centre of the object. All distances are in metres.", "note")
s.save(f"{R}/doc/tutorials/images/cube-box-scene.svg")
