#!/usr/bin/env python3
"""Build the asset pack of the HumanoidWalk plugin from the LimX repositories.

Called from external/compilation after make download-external-libs. Only the
files needed for walking are copied (the clones also hold USD files):

  - the serial URDF of HU_D04_01 (the ankles and the waist are plain joints,
    as in the training; the MuJoCo file of LimX has the closed linkages),
    without its floating joint: RobotIK adds the free joint itself;
  - the STL meshes the URDF uses, and the collision primitives of the LimX
    MuJoCo file (the URDF only collides by the feet);
  - the walking policy (policy.onnx) and its contract (walk.yaml): joint
    order, PD gains, default posture, scales, torque limits, rotor armature.

    python3 prepare_assets.py <humanoid-description> <humanoid-rl-deploy-cpp> <data dir>

Only the Python standard library is used.
"""

import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROBOT = "HU_D04_01"
DESCRIPTION = "HU_D04_description"
# Links without mass (frames) get this tiny body, so MuJoCo does not give
# them its default of 1 kg.
FRAME_MASS = "0.001"
FRAME_INERTIA = "1e-07"


def numbers(text: str, key: str) -> list[float]:
    """Values of the list named key in a LimX param.yaml (flow or block
    style, with comments)."""
    lines = text.splitlines()
    for i, line in enumerate(lines):
        match = re.match(r"^(\s*)" + re.escape(key) + r":(.*)$", line)
        if match is None:
            continue
        indent = len(match.group(1))
        body = [match.group(2)]
        for next_line in lines[i + 1:]:
            stripped = next_line.split("#")[0]
            if stripped.strip() == "":
                continue
            if len(stripped) - len(stripped.lstrip()) <= indent and \
               not stripped.lstrip().startswith(("-", "]")):
                break
            body.append(stripped)
        joined = " ".join(part.split("#")[0] for part in body)
        return [float(x) for x in re.findall(r"-?\d+\.?\d*(?:[eE]-?\d+)?", joined)]
    raise KeyError(key)


def number(text: str, key: str) -> float:
    match = re.search(r"^\s*" + re.escape(key) + r":\s*([-0-9.eE]+)", text, re.M)
    if match is None:
        raise KeyError(key)
    return float(match.group(1))


def strip_floating_base(robot: ET.Element) -> None:
    """Remove root_link and floating_base_joint: base_link becomes the root."""
    for joint in robot.findall("joint"):
        if joint.get("type") == "floating":
            robot.remove(joint)
            parent = joint.find("parent").get("link")
            for link in robot.findall("link"):
                if link.get("name") == parent:
                    robot.remove(link)


def relocate_meshes(robot: ET.Element, source: Path, meshes: Path) -> None:
    """package:// paths become meshes/<name>.stl, next to the URDF."""
    meshes.mkdir(parents=True, exist_ok=True)
    for mesh in robot.iter("mesh"):
        name = Path(mesh.get("filename")).name
        target = Path(name).stem + ".stl"
        shutil.copyfile(source / name, meshes / target)
        mesh.set("filename", "meshes/" + target)


def give_frames_mass(robot: ET.Element) -> None:
    for link in robot.findall("link"):
        inertial = link.find("inertial")
        mass = inertial.find("mass") if inertial is not None else None
        if mass is not None and float(mass.get("value")) > 1e-8:
            continue
        if inertial is not None:
            link.remove(inertial)
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
        ET.SubElement(inertial, "mass", value=FRAME_MASS)
        ET.SubElement(inertial, "inertia", ixx=FRAME_INERTIA, ixy="0", ixz="0",
                      iyy=FRAME_INERTIA, iyz="0", izz=FRAME_INERTIA)


def add_collisions(robot: ET.Element, mjcf: Path) -> None:
    """Copy the collision primitives of the LimX MuJoCo file onto the URDF
    links that have none (the URDF only keeps the feet): a robot that falls
    must land on its body, not sink into the floor. URDF has no capsule: a
    capsule becomes the cylinder that holds it."""
    links = {link.get("name"): link for link in robot.findall("link")}
    for body in ET.parse(mjcf).getroot().iter("body"):
        link = links.get(body.get("name"))
        if link is None or link.find("collision") is not None:
            continue
        for geom in body.findall("geom"):
            kind = geom.get("type", "sphere")
            if geom.get("class") == "visual" or kind == "mesh":
                continue
            euler = [float(x) for x in geom.get("euler", "0 0 0").split()]
            if sum(1 for angle in euler if angle != 0.0) > 1:
                sys.exit(f"{body.get('name')}: euler on two axes is not converted")
            size = [float(x) for x in geom.get("size").split()]
            collision = ET.SubElement(link, "collision")
            # One rotation axis: MuJoCo euler and URDF rpy agree.
            ET.SubElement(collision, "origin", xyz=geom.get("pos", "0 0 0"),
                          rpy=" ".join(f"{angle:g}" for angle in euler))
            geometry = ET.SubElement(collision, "geometry")
            if kind == "box":
                ET.SubElement(geometry, "box", size=" ".join(f"{2 * s:g}" for s in size))
            elif kind == "sphere":
                ET.SubElement(geometry, "sphere", radius=f"{size[0]:g}")
            elif kind in ("cylinder", "capsule"):
                length = 2 * size[1] + (2 * size[0] if kind == "capsule" else 0.0)
                ET.SubElement(geometry, "cylinder", radius=f"{size[0]:g}", length=f"{length:g}")
            else:
                sys.exit(f"{body.get('name')}: geom type {kind} is not converted")


def set_efforts(robot: ET.Element, efforts: dict[str, float]) -> None:
    """The deployment torque limits: a serial ankle sums its two motors."""
    for joint in robot.findall("joint"):
        limit = joint.find("limit")
        if limit is not None and joint.get("name") in efforts:
            limit.set("effort", f"{efforts[joint.get('name')]:g}")


def flow(values: list) -> str:
    return "[" + ", ".join(f"{v:g}" if isinstance(v, float) else str(v) for v in values) + "]"


def main() -> None:
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    description = Path(sys.argv[1]) / DESCRIPTION
    controller = (Path(sys.argv[2]) / "src/humanoid-rl-deploy-cpp/robot_controllers"
                  / "walking_controller/config" / ROBOT)
    output = Path(sys.argv[3]) / "hu_d04"
    output.mkdir(parents=True, exist_ok=True)

    # Joint order of the policy: the SRDF order (same as the robot SDK).
    srdf = ET.parse(description / "urdf" / f"{ROBOT}.srdf").getroot()
    motors = [j for j in srdf.iter("joint") if j.get("gear_ratio") is not None]
    names = [j.get("name") for j in motors]
    # Reflected rotor inertia: rotor inertia times the gear ratio squared.
    armature = [float(j.get("mass")) * float(j.get("gear_ratio")) ** 2 for j in motors]

    param = (controller / "param.yaml").read_text()
    size = int(number(param, "actions_size"))
    if size != len(names):
        sys.exit(f"policy has {size} actions, the SRDF {len(names)} joints")
    efforts = numbers(param, "user_torque_limit")

    tree = ET.parse(description / "urdf" / f"{ROBOT}.urdf")
    robot = tree.getroot()
    strip_floating_base(robot)
    relocate_meshes(robot, description / "meshes" / ROBOT, output / "meshes")
    give_frames_mass(robot)
    add_collisions(robot, description / "xml" / f"{ROBOT}.xml")
    set_efforts(robot, dict(zip(names, efforts)))
    ET.indent(tree)
    tree.write(output / f"{ROBOT}.urdf", encoding="utf-8", xml_declaration=True)

    shutil.copyfile(controller / "policy" / "policy.onnx", output / "walk.onnx")
    contract = [
        f"# Contract of walk.onnx, generated by prepare_assets.py from",
        f"# humanoid-rl-deploy-cpp walking_controller/config/{ROBOT}/param.yaml.",
        "walk:",
        f"  joints: {flow(names)}",
        f"  kp: {flow(numbers(param, 'kp'))}",
        f"  kd: {flow(numbers(param, 'kd'))}",
        f"  default_angle: {flow(numbers(param, 'default_angle'))}",
        f"  action_scale: {flow(numbers(param, 'action_scale'))}",
        f"  torque_limit: {flow(efforts)}",
        f"  armature: {flow(armature)}",
        f"  decimation: {int(number(param, 'decimation'))}",
        f"  history: {int(number(param, 'obs_history_length'))}",
        f"  observations: {int(number(param, 'observations_size'))}",
        f"  clip_observations: {number(param, 'clip_observations'):g}",
        f"  clip_actions: {number(param, 'clip_actions'):g}",
        f"  scale_ang_vel: {number(param, 'ang_vel'):g}",
        f"  scale_dof_pos: {number(param, 'dof_pos'):g}",
        f"  scale_dof_vel: {number(param, 'dof_vel'):g}",
        f"  max_command: [{number(param, 'max_vx'):g}, {number(param, 'max_vy'):g}, "
        f"{number(param, 'max_vz'):g}]",
    ]
    (output / "walk.yaml").write_text("\n".join(contract) + "\n")

    for notice in ("LICENSE",):
        shutil.copyfile(Path(sys.argv[1]) / notice, output / notice)
    print(f"{ROBOT}: {len(names)} joints -> {output}", file=sys.stderr)


if __name__ == "__main__":
    main()
