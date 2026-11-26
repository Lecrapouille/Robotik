# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
"""Makes the people of the Reception demo with MPFB (MakeHuman in Blender).

Each person of people.json becomes one GLB file: the body with its skin,
eyes, eyebrows, eyelashes, teeth, tongue, hair and clothes, the Mixamo
skeleton of MPFB (52 joints), the face units of ARKit and the visemes of
Microsoft as morph targets, and the clips of a file with a Mixamo skeleton
(Idle, Walk, Run of Soldier.glb), moved onto the new body.

Run it with Blender, MPFB installed (Edit > Preferences > Get Extensions):

    blender -b --python make_people.py -- \\
        --people people.json --packs ~/.cache/robotik/mpfb-packs \\
        --clips ../data/people/Soldier.glb --out ../data/people

--packs is a folder of MakeHuman asset pack zips (CC0): the ones that are not
installed in MPFB yet are installed first. --only woman,child makes some of
them. The bodies, skins, clothes and face units are CC0; the clips keep the
license of the file they come from.

A model faces -Z of its glTF file, like Soldier.glb (model_yaw: pi in a
world file).
"""

import argparse
import glob
import json
import math
import os
import sys
import zipfile

import bpy
from mathutils import Matrix, Quaternion, Vector

try:
    from bl_ext.blender_org.mpfb.services.assetservice import AssetService
    from bl_ext.blender_org.mpfb.services.exportservice import ExportService
    from bl_ext.blender_org.mpfb.services.faceservice import FaceService
    from bl_ext.blender_org.mpfb.services.humanservice import HumanService
    from bl_ext.blender_org.mpfb.services.locationservice import LocationService
    from bl_ext.blender_org.mpfb.services.targetservice import TargetService
except ImportError as error:
    sys.exit("MPFB is not installed or not enabled in this Blender: " + str(error))

HIPS = "mixamorig:Hips"
LEFT_LEG = "mixamorig:LeftUpLeg"
RIGHT_LEG = "mixamorig:RightUpLeg"

# The parts of a body: key of people.json -> MPFB asset folder and type.
BODY_PARTS = [("eyes", "eyes"), ("eyebrows", "eyebrows"), ("eyelashes", "eyelashes"),
              ("teeth", "teeth"), ("tongue", "tongue"), ("hair", "hair")]


def arguments():
    """The options after '--' on the command line of Blender."""
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description="Make the people of the Reception demo")
    parser.add_argument("--people", default=os.path.join(here, "people.json"))
    parser.add_argument("--packs", default=os.path.expanduser("~/.cache/robotik/mpfb-packs"),
                        help="folder of MakeHuman asset pack zips to install")
    parser.add_argument("--clips", default=os.path.join(here, "..", "data", "people", "Soldier.glb"),
                        help="glTF file with a Mixamo skeleton and its clips")
    parser.add_argument("--out", default=os.path.join(here, "..", "data", "people"))
    parser.add_argument("--only", default="", help="names separated by commas")
    parser.add_argument("--texture-size", type=int, default=2048,
                        help="largest side of a picture in the GLB (0 keeps them)")
    return parser.parse_args(argv)


# ----------------------------------------------------------------------------
# Asset packs
# ----------------------------------------------------------------------------

def install_packs(folder):
    """Installs the packs of the zips of a folder that MPFB does not have."""
    target = LocationService.get_user_data()
    changed = False
    for path in sorted(glob.glob(os.path.join(folder, "*.zip"))):
        with zipfile.ZipFile(path) as archive:
            names = [n for n in archive.namelist() if n.endswith(".json") and "packs/" in n]
        markers = [os.path.join(target, "packs", os.path.basename(n)) for n in names]
        if markers and all(os.path.exists(m) for m in markers):
            continue
        print("make_people: installing", os.path.basename(path))
        error = AssetService.fix_and_extract_asset_pack_zip(path, target)
        if error:
            sys.exit("make_people: cannot install " + path + ": " + error)
        changed = True
    if changed:
        AssetService.rescan_pack_metadata()
    AssetService.update_all_asset_lists()


def find_asset(subdir, name, extension):
    """The file of an asset: <root>/<subdir>/**/<name>/<something>.<extension>."""
    for root in AssetService.get_asset_roots(subdir):
        for path in sorted(glob.glob(os.path.join(root, "**", name, "*." + extension), recursive=True)):
            return path
    sys.exit("make_people: no " + subdir + " '" + name + "' in the installed packs")


# ----------------------------------------------------------------------------
# The body
# ----------------------------------------------------------------------------

def make_body(spec):
    """A human of MPFB with its rig, skin, body parts, clothes and face units."""
    macro = TargetService.get_default_macro_info_dict()
    for key, value in spec.get("macro", {}).items():
        if key == "race":
            macro["race"].update(value)
        else:
            macro[key] = value
    basemesh = HumanService.create_human(macro_detail_dict=macro)
    rig = HumanService.add_builtin_rig(basemesh, "mixamo")
    HumanService.set_character_skin(find_asset("skins", spec["skin"], "mhmat"), basemesh,
                                     skin_type="GAMEENGINE", material_instances=False)
    for key, subdir in BODY_PARTS:
        if spec.get(key):
            HumanService.add_mhclo_asset(find_asset(subdir, spec[key], "mhclo"), basemesh,
                                         asset_type=subdir, subdiv_levels=0, material_type="GAMEENGINE")
    for clothes in spec.get("clothes", []):
        HumanService.add_mhclo_asset(find_asset("clothes", clothes, "mhclo"), basemesh,
                                     asset_type="Clothes", subdiv_levels=0, material_type="GAMEENGINE")
    # The sliders become the mesh; then the face units and the visemes are
    # the only shape keys, on the body and on what follows the face.
    TargetService.bake_targets(basemesh)
    FaceService.load_targets(basemesh, load_microsoft_visemes=True, load_arkit_faceunits=True)
    FaceService.interpolate_targets(basemesh)
    ExportService.bake_modifiers_remove_helpers(basemesh, bake_masks=True, bake_subdiv=False,
                                                remove_helpers=True)
    return basemesh, rig


def top(obj):
    """The object at the top of the parents of an object."""
    while obj.parent is not None:
        obj = obj.parent
    return obj


def turn_about_z(obj, angle):
    """Turns an object and its children about the vertical axis of the world."""
    root = top(obj)
    root.matrix_world = Matrix.Rotation(angle, 4, "Z") @ root.matrix_world
    bpy.context.view_layer.update()


def head_world(armature, bone):
    """Where a bone starts, at rest, in the world."""
    return armature.matrix_world @ armature.data.bones[bone].head_local


def rotation_of(matrix):
    """The rotation of a matrix that may hold a scale."""
    return matrix.to_3x3().normalized().to_quaternion()


# ----------------------------------------------------------------------------
# The clips
# ----------------------------------------------------------------------------

def import_clips(path):
    """Imports a glTF file; returns its armature, its objects and its actions."""
    objects = set(bpy.data.objects)
    actions = set(bpy.data.actions)
    bpy.ops.import_scene.gltf(filepath=path)
    new_objects = [o for o in bpy.data.objects if o not in objects]
    armatures = [o for o in new_objects if o.type == "ARMATURE"]
    if not armatures:
        sys.exit("make_people: " + path + " has no skeleton")
    return armatures[0], new_objects, [a for a in bpy.data.actions if a not in actions]


def set_action(obj, action):
    """Gives an action to an object (with its slot, from Blender 4.4)."""
    data = obj.animation_data or obj.animation_data_create()
    data.action = action
    if action is not None and hasattr(data, "action_slot") and len(action.slots) > 0:
        data.action_slot = action.slots[0]


def retarget(source, target, action, name):
    """Moves a clip of the source skeleton onto the target, bone by bone.

    The two skeletons have the same bone names but not the same rest pose
    (a T pose for Mixamo, arms lower for MakeHuman). Each bone of the target
    turns in the world as its source bone does from its rest, starting from
    a rest where it points like the source bone. The hips also move, scaled
    by the height of the hips.
    """
    scene = bpy.context.scene
    set_action(source, action)
    first, last = (int(round(f)) for f in action.frame_range)
    world_s = source.matrix_world.copy()
    world_t = target.matrix_world.copy()
    turn_t = rotation_of(world_t)
    bones = sorted(target.data.bones, key=lambda b: len(b.parent_recursive))
    mapped = {b.name for b in bones if b.name in source.data.bones}

    rest_s = {n: rotation_of(world_s @ source.data.bones[n].matrix_local) for n in mapped}
    start_t = {}
    for n in mapped:
        sb, tb = source.data.bones[n], target.data.bones[n]
        direction_s = (world_s.to_3x3() @ (sb.tail_local - sb.head_local)).normalized()
        direction_t = (world_t.to_3x3() @ (tb.tail_local - tb.head_local)).normalized()
        start_t[n] = direction_t.rotation_difference(direction_s) @ rotation_of(world_t @ tb.matrix_local)
    hips_s = head_world(source, HIPS)
    hips_t = head_world(target, HIPS)
    ratio = hips_t.z / hips_s.z if hips_s.z > 1e-6 else 1.0

    clip = bpy.data.actions.new(name)
    set_action(target, clip)
    for pose_bone in target.pose.bones:
        pose_bone.rotation_mode = "QUATERNION"
    previous = {}
    for frame in range(first, last + 1):
        scene.frame_set(frame)
        posed = {}
        for bone in bones:
            rest = bone.matrix_local
            if bone.parent is not None:
                chain = posed[bone.parent.name] @ bone.parent.matrix_local.inverted() @ rest
            else:
                chain = rest
            basis = Matrix.Identity(4)
            if bone.name in mapped:
                source_world = world_s @ source.pose.bones[bone.name].matrix
                turn = rotation_of(source_world) @ rest_s[bone.name].inverted()
                rotation = turn_t.inverted() @ (turn @ start_t[bone.name])
                location = chain.to_translation()
                if bone.name == HIPS:
                    moved = hips_t + (source_world.to_translation() - hips_s) * ratio
                    location = world_t.inverted() @ moved
                wanted = Matrix.LocRotScale(location, rotation, Vector((1.0, 1.0, 1.0)))
                basis = chain.inverted() @ wanted
            location, rotation, _ = basis.decompose()
            if bone.name != HIPS:
                location = Vector((0.0, 0.0, 0.0))
            if bone.name in previous and rotation.dot(previous[bone.name]) < 0.0:
                rotation.negate()
            previous[bone.name] = rotation
            pose_bone = target.pose.bones[bone.name]
            pose_bone.location = location
            pose_bone.rotation_quaternion = rotation
            pose_bone.scale = (1.0, 1.0, 1.0)
            pose_bone.keyframe_insert("rotation_quaternion", frame=frame)
            if bone.name == HIPS:
                pose_bone.keyframe_insert("location", frame=frame)
            posed[bone.name] = chain @ Matrix.LocRotScale(location, rotation, Vector((1.0, 1.0, 1.0)))
    # One NLA track per clip: the glTF exporter makes one animation of each.
    set_action(target, None)
    track = target.animation_data.nla_tracks.new()
    track.name = name
    strip = track.strips.new(name, first, clip)
    if hasattr(strip, "action_slot") and len(clip.slots) > 0:
        strip.action_slot = clip.slots[0]
    return last - first + 1


def add_clips(target, path):
    """Imports the clips of a file, moves them onto the target, removes the file."""
    source, objects, actions = import_clips(path)
    # Face the same way as the target: the left leg on the same side.
    left_s = head_world(source, LEFT_LEG) - head_world(source, RIGHT_LEG)
    left_t = head_world(target, LEFT_LEG) - head_world(target, RIGHT_LEG)
    angle = math.atan2(left_t.y, left_t.x) - math.atan2(left_s.y, left_s.x)
    turn_about_z(source, angle)
    for obj in bpy.data.objects:
        if obj.animation_data is not None and obj in objects:
            obj.animation_data.action = None
            for track in list(obj.animation_data.nla_tracks):
                obj.animation_data.nla_tracks.remove(track)
    names = []
    for action in actions:
        name = action.name.split("|")[-1].split(".")[0]
        if name.lower() == "tpose":
            continue
        frames = retarget(source, target, action, name)
        names.append(name + " (" + str(frames) + " frames)")
    for obj in objects:
        bpy.data.objects.remove(obj, do_unlink=True)
    for action in actions:
        bpy.data.actions.remove(action)
    for mesh in [m for m in bpy.data.meshes if m.users == 0]:
        bpy.data.meshes.remove(mesh)
    return names


# ----------------------------------------------------------------------------
# Export
# ----------------------------------------------------------------------------

def shrink_pictures(size):
    """Scales down the pictures larger than size, to keep the GLB small."""
    if size <= 0:
        return
    for image in bpy.data.images:
        width, height = image.size
        if max(width, height) > size:
            scale = size / max(width, height)
            image.scale(max(1, int(width * scale)), max(1, int(height * scale)))


def export(path):
    """Writes the whole scene as one GLB: meshes, skin, morph targets, clips."""
    bpy.ops.export_scene.gltf(
        filepath=path,
        export_format="GLB",
        export_yup=True,
        export_apply=False,
        export_skins=True,
        export_morph=True,
        export_morph_normal=True,
        export_animations=True,
        export_animation_mode="NLA_TRACKS",
        export_image_format="JPEG",
        export_jpeg_quality=90)


def main():
    options = arguments()
    with open(options.people, encoding="utf-8") as file:
        people = json.load(file)["people"]
    only = [n for n in options.only.split(",") if n]
    os.makedirs(options.out, exist_ok=True)
    if os.path.isdir(options.packs):
        install_packs(options.packs)
    for name, spec in people.items():
        if only and name not in only:
            continue
        bpy.ops.wm.read_factory_settings(use_empty=True)
        basemesh, rig = make_body(spec)
        # Face -Z of the glTF file (-Y in Blender... then +Y), like Soldier.glb.
        turn_about_z(rig, math.pi)
        clips = add_clips(rig, options.clips) if os.path.isfile(options.clips) else []
        shrink_pictures(options.texture_size)
        path = os.path.join(options.out, name + ".glb")
        export(path)
        keys = basemesh.data.shape_keys.key_blocks if basemesh.data.shape_keys else []
        print("make_people: wrote", path, "height %.2f m," % basemesh.dimensions.z,
              len(rig.data.bones), "joints,", max(0, len(keys) - 1), "morph targets, clips:",
              ", ".join(clips) or "none")


main()
