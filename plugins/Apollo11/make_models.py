#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# Builds the meshes (binary STL) and the URDF of every element of Apollo 11,
# at scale, from data/vehicles.yaml: lengths, diameters, stage positions,
# legs, panels. The detail (fins, bells, quads) is drawn here. One mesh per
# color: a URDF visual has one material.
#
#   python3 make_models.py            # writes data/meshes/*.stl, data/*.urdf
#
# Frames: Z up, origin on the aft interface of the element (its bottom).

import math
import os
import struct
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")
MESHES = os.path.join(DATA, "meshes")

SEGMENTS = 48

WHITE = (0.92, 0.92, 0.90)
BLACK = (0.08, 0.08, 0.09)
ENGINE = (0.30, 0.29, 0.28)
SILVER = (0.80, 0.81, 0.83)
LIGHT = (0.75, 0.75, 0.74)
GOLD = (0.86, 0.66, 0.22)
GRAY = (0.58, 0.58, 0.55)
SHIELD = (0.42, 0.32, 0.22)
ORANGE = (0.95, 0.45, 0.12)


# --- Geometry ---------------------------------------------------------------

def lathe(profile, segments=SEGMENTS, a0=0.0, a1=2.0 * math.pi):
    """Turns a profile [(r, z)] about Z. Give the profile counterclockwise in
    the (r, z) half plane (up on the outer side) for outward normals."""
    triangles = []
    for j in range(segments):
        t0 = a0 + (a1 - a0) * j / segments
        t1 = a0 + (a1 - a0) * (j + 1) / segments
        c0, s0, c1, s1 = math.cos(t0), math.sin(t0), math.cos(t1), math.sin(t1)
        for (ra, za), (rb, zb) in zip(profile, profile[1:]):
            p00 = (ra * c0, ra * s0, za)
            p01 = (ra * c1, ra * s1, za)
            p10 = (rb * c0, rb * s0, zb)
            p11 = (rb * c1, rb * s1, zb)
            if ra > 1e-9:
                triangles.append((p00, p01, p11))
            if rb > 1e-9:
                triangles.append((p00, p11, p10))
    return triangles


def cylinder(radius, z0, z1, segments=SEGMENTS):
    return lathe([(0, z0), (radius, z0), (radius, z1), (0, z1)], segments)


def frustum(r0, r1, z0, z1, segments=SEGMENTS):
    return lathe([(0, z0), (r0, z0), (r1, z1), (0, z1)], segments)


def shell(r0, r1, z0, z1, thickness, a0=0.0, a1=2.0 * math.pi, segments=SEGMENTS):
    """A wall of revolution between radii r0 (at z0) and r1 (at z1)."""
    profile = [(r0 - thickness, z0), (r0, z0), (r1, z1), (r1 - thickness, z1), (r0 - thickness, z0)]
    return lathe(profile, segments, a0, a1)


def bell(throat, exit, z_throat, z_exit, thickness=0.04, segments=32):
    """An engine nozzle, open at both ends, exit below the throat."""
    samples = 8
    outer = []
    for i in range(samples + 1):
        u = i / samples
        outer.append((throat + (exit - throat) * math.sqrt(u), z_throat + (z_exit - z_throat) * u))
    outer.reverse()  # from the exit up to the throat
    inner = [(r - thickness, z) for r, z in reversed(outer)]
    profile = [(r + thickness * 0.0, z) for r, z in outer] + inner + [outer[0]]
    return lathe(profile, segments)


def sphere(radius, segments=24):
    profile = [(radius * math.sin(math.pi * i / 12), -radius * math.cos(math.pi * i / 12)) for i in range(13)]
    return lathe(profile, segments)


def box(sx, sy, sz, center=(0.0, 0.0, 0.0)):
    cx, cy, cz = center
    x0, x1, y0, y1, z0, z1 = cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2, cz - sz / 2, cz + sz / 2
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    triangles = []
    for a, b, c, d in faces:
        triangles.append((v[a], v[b], v[c]))
        triangles.append((v[a], v[c], v[d]))
    return triangles


def prism(polygon, y0, y1):
    """Extrudes a convex polygon of the XZ plane (counterclockwise seen from
    -Y) between y0 and y1: a fin."""
    triangles = []
    front = [(x, y0, z) for x, z in polygon]
    back = [(x, y1, z) for x, z in polygon]
    for i in range(1, len(polygon) - 1):
        triangles.append((front[0], front[i], front[i + 1]))
        triangles.append((back[0], back[i + 1], back[i]))
    n = len(polygon)
    for i in range(n):
        j = (i + 1) % n
        triangles.append((front[i], back[i], back[j]))
        triangles.append((front[i], back[j], front[j]))
    return triangles


def rotation_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def rotation_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def transform(triangles, rotation=None, translation=(0.0, 0.0, 0.0)):
    rotation = rotation or [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    out = []
    for triangle in triangles:
        out.append(tuple(tuple(sum(rotation[i][k] * p[k] for k in range(3)) + translation[i] for i in range(3))
                         for p in triangle))
    return out


def between(p0, p1, radius, segments=12):
    """A rod from p0 to p1."""
    d = [p1[i] - p0[i] for i in range(3)]
    length = math.sqrt(sum(x * x for x in d))
    pitch = math.acos(max(-1.0, min(1.0, d[2] / length)))
    yaw = math.atan2(d[1], d[0])
    rotation = multiply(rotation_z(yaw), rotation_y(pitch))
    return transform(cylinder(radius, 0.0, length, segments), rotation, p0)


def around(triangles, radius, angles, z=0.0):
    """Copies of a shape placed at a radius, turned with it."""
    out = []
    for angle in angles:
        out += transform(triangles, rotation_z(angle), (radius * math.cos(angle), radius * math.sin(angle), z))
    return out


def write_stl(path, triangles):
    with open(path, "wb") as f:
        f.write(b"RobotIK Apollo 11 model".ljust(80, b" "))
        f.write(struct.pack("<I", len(triangles)))
        for a, b, c in triangles:
            u = [b[i] - a[i] for i in range(3)]
            v = [c[i] - a[i] for i in range(3)]
            n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
            length = math.sqrt(sum(x * x for x in n)) or 1.0
            f.write(struct.pack("<3f", *[x / length for x in n]))
            for p in (a, b, c):
                f.write(struct.pack("<3f", *p))
            f.write(b"\0\0")


# --- URDF -------------------------------------------------------------------

class Link:
    def __init__(self, name):
        self.name = name
        self.visuals = []  # (geometry xml, color, origin xyz)

    def mesh(self, mesh_name, triangles, color):
        if triangles:
            write_stl(os.path.join(MESHES, mesh_name + ".stl"), triangles)
            self.visuals.append(('<mesh filename="meshes/%s.stl"/>' % mesh_name, color, (0, 0, 0)))

    def primitive(self, geometry, color, origin=(0, 0, 0)):
        self.visuals.append((geometry, color, origin))

    def xml(self, mass):
        lines = ['  <link name="%s">' % self.name]
        for i, (geometry, color, origin) in enumerate(self.visuals):
            lines.append('    <visual>')
            lines.append('      <origin xyz="%g %g %g" rpy="0 0 0"/>' % origin)
            lines.append('      <geometry>%s</geometry>' % geometry)
            lines.append('      <material name="%s_%d"><color rgba="%.3f %.3f %.3f 1"/></material>'
                         % (self.name, i, *color))
            lines.append('    </visual>')
        lines.append('    <inertial><mass value="%g"/><origin xyz="0 0 0"/>'
                     '<inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>' % mass)
        lines.append('  </link>')
        return "\n".join(lines)


def write_urdf(name, comment, base, mass, joints=()):
    """joints: (name, child link, xyz, yaw, lower, upper)."""
    lines = ['<?xml version="1.0"?>', '<!-- %s -->' % comment, '<robot name="%s">' % name, base.xml(mass)]
    for joint, child, xyz, yaw, lower, upper in joints:
        lines.append(child.xml(1.0))
        lines.append('  <joint name="%s" type="revolute">' % joint)
        lines.append('    <parent link="%s"/>' % base.name)
        lines.append('    <child link="%s"/>' % child.name)
        lines.append('    <origin xyz="%g %g %g" rpy="0 0 %g"/>' % (*xyz, yaw))
        lines.append('    <axis xyz="0 1 0"/>')
        lines.append('    <limit lower="%g" upper="%g" effort="1000" velocity="1"/>' % (lower, upper))
        lines.append('  </joint>')
    lines.append('</robot>')
    with open(os.path.join(DATA, name + ".urdf"), "w") as f:
        f.write("\n".join(lines) + "\n")


def quadrants(first):
    """Angle ranges of two opposite quadrants: the black and white roll
    pattern of the Saturn V."""
    return [(first + k * math.pi, first + k * math.pi + math.pi / 2) for k in (0, 1)]


# --- Elements ----------------------------------------------------------------

def s_ic(e):
    r, h = e["diameter"] / 2, e["length"]
    base = Link("base")
    body = cylinder(r, 0.0, h)
    fins, fairings, black = [], [], []
    for a0, a1 in quadrants(0.0):
        black += shell(r + 0.01, r + 0.01, 0.0, 6.5, 0.02, a0, a1, 12)
        black += shell(r + 0.01, r + 0.01, 17.0, 23.0, 0.02, a0, a1, 12)
        black += shell(r + 0.01, r + 0.01, h - 3.0, h, 0.02, a0 + math.pi / 2, a1 + math.pi / 2, 12)
    outer = [math.pi / 4 + k * math.pi / 2 for k in range(4)]
    fin = prism([(r - 0.2, -0.5), (r + 3.7, -0.5), (r + 3.7, 2.2), (r - 0.2, 7.5)], -0.2, 0.2)
    for angle in outer:
        fins += transform(fin, rotation_z(angle))
    fairings += around(frustum(1.95, 0.35, -0.5, 9.0, 24), 4.4, outer)
    bells = bell(0.55, 1.88, 1.0, e["engine"]["exit"])
    bells += around(bell(0.55, 1.88, 1.0, e["engine"]["exit"]), 3.9, outer)
    base.mesh("s_ic_white", body + fins + fairings, WHITE)
    base.mesh("s_ic_black", black, BLACK)
    base.mesh("s_ic_engines", bells, ENGINE)
    write_urdf("s_ic", "S-IC first stage, five F-1 engines, at scale", base, e["dry_mass"])


def interstage(e):
    r = e["diameter"] / 2
    base = Link("base")
    base.mesh("interstage_white", shell(r, r, 0.0, e["length"], 0.08), WHITE)
    write_urdf("interstage", "S-IC/S-II interstage ring, at scale", base, e["dry_mass"])


def s_ii(e):
    r, top = e["diameter"] / 2, 3.3
    tanks = 16.3
    base = Link("base")
    body = cylinder(r, 0.0, tanks) + shell(r, top, tanks, e["length"], 0.06)
    outer = [math.pi / 4 + k * math.pi / 2 for k in range(4)]
    bells = bell(0.35, 0.98, 0.0, e["engine"]["exit"])
    bells += around(bell(0.35, 0.98, 0.0, e["engine"]["exit"]), 2.65, outer)
    base.mesh("s_ii_white", body, WHITE)
    base.mesh("s_ii_engines", bells, ENGINE)
    write_urdf("s_ii", "S-II second stage, five J-2 engines, and the S-II/S-IVB interstage, at scale",
               base, e["dry_mass"])


def s_ivb(e):
    r = e["diameter"] / 2
    stage = 17.8
    base = Link("base")
    black = []
    for a0, a1 in quadrants(math.pi / 4):
        black += shell(r + 0.01, r + 0.01, 0.0, 3.0, 0.02, a0, a1, 12)
    black += around(box(1.0, 0.6, 2.3, (0.0, 0.0, 1.45)), r + 0.25, [0.0, math.pi])
    base.mesh("s_ivb_white", cylinder(r, 0.0, stage), WHITE)
    base.mesh("s_ivb_black", black, BLACK)
    base.mesh("s_ivb_iu", shell(r, r, stage, e["length"], 0.1), LIGHT)
    base.mesh("s_ivb_engine", bell(0.35, 0.98, 0.0, e["engine"]["exit"]), ENGINE)
    write_urdf("s_ivb", "S-IVB third stage, one J-2 engine, and the Instrument Unit, at scale",
               base, e["dry_mass"])


def sla(e):
    r0, h = e["diameter"] / 2, e["length"]
    r1 = 1.955
    hinge = e["panels"]["hinge"]
    rh = r0 + (r1 - r0) * hinge / h
    base = Link("base")
    base.mesh("sla_ring", shell(r0, rh, 0.0, hinge, 0.06), WHITE)
    joints = []
    for k in range(4):
        middle = math.pi / 4 + k * math.pi / 2
        link = Link("panel_%d" % (k + 1))
        panel = shell(rh, r1, hinge, h, 0.06, middle - math.pi / 4, middle + math.pi / 4, 12)
        hinge_point = (rh * math.cos(middle), rh * math.sin(middle), hinge)
        local = transform(transform(panel, None, tuple(-x for x in hinge_point)), rotation_z(-middle))
        link.mesh("sla_panel_%d" % (k + 1), local, WHITE)
        joints.append(("panel_%d" % (k + 1), link, hinge_point, middle, -0.05, 1.6))
    write_urdf("sla", "Spacecraft LM Adapter, four panels hinged at their base, at scale",
               base, e["dry_mass"], joints)


def lm_descent(e):
    width = e["diameter"]
    corner = (width / 2) / math.cos(math.pi / 8)
    base = Link("base")
    octagon = lathe([(0, 0.0), (corner, 0.0), (corner, e["length"]), (0, e["length"])], 8, math.pi / 8,
                    math.pi / 8 + 2 * math.pi)
    base.mesh("lm_descent_gold", octagon, GOLD)
    base.mesh("lm_descent_black", cylinder(2.0, -0.05, 0.0, 8), BLACK)
    base.mesh("lm_descent_engine", bell(0.35, 0.76, 0.35, e["engine"]["exit"]), ENGINE)
    legs = e["legs"]
    hinge_r, hinge_z = legs["hinge"]
    foot_r, foot_z = legs["foot"]
    foot = (foot_r - hinge_r, 0.0, foot_z - hinge_z)
    joints = []
    for k in range(4):
        angle = math.pi / 4 + k * math.pi / 2
        link = Link("leg_%d" % (k + 1))
        strut = between((0.0, 0.0, 0.0), foot, 0.09)
        strut += between((-0.4, 0.0, -0.45), (foot[0] * 0.55, 0.0, foot[2] * 0.55), 0.05)
        strut += between((-0.3, 0.6, 0.6), (foot[0] * 0.55, 0.0, foot[2] * 0.55), 0.04)
        strut += between((-0.3, -0.6, 0.6), (foot[0] * 0.55, 0.0, foot[2] * 0.55), 0.04)
        pad = transform(lathe([(0, -0.05), (0.47, -0.02), (0.47, 0.05), (0, 0.05)], 16), None, foot)
        probe = between((foot[0], 0.0, foot[2] - 0.05), (foot[0], 0.0, foot[2] - 1.7), 0.015, 6)
        link.mesh("lm_leg_%d_gold" % (k + 1), strut, GOLD)
        link.mesh("lm_leg_%d_pad" % (k + 1), pad + probe, GRAY)
        joints.append(("leg_%d" % (k + 1), link,
                       (hinge_r * math.cos(angle), hinge_r * math.sin(angle), hinge_z), angle,
                       legs["stowed"] - 0.1, 0.05))
    write_urdf("lm_descent", "LM Eagle descent stage, DPS engine, four landing legs (q = 0: deployed), at scale",
               base, e["dry_mass"], joints)


def lm_ascent(e):
    base = Link("base")
    gray, black = [], []
    gray += box(1.5, 2.4, 2.0, (-0.15, 0.0, 1.3))                                   # midsection
    gray += transform(cylinder(1.17, 0.0, 1.0), rotation_y(math.pi / 2), (0.55, 0.0, 1.4))  # crew cabin
    gray += around(cylinder(0.62, 0.3, 1.6, 20), 1.5, [math.pi / 2, -math.pi / 2])  # propellant tanks
    gray += box(0.3, 0.3, 0.3, (1.25, 1.25, 1.95)) + box(0.3, 0.3, 0.3, (1.25, -1.25, 1.95))
    gray += box(0.3, 0.3, 0.3, (-1.25, 1.25, 1.95)) + box(0.3, 0.3, 0.3, (-1.25, -1.25, 1.95))
    black += box(0.8, 2.0, 1.4, (-1.25, 0.0, 1.3))                                  # aft equipment bay
    black += box(0.06, 0.45, 0.4, (1.57, 0.45, 1.85)) + box(0.06, 0.45, 0.4, (1.57, -0.45, 1.85))
    black += cylinder(0.42, 2.3, e["length"], 24)                                    # docking tunnel
    black += transform(cylinder(0.3, 0.0, 0.08, 16), rotation_y(-0.6), (1.0, 0.0, 2.35))  # radar
    base.mesh("lm_ascent_gray", gray, GRAY)
    base.mesh("lm_ascent_black", black, BLACK)
    base.mesh("lm_ascent_engine", bell(0.18, 0.45, 0.25, e["engine"]["exit"], 0.03, 20), ENGINE)
    write_urdf("lm_ascent", "LM Eagle ascent stage, APS engine, docking drogue on top, at scale",
               base, e["dry_mass"])


def sm(e):
    r, h = e["diameter"] / 2, e["length"]
    base = Link("base")
    quads = around(box(0.35, 0.7, 0.9, (0.0, 0.0, 0.0)), r + 0.15,
                   [k * math.pi / 2 + 0.13 for k in range(4)], h - 1.0)
    base.mesh("sm_silver", cylinder(r, 0.15, h), SILVER)
    base.mesh("sm_dark", cylinder(r, 0.0, 0.15) + quads, BLACK)
    base.mesh("sm_engine", bell(0.45, 1.25, 0.0, e["engine"]["exit"]), ENGINE)
    write_urdf("sm", "Service Module, SPS engine and RCS quads, at scale", base, e["dry_mass"])


def cm(e):
    r, h = e["diameter"] / 2, e["length"]
    base = Link("base")
    shield = lathe([(0, -0.35), (1.6, -0.25), (r, 0.0), (r, 0.06), (0, 0.06)])
    cone = lathe([(0, 0.06), (r, 0.06), (0.42, h - 0.15), (0.42, h), (0, h)])
    probe = cylinder(0.06, h, 3.45, 12) + frustum(0.06, 0.01, 3.45, 3.55, 12)
    base.mesh("cm_shield", shield, SHIELD)
    base.mesh("cm_silver", cone, SILVER)
    base.mesh("cm_probe", probe, GRAY)
    write_urdf("cm", "Command Module Columbia, docking probe on top, at scale", base, e["dry_mass"])


def les(e):
    base = Link("base")
    cover = frustum(2.0, 0.5, -0.05, 3.35)
    tower = []
    for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        tower += between((0.55 * sx, 0.55 * sy, 3.35), (0.25 * sx, 0.25 * sy, 6.2), 0.05, 8)
    tower += shell(0.6, 0.6, 3.3, 3.45, 0.05) + shell(0.3, 0.3, 6.1, 6.25, 0.05)
    motor = cylinder(0.33, 6.2, 9.4) + frustum(0.33, 0.1, 9.4, 10.0) + cylinder(0.05, 10.0, e["length"], 8)
    base.mesh("les_cover", cover + motor, WHITE)
    base.mesh("les_tower", tower, ORANGE)
    write_urdf("les", "Launch Escape System: tower, motor and boost protective cover, at scale",
               base, e["dry_mass"])


def chutes():
    """Three main parachutes, 25.4 m wide, 30 m of lines above the apex of
    the CM: the root is the point where the lines meet."""
    base = Link("base")
    radius, lines = 12.7, 30.0
    orange, white = [], []
    for k in range(3):
        tilt = 0.32
        heading = k * 2 * math.pi / 3
        rotation = multiply(rotation_z(heading), rotation_y(tilt))
        canopy = []
        profile = [(radius * math.sin(a), lines + radius * 0.55 * math.cos(a) - radius * 0.55)
                   for a in [math.pi / 2 * (1 - i / 10) for i in range(11)]]
        canopy_profile = profile + [(max(0.0, r - 0.05), z - 0.02) for r, z in reversed(profile)]
        for gore in range(12):
            a0 = gore * 2 * math.pi / 12
            piece = lathe(canopy_profile, 3, a0, a0 + 2 * math.pi / 12)
            (orange if gore % 2 == 0 else white).extend(transform(piece, rotation))
        for i in range(8):
            a = i * 2 * math.pi / 8
            rim = (radius * math.cos(a), radius * math.sin(a), lines - radius * 0.55)
            white.extend(between((0.0, 0.0, 0.0), tuple(sum(rotation[m][n] * rim[n] for n in range(3))
                                                         for m in range(3)), 0.04, 4))
    base.mesh("chutes_orange", orange, ORANGE)
    base.mesh("chutes_white", white, WHITE)
    write_urdf("chutes", "Three main parachutes of the CM, 25.4 m wide, at scale", base, 1.0)


def bodies():
    earth = Link("base")
    earth.primitive('<sphere radius="1"/>', (0.18, 0.36, 0.70))
    write_urdf("earth", "The Earth, unit sphere scaled by the plugin", earth, 1.0)
    moon = Link("base")
    moon.primitive('<sphere radius="1"/>', (0.62, 0.61, 0.58))
    write_urdf("moon", "The Moon, unit sphere scaled by the plugin", moon, 1.0)
    for name, color in (("earth_ground", (0.22, 0.36, 0.42)), ("moon_ground", (0.55, 0.54, 0.51))):
        ground = Link("base")
        ground.primitive('<cylinder radius="150000" length="20"/>', color, (0, 0, -10))
        write_urdf(name, "A flat disc, 150 km wide, under a vehicle close to the ground", ground, 1.0)


def main():
    with open(os.path.join(DATA, "vehicles.yaml")) as f:
        spec = yaml.safe_load(f)["elements"]
    os.makedirs(MESHES, exist_ok=True)
    for name, build in (("s_ic", s_ic), ("interstage", interstage), ("s_ii", s_ii), ("s_ivb", s_ivb),
                        ("sla", sla), ("lm_descent", lm_descent), ("lm_ascent", lm_ascent), ("sm", sm),
                        ("cm", cm), ("les", les)):
        build(spec[name])
    chutes()
    bodies()
    return 0


if __name__ == "__main__":
    sys.exit(main())
