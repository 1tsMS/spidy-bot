#!/usr/bin/env python3
"""
Fusion STL exports  ->  per-link meshes + mass properties for the Spidy URDF.

Why this script exists
----------------------
URDF needs every link's mesh expressed in *that link's own frame* (origin at its
joint), plus mass / centre of mass / inertia. Fusion gives us one STL per
component in the component's local frame, and the 4x4 transform that places it
in the assembly. So we:

  1. put every component into world coordinates (apply its Fusion transform)
  2. group components into URDF links (servo body rides with the bracket it's
     screwed into, horn rides with the link it's screwed to)
  3. shift each group so its joint becomes the origin  -> link-frame mesh
  4. compute mass properties from the mesh volumes + masses.yaml

Re-run after you weigh parts and edit config/masses.yaml:
    python scripts/build_description.py --raw <folder with Fusion STLs>
"""
import argparse, json, os
import numpy as np
import trimesh
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)

# ---------------------------------------------------------------- geometry
# All measured off the Fusion model (horn hub circles), in metres, world frame.
BASE_ORIGIN = np.array([0.060, 0.030, 0.0382])   # body centre, at hip-pitch/knee height
HIP_YAW_XY  = np.array([0.060, 0.050])           # |x|,|y| of hip-yaw axes from body centre
COXA_LEN    = 0.0342                             # hip-yaw axis -> hip-pitch axis (along y)
FEMUR_LEN   = 0.0558                             # hip-pitch axis -> knee axis (along y)

LEGS = {  # name: (sx, sy)   sx=+1 front, sy=+1 left
    "fl": (+1, +1), "fr": (+1, -1), "bl": (-1, +1), "br": (-1, -1),
}

# Which Fusion components belong to which URDF link
def leg_groups(L):
    return {
        f"{L}_coxa":  [f"shoulder_{L}", f"m1_horn_{L}", f"m2_{L}"],
        f"{L}_femur": [f"elbow_{L}", f"m2_horn_{L}", f"m3_horn_{L}"],
        f"{L}_tibia": [f"claw_{L}", f"m3_{L}"],
    }
BASE_GROUP = ["base_link", "TABLE", "Battery", "BUCK", "PCA", "MCU_board",
              "m1_fl", "m1_fr", "m1_bl", "m1_br"]


def joint_frames(sx, sy):
    yaw = BASE_ORIGIN + np.array([sx * HIP_YAW_XY[0], sy * HIP_YAW_XY[1], 0.0])
    pitch = yaw + np.array([0.0, sy * COXA_LEN, 0.0])
    knee = pitch + np.array([0.0, sy * FEMUR_LEN, 0.0])
    return yaw, pitch, knee


def load_world(raw, name, T):
    m = trimesh.load(os.path.join(raw, name + ".stl"))
    M = np.array(T[name]).reshape(4, 4).copy()
    M[:3, 3] *= 10.0                       # Fusion API translation is cm, STL is mm
    m.apply_transform(M)
    m.apply_scale(0.001)                   # mm -> m
    return m


def part_mass_kg(name, mesh, cfg):
    p = cfg["parts"]
    key = name
    for k in ("shoulder", "elbow", "claw"):
        if name.startswith(k + "_"):
            key = k
    if name.startswith("m") and "_horn_" in name:
        key = "horn"
    elif name[:3] in ("m1_", "m2_", "m3_"):
        key = "servo"
    spec = p[key]
    if spec.get("printed"):
        vol_cm3 = mesh.volume * 1e6
        return vol_cm3 * cfg["pla_effective_density_g_cm3"] / 1000.0
    return spec["grams"] / 1000.0


def combine(parts):
    """parts: list of (mass, com[3], I_about_com[3x3]) -> combined (m, com, I)."""
    M = sum(p[0] for p in parts)
    c = sum(p[0] * p[1] for p in parts) / M
    I = np.zeros((3, 3))
    for m, ci, Ii in parts:
        d = ci - c
        I += Ii + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return M, c, I


def mass_props(meshes_named, cfg, origin, extra=()):
    parts = []
    for name, mesh in meshes_named:
        m = part_mass_kg(name, mesh, cfg)
        mesh = mesh.copy()
        mesh.density = m / mesh.volume
        parts.append((m, mesh.center_mass - origin, mesh.moment_inertia))
    parts += list(extra)
    return combine(parts)


def fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", required=True, help="folder with Fusion per-component STLs")
    args = ap.parse_args()

    T = json.load(open(os.path.join(PKG, "config", "fusion_transforms.json")))
    cfg = yaml.safe_load(open(os.path.join(PKG, "config", "masses.yaml")))
    meshdir = os.path.join(PKG, "meshes")
    os.makedirs(meshdir, exist_ok=True)

    report = {}

    # ---- base link
    base_parts = [(n, load_world(args.raw, n, T)) for n in BASE_GROUP]
    misc = cfg["misc_on_base_g"] / 1000.0
    misc_pt = (misc, np.array([0.0, 0.0, -0.018]), np.zeros((3, 3)))
    mb, cb, Ib = mass_props(base_parts, cfg, BASE_ORIGIN, extra=[misc_pt])
    base_mesh = trimesh.util.concatenate([m for _, m in base_parts])
    base_mesh.apply_translation(-BASE_ORIGIN)
    base_mesh.export(os.path.join(meshdir, "base_link.stl"))
    # collision boxes: chassis plate + electronics/battery stack
    plate = base_parts[0][1].bounds - BASE_ORIGIN
    stack = trimesh.util.concatenate([m for n, m in base_parts if n in ("TABLE", "Battery", "PCA", "MCU_board", "BUCK")]).bounds - BASE_ORIGIN
    report["base"] = dict(mass=mb, com=cb, I=Ib, plate=plate, stack=stack)

    # ---- legs
    legs = {}
    for L, (sx, sy) in LEGS.items():
        yaw, pitch, knee = joint_frames(sx, sy)
        origins = {f"{L}_coxa": yaw, f"{L}_femur": pitch, f"{L}_tibia": knee}
        legs[L] = {}
        for link, comps in leg_groups(L).items():
            named = [(n, load_world(args.raw, n, T)) for n in comps]
            o = origins[link]
            m, c, I = mass_props(named, cfg, o)
            mesh = trimesh.util.concatenate([mm for _, mm in named])
            mesh.apply_translation(-o)
            mesh.export(os.path.join(meshdir, f"{link}.stl"))
            legs[L][link.split("_")[1]] = dict(mass=m, com=c, I=I)
            if link.endswith("tibia"):
                claw = dict(named)[f"claw_{L}"].copy()
                claw.apply_translation(-o)
                v = claw.vertices
                low = v[v[:, 2] < v[:, 2].min() + 0.0005]
                legs[L]["foot"] = low.mean(axis=0)

    # ---- mirror check: every leg must equal FL mirrored by (sx, sy)
    fl = legs["fl"]
    for L, (sx, sy) in LEGS.items():
        S = np.array([sx, sy, 1.0])
        for seg in ("coxa", "femur", "tibia"):
            a, b = fl[seg], legs[L][seg]
            assert abs(a["mass"] - b["mass"]) < 1e-6, (L, seg)
            assert np.allclose(a["com"] * S, b["com"], atol=2e-4), (L, seg, a["com"], b["com"])
            assert np.allclose(np.abs(a["I"]), np.abs(b["I"]), rtol=0.05, atol=1e-9), (L, seg)
        assert np.allclose(fl["foot"] * S, legs[L]["foot"], atol=1e-3), (L, fl["foot"], legs[L]["foot"])
    print("mirror check OK: all four legs are FL mirrored")

    # ---- write generated xacro (numbers only; structure lives in spidy.urdf.xacro)
    def inertial_props(prefix, d):
        I = d["I"]
        return (f'  <xacro:property name="{prefix}_mass" value="{d["mass"]:.6g}"/>\n'
                f'  <xacro:property name="{prefix}_cx" value="{d["com"][0]:.6g}"/>\n'
                f'  <xacro:property name="{prefix}_cy" value="{d["com"][1]:.6g}"/>\n'
                f'  <xacro:property name="{prefix}_cz" value="{d["com"][2]:.6g}"/>\n'
                f'  <xacro:property name="{prefix}_ixx" value="{I[0,0]:.4e}"/>\n'
                f'  <xacro:property name="{prefix}_iyy" value="{I[1,1]:.4e}"/>\n'
                f'  <xacro:property name="{prefix}_izz" value="{I[2,2]:.4e}"/>\n'
                f'  <xacro:property name="{prefix}_ixy" value="{I[0,1]:.4e}"/>\n'
                f'  <xacro:property name="{prefix}_ixz" value="{I[0,2]:.4e}"/>\n'
                f'  <xacro:property name="{prefix}_iyz" value="{I[1,2]:.4e}"/>\n')

    b = report["base"]
    pc = b["plate"].mean(axis=0); ps = b["plate"][1] - b["plate"][0]
    sc = b["stack"].mean(axis=0); ss = b["stack"][1] - b["stack"][0]
    out = ['<?xml version="1.0"?>',
           '<!-- GENERATED by scripts/build_description.py from Fusion + config/masses.yaml. Do not hand-edit. -->',
           '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">',
           f'  <xacro:property name="hip_x" value="{HIP_YAW_XY[0]}"/>',
           f'  <xacro:property name="hip_y" value="{HIP_YAW_XY[1]}"/>',
           f'  <xacro:property name="coxa_len" value="{COXA_LEN}"/>',
           f'  <xacro:property name="femur_len" value="{FEMUR_LEN}"/>',
           f'  <xacro:property name="foot_x" value="{fl["foot"][0]:.5f}"/>',
           f'  <xacro:property name="foot_y" value="{fl["foot"][1]:.5f}"/>',
           f'  <xacro:property name="foot_z" value="{fl["foot"][2]:.5f}"/>',
           f'  <xacro:property name="plate_box_xyz"  value="{fmt(pc)}"/>',
           f'  <xacro:property name="plate_box_size" value="{fmt(ps)}"/>',
           f'  <xacro:property name="stack_box_xyz"  value="{fmt(sc)}"/>',
           f'  <xacro:property name="stack_box_size" value="{fmt(ss)}"/>',
           inertial_props("base", b)]
    for seg in ("coxa", "femur", "tibia"):
        out.append(inertial_props(seg, fl[seg]))
    out.append("</robot>\n")
    open(os.path.join(PKG, "urdf", "spidy_generated.xacro"), "w").write("\n".join(out))

    total = b["mass"] + 4 * sum(fl[s]["mass"] for s in ("coxa", "femur", "tibia"))
    print(f"base {b['mass']*1000:.1f} g | per leg: coxa {fl['coxa']['mass']*1000:.1f} g, "
          f"femur {fl['femur']['mass']*1000:.1f} g, tibia {fl['tibia']['mass']*1000:.1f} g | total {total*1000:.0f} g")
    print("base COM (m, base frame):", np.round(b["com"], 4))
    print("FL foot tip in tibia frame (m):", np.round(fl["foot"], 4))


if __name__ == "__main__":
    main()
