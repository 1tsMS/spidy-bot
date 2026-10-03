"""Make the MuJoCo scene look like a studio render. Visual only: no physics changes.

  floor      dark blueprint grid (repaints the model's checker texture)
  robot      pale body, graphite legs
  light      softer headlight
  background gradient + vignette are painted by the app's SimView (Qt), not here
Call style(sim) right after SimWorld() and before its first render, because the
floor texture is uploaded to the GPU when the renderer is created.
"""
import numpy as np

from . import theme


def _rgb(hex_color):
    return np.array([int(hex_color[i:i + 2], 16) for i in (1, 3, 5)], np.uint8)


def _grid_texture(n=512, minor=4):
    """One floor tile (0.2 m): dark base, faint 5 cm lines, brighter tile edge."""
    img = np.empty((n, n, 3), np.uint8)
    img[:] = _rgb("#14171d")
    step = n // minor
    for k in range(minor):
        img[k * step:k * step + 2, :] = _rgb("#1e232c")
        img[:, k * step:k * step + 2] = _rgb("#1e232c")
    img[:3, :] = _rgb("#2c3340")
    img[:, :3] = _rgb("#2c3340")
    return img


def style(sim):
    m = sim.m
    tid = 0                                       # the only texture in the model: the floor grid
    w, h = int(m.tex_width[tid]), int(m.tex_height[tid])
    adr = int(m.tex_adr[tid])
    m.tex_data[adr:adr + w * h * 3] = _grid_texture(w).reshape(-1)

    for g in range(m.ngeom):
        if m.geom_contype[g] != 0 or m.geom_bodyid[g] == 0:
            continue                              # only visual meshes of the robot
        body = m.body(m.geom_bodyid[g]).name
        m.geom_rgba[g] = (0.56, 0.59, 0.64, 1) if body == "base_link" else (0.34, 0.37, 0.43, 1)

    m.vis.headlight.ambient[:] = 0.22
    m.vis.headlight.diffuse[:] = 0.42
    m.vis.headlight.specular[:] = 0.25
    sim._rgba0 = m.geom_rgba.copy()               # highlight() resets to these colours
    sim._mat0 = m.geom_matid.copy()
    sim.cam.elevation = -20
    sim.cam.distance = 0.6


HIGHLIGHT = theme.rgba(theme.ACCENT)
