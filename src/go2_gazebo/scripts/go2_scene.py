#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
go2_scene.py -- generate the static indoor scene for the Go2 patrol world.

Two jobs share one "box emitter" so the default warehouse and an imported
floor plan end up in the same SDF style (a handful of static <model>s, each a
single <link> holding many <visual>+<collision> boxes -- cheap for gz-sim and
cheap on an iGPU).

Subcommands
-----------
sample
    Rewrite the SCENE region of the default scenery world (go2_patrol.sdf)
    with a synthetic indoor *warehouse* layout: perimeter walls with door gaps,
    a couple of partition walls, two aisles flanked by racking, crates/pallets
    and a small workroom. Idempotent -- only the text between the
    <!-- SCENE_BEGIN --> / <!-- SCENE_END --> markers is replaced.

import
    Read a standard PGM occupancy grid (value >= threshold = occupied) and turn
    the occupied cells into wall rectangles (run merging), emitting them into
    the same SCENE region. This is the "bring your own floor plan" path.
    Optional --apply-to <file.sdf> writes to a *copy* so the committed default
    world is never damaged.

Usage
-----
  # regenerate the default warehouse layout (dev time; result is committed)
  python3 go2_scene.py sample --world worlds/go2_patrol.sdf

  # turn an occupancy grid into walls (dry-run: print model count)
  python3 go2_scene.py import --pgm scene/sample_floorplan.pgm --res 0.3

  # ...and splice it into a copy of the world, ready for go2_urdf2sdf.py
  python3 go2_scene.py import --pgm my_map.pgm --res 0.05 \
      --apply-to /tmp/my_world.sdf --off-x 2.0 --off-y -1.0
"""

import argparse
import os
import re
import sys


# --------------------------------------------------------------------------- #
# world-region splicing
# --------------------------------------------------------------------------- #
SCENE_BEGIN = '<!-- ============ SCENE_BEGIN (go2_scene.py managed) ============ -->'
SCENE_END = '<!-- ============ SCENE_END ============ -->'

_SCENE_RE = re.compile(
    r'(?ms)\n?<!--\s*={2,}\s*SCENE_BEGIN.*?SCENE_END\s*={2,}\s*-->\s*\n?')


def rewrite_scene_region(world_text, inner_xml):
    """Replace everything between the SCENE_BEGIN/SCENE_END markers."""
    body = SCENE_BEGIN + '\n' + inner_xml.rstrip('\n') + '\n' + SCENE_END + '\n'
    new, n = _SCENE_RE.subn(body, world_text, count=1)
    if n != 1:
        sys.exit('error: world has no SCENE_BEGIN/SCENE_END markers to replace')
    return new


# --------------------------------------------------------------------------- #
# box emitter: a list of box records -> SDF static-model XML
# --------------------------------------------------------------------------- #
# Box record: (cx, cy, sx, sy, sz, (r, g, b))  -- centre on the floor plane,
# so the box rises from z=0 to z=sz.  All colours are flat (flow-first render).


def _model_xml(boxes, model_name='scene_indoor', link_name='fixed'):
    out = ['    <model name="%s">' % model_name,
           '      <static>true</static>',
           '      <link name="%s">' % link_name]
    for i, rec in enumerate(boxes):
        cx, cy, sx, sy, sz, col = rec[0], rec[1], rec[2], rec[3], rec[4], rec[5]
        cz = rec[6] if len(rec) > 6 else sz / 2.0   # explicit z-centre override
        out.append('        <visual name="vis_%d">' % i)
        out.append('          <pose>%.4f %.4f %.4f 0 0 0</pose>' % (cx, cy, cz))
        out.append('          <geometry><box><size>%.4f %.4f %.4f</size></box></geometry>'
                   % (sx, sy, sz))
        out.append('          <material>')
        out.append('            <ambient>%s 1</ambient>' % ' '.join('%.3f' % c for c in col))
        out.append('            <diffuse>%s 1</diffuse>' % ' '.join('%.3f' % c for c in col))
        out.append('          </material>')
        out.append('        </visual>')
        out.append('        <collision name="col_%d">' % i)
        out.append('          <pose>%.4f %.4f %.4f 0 0 0</pose>' % (cx, cy, cz))
        out.append('          <geometry><box><size>%.4f %.4f %.4f</size></box></geometry>'
                   % (sx, sy, sz))
        out.append('        </collision>')
    out.append('      </link>')
    out.append('    </model>')
    return '\n'.join(out)


def add_axis_run(boxes, axis, fixed, start, end, th, col, h=3.0):
    """Straight wall run.  axis is the direction the run extends along;
    `fixed` is the value of the *other* coordinate.  Thickness `th` is taken
    perpendicular to the run; the wall rises from z=0 to z=h.
    axis='x': runs along x at constant y=fixed  -> size (length, th, h)
    axis='y': runs along y at constant x=fixed  -> size (th, length, h)
    """
    length = abs(end - start)
    lo, hi = min(start, end), max(start, end)
    mid = (lo + hi) / 2.0
    if axis == 'x':
        boxes.append((mid, fixed, length, th, h, col))
    else:
        boxes.append((fixed, mid, th, length, h, col))


def add_box(boxes, cx, cy, sx, sy, sz, col, cz=None):
    """Add a box.  Default z-centre = sz/2 (sits on the floor); pass cz to lift it."""
    if cz is None:
        boxes.append((cx, cy, sx, sy, sz, col))
    else:
        boxes.append((cx, cy, sx, sy, sz, col, cz))


# --------------------------------------------------------------------------- #
# sample warehouse layout
# --------------------------------------------------------------------------- #
# colours
WALL = (0.76, 0.76, 0.75)        # concrete grey
PART = (0.80, 0.79, 0.77)        # lighter partition
RACK = (0.22, 0.30, 0.42)        # steel blue cabinet
RACK_SHELF = (0.13, 0.18, 0.26)  # shelf lines on the aisle face
CRATE = (0.72, 0.56, 0.33)       # wood-pallet tan
PILLAR = (0.60, 0.62, 0.65)
DESK = (0.45, 0.50, 0.58)

HALF_X, HALF_Y = 10.0, 7.0       # footprint extends to +/- these (perimeter)
H = 3.0                          # wall height
T = 0.30                         # wall thickness
DOOR = 1.2                       # nominal door-gap width


def build_warehouse():
    """Return the box list of the default indoor warehouse scene.

    Geometry intent (robot spawns at the origin facing +x, in a clear central
    hall / aisle): perimeter wall with two loading-bay door gaps, one interior
    partition wall with a door, two rooms to the -x side, two racking rows
    flanking the +x aisle, pallet stacks, pillars.  The origin stays open so
    the patrol demo (straight +x then a curve) runs between the racks.
    """
    bx = []
    y = HALF_Y
    # ---- perimeter (with door gaps) --------------------------------------
    add_axis_run(bx, 'x', y, -HALF_X, -DOOR, T, WALL, H)          # north wall
    add_axis_run(bx, 'x', y, DOOR, HALF_X, T, WALL, H)
    add_axis_run(bx, 'x', -y, -HALF_X, -HALF_Y + 2.0 - DOOR, T, WALL, H)  # south
    add_axis_run(bx, 'x', -y, -HALF_Y + 2.0, HALF_X, T, WALL, H)
    add_axis_run(bx, 'y', -HALF_X, -y, y, T, WALL, H)             # west wall
    add_axis_run(bx, 'y', HALF_X, -y, y, T, WALL, H)              # east wall

    # ---- interior partition: door into the west work rooms ----------------
    add_axis_run(bx, 'y', -3.0, -y, -DOOR / 2.0, 0.24, PART, H)
    add_axis_run(bx, 'y', -3.0, DOOR / 2.0, y, 0.24, PART, H)

    # ---- racks flanking the main aisle (open floor around origin) ---------
    for cy in (2.3, -2.3):
        for cx in (2.6, 5.6, 8.2):
            add_box(bx, cx, cy, 2.4, 0.8, 2.4, RACK)
            s = -1.0 if cy > 0 else 1.0            # shelf lines on aisle face
            fy = cy + s * 0.41
            for hz in (0.6, 1.2, 1.8):
                add_box(bx, cx, fy, 2.4, 0.02, 0.05, RACK_SHELF, cz=hz)
    # far racks in the north/south rooms (visible through the partition door)
    for cx in (5.6,):
        add_box(bx, cx, 4.6, 2.4, 0.8, 2.4, RACK)
    for cx in (5.6,):
        add_box(bx, cx, -4.6, 2.4, 0.8, 2.4, RACK)

    # ---- crates / pallets ------------------------------------------------
    for cx, cy in ((-6.5, -4.2), (-7.5, -1.6), (-5.2, 0.6)):
        add_box(bx, cx, cy, 0.9, 0.9, 0.9, CRATE)
    add_box(bx, -7.0, -4.2, 0.9, 0.9, 0.45, CRATE)  # low stack on top
    # a few pallets against the east wall (visible at the far end of the aisle)
    for cx, cy in ((8.8, 1.6), (8.8, -1.6)):
        add_box(bx, cx, cy, 1.2, 1.0, 0.25, CRATE)
        add_box(bx, cx, cy, 0.7, 0.7, 1.0, RACK)

    # ---- pillars near the four obstacle corners of the hall ----------------
    for cx, cy in ((1.2, 1.2), (1.2, -1.2)):
        add_box(bx, cx, cy, 0.4, 0.4, 3.0, PILLAR)

    # ---- west work-room furniture (a low bench + chair-ish boxes) ----------
    add_box(bx, -8.2, 3.2, 2.6, 1.0, 0.8, DESK)
    add_box(bx, -6.4, 3.2, 0.5, 0.5, 0.5, CRATE)

    return bx


# --------------------------------------------------------------------------- #
# PGM occupancy grid -> wall boxes
# --------------------------------------------------------------------------- #
def read_pgm(path):
    """Read a P2 (ascii) or P5 (binary) PGM, return (width, height, bytes/str rows).
    Pixels: value >= 255 = occupied here is not assumed; caller thresholds."""
    with open(path, 'rb') as fh:
        magic = fh.readline().strip()               # b'P2' / b'P5'
        if magic not in (b'P2', b'P5'):
            sys.exit('error: %s is not a P2/P5 PGM (magic %r)' % (path, magic))
        # skip comment lines
        line = fh.readline()
        while line.startswith(b'#'):
            line = fh.readline()
        dims = line.split()
        if len(dims) < 2:                            # width/height spilled over
            dims += fh.readline().split()
        w, h = int(dims[0]), int(dims[1])
        maxv = int(fh.readline().split()[0])
        if magic == b'P5':
            data = fh.read()
        else:
            data = fh.read().decode('ascii')
        return w, h, maxv, data


def _px(w, h, data, binary, col, row):
    if binary:
        return data[row * w + col]
    toks = data.split()
    return int(toks[row * w + col])


def occupied_to_boxes(w, h, data, binary, thresh=100, res=0.30, height=3.0,
                      wall_col=WALL, off_x=0.0, off_y=0.0, invert=False):
    """Merge occupied cells into axis-aligned wall rectangles (greedy
    row-run growth). World origin = centre of the grid + (off_x, off_y).

    Occupancy convention follows ROS map_server PGM: a dark pixel (low value)
    is occupied, free space is bright.  Bright (high value) pixels can be
    marked occupied instead by setting invert=True.
    """
    occ = set()
    for r in range(h):
        for c in range(w):
            v = _px(w, h, data, binary, c, r)
            if (v >= thresh) if invert else (v <= thresh):
                occ.add((c, r))
    # world coords: col c -> x = c*res, row r (top-down) -> y = (h-1-r)*res,
    # then shift so grid centre sits at the origin.
    cx0 = (w - 1) * res / 2.0
    cy0 = (h - 1) * res / 2.0
    rects = []
    remaining = set(occ)
    while remaining:
        # pick the topmost-leftmost remaining cell as the seed
        c0, r0 = min(remaining)
        # grow the row-run to the right
        c1 = c0
        while (c1 + 1, r0) in remaining:
            c1 += 1
        # grow downward while every cell of the whole run [c0..c1]x{r} is free
        r1 = r0
        grow = True
        while grow and r1 + 1 < h:
            for cc in range(c0, c1 + 1):
                if (cc, r1 + 1) not in remaining:
                    grow = False
                    break
            if grow:
                r1 += 1
        for rr in range(r0, r1 + 1):
            for cc in range(c0, c1 + 1):
                remaining.discard((cc, rr))
        wpx = c1 - c0 + 1
        hpx = r1 - r0 + 1
        wx = (c0 + (wpx - 1) / 2.0) * res - cx0 + off_x
        wy = cy0 - ((h - 1 - r0) + (hpx - 1) / 2.0) * res + off_y
        rects.append((wx, wy, wpx * res, hpx * res))
    return rects


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
def _scene_xml_for(boxes):
    return _model_xml(boxes) + '\n'


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    p_sample = sub.add_parser('sample', help='regenerate default warehouse scene')
    p_sample.add_argument('--world', required=True,
                          help='scenery world .sdf whose SCENE region is rewritten')

    p_import = sub.add_parser('import', help='PGM occupancy grid -> walls')
    p_import.add_argument('--pgm', required=True)
    p_import.add_argument('--res', type=float, default=0.30, help='m per pixel')
    p_import.add_argument('--height', type=float, default=3.0)
    p_import.add_argument('--thresh', type=int, default=100,
                          help='pixel <= this value is occupied (dark = wall, '
                               'like ROS map_server)')
    p_import.add_argument('--invert', action='store_true',
                          help='treat bright pixels (>= --thresh) as occupied')
    p_import.add_argument('--off-x', type=float, default=0.0)
    p_import.add_argument('--off-y', type=float, default=0.0)
    p_import.add_argument('--apply-to', default=None,
                          help='world .sdf to splice into (writes a modified copy '
                               'in place; make a copy yourself to keep the default)')
    args = ap.parse_args()

    if args.cmd == 'sample':
        boxes = build_warehouse()
        with open(args.world, 'r', encoding='utf-8') as fh:
            world = fh.read()
        out = rewrite_scene_region(world, _scene_xml_for(boxes))
        with open(args.world, 'w', encoding='utf-8') as fh:
            fh.write(out)
        print('sample: wrote %d boxes into %s' % (len(boxes), args.world))

    else:  # import
        w, h, _maxv, data = read_pgm(args.pgm)
        binary = open(args.pgm, 'rb').readline().strip() == b'P5'
        rects = occupied_to_boxes(w, h, data, binary, args.thresh, args.res,
                                  args.height, WALL, args.off_x, args.off_y,
                                  args.invert)
        boxes = [(cx, cy, sx, sy, args.height, WALL)
                 for (cx, cy, sx, sy) in rects]
        xml = _scene_xml_for(boxes)
        if args.apply_to:
            with open(args.apply_to, 'r', encoding='utf-8') as fh:
                world = fh.read()
            with open(args.apply_to, 'w', encoding='utf-8') as fh:
                fh.write(rewrite_scene_region(world, xml))
            print('import: %d walls (%dx%d px @%.2f m/px) spliced into %s'
                  % (len(rects), w, h, args.res, args.apply_to))
        else:
            print('import: %d walls from %dx%d px @%.2f m/px' % (len(rects), w, h, args.res))
            print(xml)


if __name__ == '__main__':
    main()
