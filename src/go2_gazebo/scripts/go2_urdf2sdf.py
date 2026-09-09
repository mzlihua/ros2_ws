#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
go2_urdf2sdf.py  --  Unitree Go2 URDF -> Gazebo Sim (gz-sim) world converter
                    for the "patrol mode" simulation.

Reads the official `go2.urdf` description (base + 4 legs + sensors declared as
<gazebo> blocks), and emits a gz-sim SDF <model name="go2"> that is injected
into the patrol scenery world (go2_patrol.sdf) at the <!--GO2_MODEL_INSERT_HERE-->
marker.

Patrol-mode choices encoded here:
  * dartsim world with zero gravity (world file) -> the body is moved only by the
    VelocityControl system; it never falls, tips or contacts the ground.
  * legs are driven by a JointTrajectoryController (one topic per whole robot)
    so a ROS2 driver can animate a walking gait via trajectory_msgs/JointTrajectory.
  * collisions are intentionally *omitted* so swinging feet can never snag the
    ground plane or obstacles (pure kinematic patrol).
  * sensors (imu / 2D lidar / rgb+depth cameras) from the URDF <gazebo> blocks
    are emitted as native SDF <sensor> elements.

Usage:
  python3 go2_urdf2sdf.py \
      --urdf   <path/go2.urdf> \
      --share  <install/share/go2_description> \
      --scenery <go2_patrol.sdf> \
      --out    <generated_world.sdf> \
      [--spawn-z 0.32] [--yaw 0.0] [--lidar-z 0.10]
"""

import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET


# --------------------------------------------------------------------------- #
# small XML helpers
# --------------------------------------------------------------------------- #
def sub(parent, tag, text=None, **attr):
    el = ET.SubElement(parent, tag)
    for k, v in attr.items():
        el.set(k, v)
    if text is not None:
        el.text = text
    return el


def pos3(xyz, rpy):
    """return "x y z r p y" string"""
    def f(v):
        return '%.6g' % v
    return ' '.join([f(xyz[0]), f(xyz[1]), f(xyz[2]),
                     f(rpy[0]), f(rpy[1]), f(rpy[2])])


def parse_pose(el):
    """parse <origin xyz rpy> or return identity"""
    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]
    if el is not None:
        if el.get('xyz'):
            xyz = [float(x) for x in el.get('xyz').split()]
        if el.get('rpy'):
            rpy = [float(x) for x in el.get('rpy').split()]
    return xyz, rpy


def num3(s, default=(1.0, 1.0, 1.0)):
    if not s:
        return list(default)
    return [float(x) for x in s.split()]


def add_geometry(parent, geom, material_rgba):
    """URDF <geometry> -> SDF <geometry>, returns element"""
    g = sub(parent, 'geometry')
    mesh = geom.find('mesh')
    box = geom.find('box')
    cyl = geom.find('cylinder')
    sph = geom.find('sphere')
    if mesh is not None:
        m = sub(g, 'mesh')
        uri = mesh.get('filename', '')
        sub(m, 'uri', uri)
        sc = num3(mesh.get('scale'))
        sub(m, 'scale', ' '.join('%.6g' % x for x in sc))
    elif box is not None:
        b = sub(g, 'box')
        sub(b, 'size', box.get('size', '1 1 1'))
    elif cyl is not None:
        c = sub(g, 'cylinder')
        sub(c, 'radius', cyl.get('radius', '0.5'))
        sub(c, 'length', cyl.get('length', '1.0'))
    elif sph is not None:
        s = sub(g, 'sphere')
        sub(s, 'radius', sph.get('radius', '0.5'))
    else:
        # unsupported / empty geometry -> tiny box so SDF stays valid
        b = sub(g, 'box')
        sub(b, 'size', '0.001 0.001 0.001')
    return g


def material_from_color(rgba):
    """rgba 'r g b a' -> sdf <material> diffuse/ambient"""
    parts = rgba.split()
    try:
        vals = [float(x) for x in parts[:4]]
    except Exception:
        return None
    if len(vals) < 4:
        vals = [1.0, 1.0, 1.0, 1.0]
    mt = []
    return vals


def add_material(parent, vals):
    m = sub(parent, 'material')
    txt = ' '.join('%.4f' % x for x in vals)
    sub(m, 'ambient', txt)
    sub(m, 'diffuse', txt)


# --------------------------------------------------------------------------- #
# URDF model structure
# --------------------------------------------------------------------------- #
class URDF:
    def __init__(self, path):
        self.root = ET.parse(path).getroot()
        self.name = self.root.get('name', 'go2')
        self.links = {}
        self.joints = {}
        for l in self.root.findall('link'):
            self.links[l.get('name')] = l
        for j in self.root.findall('joint'):
            self.joints[j.get('name')] = j
        # root link = link that is never a joint child
        children = {j.find('child').get('link') for j in self.joints.values()}
        roots = [n for n in self.links if n not in children]
        self.root_link = roots[0] if roots else 'base'

    def revolute_joints(self):
        return [j for j in self.joints.values() if j.get('type') in
                ('revolute', 'continuous')]


def resolve_mesh_uri(uri, share_dir):
    """package://go2_description/dae/x.dae -> file://<share>/dae/x.dae"""
    if uri.startswith('package://'):
        rest = uri[len('package://'):]            # go2_description/dae/x.dae
        parts = rest.split('/')
        fname = parts[-1]
        # find the /dae/ dir segment, fall back to anything after package name
        try:
            idx = parts.index('dae')
            rel = '/'.join(parts[idx:])
        except ValueError:
            rel = parts[0] + '/' + fname if len(parts) > 1 else fname
        return 'file://' + os.path.join(share_dir, rel)
    if uri.startswith('model://') or uri.startswith('file://'):
        return uri
    # plain relative filename -> assume under share dir
    return 'file://' + os.path.join(share_dir, os.path.basename(uri))


# --------------------------------------------------------------------------- #
# sensor specs (mirrors the <gazebo> blocks inside go2.urdf)
# --------------------------------------------------------------------------- #
def sensors_for(link_name, urdf):
    """Return list of (type, sensor dict) from the URDF <gazebo reference=...>"""
    out = []
    for gz in urdf.root.findall('gazebo'):
        if gz.get('reference') != link_name:
            continue
        for s in gz.findall('sensor'):
            st = s.get('type')
            name = s.get('name')
            if st == 'imu':
                out.append(('imu', dict(name=name)))
            elif st == 'gpu_lidar':
                out.append(('lidar', dict(name=name)))
            elif st == 'camera':
                out.append(('rgb', dict(name=name)))
            elif st == 'depth_camera':
                out.append(('depth', dict(name=name)))
    return out


def add_sensor(parent, kind, cfg, urdf, extra):
    """Build an SDF <sensor> for kind."""
    if kind == 'imu':
        s = sub(parent, 'sensor', name=cfg['name'], type='imu')
        sub(s, 'topic', 'imu')
        sub(s, 'always_on', '1')
        sub(s, 'update_rate', '100')
    elif kind == 'lidar':
        s = sub(parent, 'sensor', name=cfg['name'], type='gpu_lidar')
        # mount above the trunk shell so the robot does not see itself
        sub(s, 'pose', '0 0 %.4f 0 0 0' % extra['lidar_z'])
        sub(s, 'topic', 'scan')
        sub(s, 'always_on', '1')
        sub(s, 'update_rate', '10')
        sub(s, 'visualize', 'false')
        ray = sub(s, 'ray')
        sc = sub(ray, 'scan')
        h = sub(sc, 'horizontal')
        sub(h, 'samples', '360')
        sub(h, 'resolution', '1')
        sub(h, 'min_angle', '-3.14159265')
        sub(h, 'max_angle', '3.14159265')
        v = sub(sc, 'vertical')
        sub(v, 'samples', '1')
        sub(v, 'resolution', '1')
        sub(v, 'min_angle', '0')
        sub(v, 'max_angle', '0')
        rg = sub(ray, 'range')
        sub(rg, 'min', '0.10')
        sub(rg, 'max', '30.0')
        sub(rg, 'resolution', '0.01')
    elif kind in ('rgb', 'depth'):
        s = sub(parent, 'sensor', name=cfg['name'],
                type='camera' if kind == 'rgb' else 'depth_camera')
        sub(s, 'topic', 'rgb/image' if kind == 'rgb' else 'depth/image')
        sub(s, 'always_on', '1')
        sub(s, 'update_rate', '30' if kind == 'rgb' else '15')
        sub(s, 'visualize', 'false')
        cam = sub(s, 'camera')
        sub(cam, 'horizontal_fov', '1.089')
        img = sub(cam, 'image')
        sub(img, 'width', '640')
        sub(img, 'height', '480')
        sub(img, 'format', 'R8G8B8' if kind == 'rgb' else 'R_FLOAT32')
        clip = sub(cam, 'clip')
        sub(clip, 'near', '0.05')
        sub(clip, 'far', '30.0')


# --------------------------------------------------------------------------- #
# model generation
# --------------------------------------------------------------------------- #
def build_model(urdf, share_dir, extra):
    model = ET.Element('model', name=urdf.name)
    # spawn pose (in world)
    sub(model, 'pose',
        '0 0 %.4f 0 0 %.4f' % (extra['spawn_z'], extra['yaw']))
    sub(model, 'static', 'false')
    sub(model, 'self_collide', 'false')

    # URDF -> SDF frame mapping: the URDF joint <origin> is the pose of the
    # child frame in the parent. We reproduce it exactly by putting that pose on
    # the SDF joint (relative to the parent link) and making the child link's
    # frame coincide with its parent joint's frame at q=0.
    child2joint = {j.find('child').get('link'): jn
                   for jn, j in urdf.joints.items()}

    # ---- links ----
    for name, link in urdf.links.items():
        l = sub(model, 'link', name=name)
        if name == urdf.root_link:
            sub(l, 'pose', '0 0 0 0 0 0')          # root at model origin
        else:
            parent_joint = child2joint[name]
            sub(l, 'pose', '0 0 0 0 0 0',
                relative_to=parent_joint)

        # visual(s): only the real URDF <visual> meshes. Links that only carry
        # <collision> proxies (head collar, foot spikes, rotor masses) have no
        # visual geometry of their own -- base.dae already includes the head +
        # trunk shell, and calf/foot meshes cover the lower legs.
        visuals = link.findall('visual')
        for i, vis in enumerate(visuals):
            xyz, rpy = parse_pose(vis.find('origin'))
            v = sub(l, 'visual', name=('visual' if i == 0 else 'visual_%d' % i))
            sub(v, 'pose', pos3(xyz, rpy))
            geom = vis.find('geometry')
            # only apply a colour tint to primitive shapes; CAD meshes keep
            # the materials embedded in the .dae file
            mat = None
            if geom is not None and geom.find('mesh') is None:
                mc = vis.find('material')
                if mc is not None and mc.find('color') is not None:
                    mat = material_from_color(mc.find('color').get('rgba'))
            add_geometry(v, geom, mat)
            if mat:
                add_material(v, mat)

        # collision(s): intentionally omitted (kinematic patrol)
        # ---- inertial ----
        inertial = link.find('inertial')
        mass_val = 0.0
        ok = False
        if inertial is not None:
            m = inertial.find('mass')
            mass_val = float(m.get('value')) if m is not None else 0.0
            ok = inertial.find('inertia') is not None and mass_val > 0.0
        if ok:
            xyz, rpy = parse_pose(inertial.find('origin'))
            it = sub(l, 'inertial')
            sub(it, 'pose', pos3(xyz, rpy))
            mval = inertial.find('mass').get('value')
            sub(it, 'mass', mval)
            inertia = inertial.find('inertia')
            ii = sub(it, 'inertia')
            for tag in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'):
                el = inertia.get(tag)
                sub(ii, tag, str(float(el)) if el else '0')
        else:
            # massless/decorative link (imu, camera frames, ...) -> tiny mass
            it = sub(l, 'inertial')
            sub(it, 'pose', '0 0 0 0 0 0')
            sub(it, 'mass', '0.001')
            ii = sub(it, 'inertia')
            for tag in ('ixx', 'iyy', 'izz'):
                sub(ii, tag, '1e-6')
            for tag in ('ixy', 'ixz', 'iyz'):
                sub(ii, tag, '0')

        # ---- sensors (from <gazebo reference=link> blocks) ----
        for kind, cfg in sensors_for(name, urdf):
            add_sensor(l, kind, cfg, urdf, extra)

    # ---- joints ----
    for jname, joint in urdf.joints.items():
        jtype = joint.get('type')
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        xyz, rpy = parse_pose(joint.find('origin'))
        j = sub(model, 'joint', name=jname, type=jtype)
        sub(j, 'pose', pos3(xyz, rpy), relative_to=parent)
        sub(j, 'parent', parent)
        sub(j, 'child', child)
        if jtype in ('revolute', 'continuous', 'prismatic'):
            axis = sub(j, 'axis')
            ax = joint.find('axis')
            axv = [1.0, 0.0, 0.0]
            if ax is not None and ax.get('xyz'):
                axv = [float(x) for x in ax.get('xyz').split()]
            # URDF axis is in the joint (child) frame at q=0; our SDF joint
            # pose carries the same orientation, so numeric axes carry over.
            sub(axis, 'xyz', ' '.join('%.6g' % x for x in axv))
            limit = joint.find('limit')
            if limit is not None:
                lim = sub(axis, 'limit')
                low = limit.get('lower', '-1e16')
                up = limit.get('upper', '1e16')
                eff = limit.get('effort', '1e6')
                vel = limit.get('velocity', '1e6')
                sub(lim, 'lower', low)
                sub(lim, 'upper', up)
                sub(lim, 'effort', eff)
                sub(lim, 'velocity', vel)

    # ---- model-level systems ----
    vc = sub(model, 'plugin',
             filename='gz-sim-velocity-control-system',
             name='gz::sim::systems::VelocityControl')
    sub(vc, 'link_name', urdf.root_link)

    # standing pose per joint (hip 0, thigh +0.9, calf -1.8)
    def standing_angle(jname):
        if '_hip_joint' in jname:
            return 0.0
        if '_thigh_joint' in jname:
            return 0.9
        if '_calf_joint' in jname:
            return -1.8
        return 0.0

    jtc = sub(model, 'plugin',
              filename='gz-sim-joint-trajectory-controller-system',
              name='gz::sim::systems::JointTrajectoryController')
    sub(jtc, 'topic', '/model/%s/joint_trajectory' % urdf.name)
    for jt in urdf.revolute_joints():
        jname = jt.get('name')
        sub(jtc, 'joint_name', jname)
        sub(jtc, 'initial_position', '%.4f' % standing_angle(jname))
        sub(jtc, 'position_p_gain', '5.0')
        sub(jtc, 'position_i_gain', '0.05')
        sub(jtc, 'position_d_gain', '0.3')
        sub(jtc, 'position_i_min', '-1')
        sub(jtc, 'position_i_max', '1')
        sub(jtc, 'position_cmd_min', '-30')
        sub(jtc, 'position_cmd_max', '30')
    return model


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--share', required=True,
                    help='share dir of go2_description (dae/ lives here)')
    ap.add_argument('--scenery', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--spawn-z', type=float, default=0.32)
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--lidar-z', type=float, default=0.10)
    args = ap.parse_args()

    if not os.path.isdir(os.path.join(args.share, 'dae')):
        sys.exit('error: share dir %s has no dae/ (install go2_description?)'
                 % args.share)

    urdf = URDF(args.urdf)
    extra = dict(spawn_z=args.spawn_z, yaw=args.yaw, lidar_z=args.lidar_z)

    # rewrite mesh URIs package://go2_description/... -> file://<share>/...
    for link in urdf.root.iter('mesh'):
        f = link.get('filename')
        if f:
            link.set('filename', resolve_mesh_uri(f, args.share))

    model = build_model(urdf, args.share, extra)

    # inject model into scenery world at the marker
    with open(args.scenery, 'r', encoding='utf-8') as fh:
        world = fh.read()
    marker = '<!--GO2_MODEL_INSERT_HERE-->'
    if marker not in world:
        sys.exit('error: scenery %s missing marker %s' % (args.scenery, marker))
    xml_str = ET.tostring(model, encoding='unicode')
    world = world.replace(marker, xml_str, 1)

    with open(args.out, 'w', encoding='utf-8') as fh:
        fh.write(world)
    print('wrote %s (%d bytes), robot=%s links=%d joints=%d' %
          (args.out, len(world), urdf.name, len(urdf.links), len(urdf.joints)))


if __name__ == '__main__':
    main()
