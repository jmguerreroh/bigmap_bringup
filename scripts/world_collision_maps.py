#!/usr/bin/env python3
# Copyright 2026 José Miguel Guerrero
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The maps of a Gazebo world made from its collision geometry (what the robot can touch).

    world_collision_maps.py --world large_warehouse_world.sdf --models <dir> --out <prefix>
        [--res 0.5] [--bounds x0,y0,x1,y1] [--robot-height 1.2] [--cloud-step 0.2]
        [--cloud-visuals model[,model...]]

Writes <prefix>.navmap and <prefix>.pcd:

  * <prefix>.navmap: the ground as a grid of `res` m with no NavCel where a collision occupies any
    height between 0.05 m and `robot_height` above the ground (a cell is blocked if any part of an
    obstacle is in it: the conservative rule). Ground height = the top of the highest thin slab
    (floor, yard, grass) under the cell.
  * <prefix>.pcd: points on every face of every collision (every `cloud_step` m), what a lidar can
    see, for the localization map (Bonxai / AMCL); the slabs (ground) every 1 m. With
    --cloud-visuals, also the visuals of those included models (by include name): things a lidar
    sees but the robot cannot touch (e.g. large_warehouse_world's warehouse_a_floor_stock, packages
    with no collision). They go to the cloud only, never to the NavMap.

Why: the clouds that come with large_warehouse_world are sampled from above (floor and rack tops):
neither the planner's map nor the localizer's had what the robot meets at its height.

Reads <world> includes (model://name in --models, Fuel models from the Gazebo cache
~/.gz/fuel/<server>/<owner>/models/<name>/<version>/) recursively, with their poses
(SDF: x y z roll pitch yaw, rotation Rz(yaw) Ry(pitch) Rx(roll)); geometry box, cylinder and mesh
(OBJ/STL/GLB through trimesh, with <scale>). `relative_to` frames are not supported (an error).
"""

import argparse
import glob
import math
import os
import struct
import sys
import urllib.parse
import xml.etree.ElementTree as ET

import numpy as np


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def pose_of(elem):
    """4x4 transform of the <pose> child of elem (identity without one)."""
    T = np.eye(4)
    p = elem.find('pose') if elem is not None else None
    if p is not None and p.text and p.text.strip():
        if p.get('relative_to'):
            raise SystemExit('relative_to poses are not supported (%s)' % p.get('relative_to'))
        v = [float(t) for t in p.text.split()]
        v += [0.0] * (6 - len(v))
        T[:3, :3] = rpy_matrix(v[3], v[4], v[5])
        T[:3, 3] = v[:3]
    return T


class Resolver:
    def __init__(self, models_dir):
        self.models_dir = models_dir
        self.fuel = os.path.expanduser('~/.gz/fuel')

    def model_sdf(self, uri):
        if uri.startswith('model://'):
            d = os.path.join(self.models_dir, uri[len('model://'):].split('/')[0])
        elif uri.startswith('http'):
            u = urllib.parse.urlparse(uri)
            parts = [p for p in u.path.split('/') if p]
            # /1.0/<owner>/models/<name>[/<version>]
            owner, name = parts[1].lower(), urllib.parse.unquote(parts[3]).lower()
            cands = sorted(glob.glob(os.path.join(self.fuel, u.netloc, owner, 'models',
                                                  urllib.parse.quote(name), '*')) +
                           glob.glob(os.path.join(self.fuel, u.netloc, owner, 'models', name, '*')))
            if not cands:
                raise SystemExit('Fuel model not in the Gazebo cache: %s' % uri)
            d = cands[-1]
        else:
            raise SystemExit('unsupported uri %s' % uri)
        for f in ('model.sdf', 'model.urdf'):
            if os.path.isfile(os.path.join(d, f)):
                return os.path.join(d, f), d
        raise SystemExit('no model.sdf in %s' % d)

    def mesh_path(self, uri, model_dir):
        if uri.startswith('model://'):
            rest = uri[len('model://'):]
            return os.path.join(self.models_dir, rest)
        if uri.startswith('http'):
            # https://fuel.../models/<name>/<version>/files/meshes/x.obj -> relative to the model
            i = uri.find('/files/')
            return os.path.join(model_dir, urllib.parse.unquote(uri[i + len('/files/'):]))
        return os.path.join(model_dir, uri)


def collisions_of_model(model, T, model_dir, res, out, depth=0):
    """Appends (world transform, geometry element, model dir) of every collision of an SDF model."""
    Tm = T @ pose_of(model)
    for link in model.findall('link'):
        Tl = Tm @ pose_of(link)
        for col in link.findall('collision'):
            out.append((Tl @ pose_of(col), col.find('geometry'), model_dir))
    for inc in model.findall('include'):
        include(inc, Tm, res, out, depth + 1)
    for sub in model.findall('model'):
        collisions_of_model(sub, Tm, model_dir, res, out, depth + 1)


# include names whose visuals go to the cloud (--cloud-visuals), and those visuals: (world transform,
# geometry element, model dir)
CLOUD_VISUAL_MODELS = set()
CLOUD_VISUALS = []


def include(inc, T, res, out, depth=0):
    if depth > 10:
        raise SystemExit('includes nested too deep')
    uri = inc.find('uri').text.strip()
    sdf, d = res.model_sdf(uri)
    root = ET.parse(sdf).getroot()
    model = root.find('model')
    Ti = T @ pose_of(inc)
    name = inc.find('name').text.strip() if inc.find('name') is not None else model.get('name')
    if name in CLOUD_VISUAL_MODELS:
        for link in model.findall('link'):
            Tl = Ti @ pose_of(link)
            for vis in link.findall('visual'):
                CLOUD_VISUALS.append((Tl @ pose_of(vis), vis.find('geometry'), d))
    # the include's pose replaces the model's own top-level pose
    m = ET.Element('model')
    for child in model:
        if child.tag != 'pose':
            m.append(child)
    collisions_of_model(m, Ti, d, res, out, depth)


def box_points(size, step):
    """Points on the 6 faces of a box centred on the origin, every `step` m (at least 2 per side)."""
    sx, sy, sz = size
    def lin(a):
        return np.linspace(-a / 2, a / 2, max(2, int(math.ceil(a / step)) + 1))
    xs, ys, zs = lin(sx), lin(sy), lin(sz)
    pts = []
    for z in (-sz / 2, sz / 2):
        X, Y = np.meshgrid(xs, ys)
        pts.append(np.stack([X.ravel(), Y.ravel(), np.full(X.size, z)], 1))
    for y in (-sy / 2, sy / 2):
        X, Z = np.meshgrid(xs, zs)
        pts.append(np.stack([X.ravel(), np.full(X.size, y), Z.ravel()], 1))
    for x in (-sx / 2, sx / 2):
        Y, Z = np.meshgrid(ys, zs)
        pts.append(np.stack([np.full(Y.size, x), Y.ravel(), Z.ravel()], 1))
    return np.concatenate(pts)


def box_volume_xy(size, step):
    """A grid of points filling the box's xy section (to rasterize its footprint)."""
    sx, sy, _ = size
    xs = np.linspace(-sx / 2, sx / 2, max(2, int(math.ceil(sx / step)) + 1))
    ys = np.linspace(-sy / 2, sy / 2, max(2, int(math.ceil(sy / step)) + 1))
    X, Y = np.meshgrid(xs, ys)
    return np.stack([X.ravel(), Y.ravel(), np.zeros(X.size)], 1)


def dae_bbox(path):
    """(min, max) corners, metres, of the vertex positions of a COLLADA file (its <unit>).
    Without pycollada the mesh is taken as its bounding box: conservative for the planner, enough
    for a lidar. A single node <matrix> (one geometry instance) is applied; other node transforms
    are not supported (an error)."""
    import re
    s = open(path, errors='replace').read()
    M = None
    if re.search(r'<rotate[^>]*>', s) or len(re.findall(r'<matrix[^>]*>', s)) > 1:
        raise SystemExit('%s: node transforms are not supported without pycollada' % path)
    m = re.search(r'<matrix[^>]*>([^<]*)</matrix>', s)
    if m:
        if len(re.findall(r'<instance_geometry', s)) != 1:
            raise SystemExit('%s: a node matrix with several geometries is not supported' % path)
        M = np.array(m.group(1).split(), float).reshape(4, 4)   # COLLADA: row-major
    unit = re.search(r'<unit[^>]*meter="([0-9.eE+-]+)"', s)
    k = float(unit.group(1)) if unit else 1.0
    P = []
    for sid, arr in re.findall(r'<source id="([^"]*)">\s*<float_array[^>]*>([^<]*)</float_array>', s):
        if 'position' in sid.lower():
            P.append(np.array(arr.split(), float).reshape(-1, 3))
    if not P:
        raise SystemExit('%s: no vertex positions' % path)
    P = np.concatenate(P)
    if M is not None:
        P = P @ M[:3, :3].T + M[:3, 3]
    P = P * k
    return P.min(0), P.max(0)


def mesh_points(geom, res, model_dir, step):
    import trimesh
    m = geom.find('mesh')
    path = res.mesh_path(m.find('uri').text.strip(), model_dir)
    if path.lower().endswith('.dae'):
        lo, hi = dae_bbox(path)
        sc = m.find('scale')
        k = np.array([float(v) for v in sc.text.split()]) if sc is not None else np.ones(3)
        lo, hi = lo * k, hi * k
        print('%s: COLLADA taken as its bounding box %s .. %s m' % (os.path.basename(path),
              np.round(lo, 2), np.round(hi, 2)), file=sys.stderr)
        return box_points(hi - lo, step) + (hi + lo) / 2
    mesh = trimesh.load(path, force='mesh')
    sc = m.find('scale')
    if sc is not None:
        mesh.apply_scale([float(v) for v in sc.text.split()])
    n = max(100, int(mesh.area / (step * step)))
    # a fixed seed: the same world gives the same maps
    pts, _ = trimesh.sample.sample_surface_even(mesh, n, seed=0)
    return np.asarray(pts)


def transform(T, P):
    return P @ T[:3, :3].T + T[:3, 3]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', required=True)
    ap.add_argument('--models', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--res', type=float, default=0.5)
    ap.add_argument('--bounds', default='-10,-10,710,510')
    ap.add_argument('--robot-height', type=float, default=1.2)
    ap.add_argument('--cloud-step', type=float, default=0.2)
    ap.add_argument('--slab-thickness', type=float, default=0.3,
                    help='a box at most this thick whose top is within 0.1 m of z = 0 is ground')
    ap.add_argument('--cloud-visuals', default='',
                    help='included models (by name) whose visuals also go to the cloud, not the NavMap')
    a = ap.parse_args()
    CLOUD_VISUAL_MODELS.update(n for n in a.cloud_visuals.split(',') if n)
    x0, y0, x1, y1 = [float(v) for v in a.bounds.split(',')]
    res = Resolver(a.models)
    world = ET.parse(a.world).getroot().find('world')
    cols = []
    for inc in world.findall('include'):
        include(inc, np.eye(4), res, cols)
    for m in world.findall('model'):
        collisions_of_model(m, np.eye(4), os.path.dirname(a.world), res, cols)
    print('collisions: %d' % len(cols), file=sys.stderr)

    nx, ny = int(round((x1 - x0) / a.res)), int(round((y1 - y0) / a.res))
    ground = np.full((ny, nx), np.nan)
    blocked = np.zeros((ny, nx), bool)
    cloud = []
    kinds = {}

    def cells(P):
        i = np.floor((P[:, 0] - x0) / a.res).astype(int)
        j = np.floor((P[:, 1] - y0) / a.res).astype(int)
        ok = (i >= 0) & (j >= 0) & (i < nx) & (j < ny)
        return i[ok], j[ok], ok

    slabs = []
    for T, g, d in cols:
        kind = g[0].tag
        kinds[kind] = kinds.get(kind, 0) + 1
        if kind == 'box':
            size = [float(v) for v in g.find('box/size').text.split()]
            corners = transform(T, np.array([[sx * size[0] / 2, sy * size[1] / 2, sz * size[2] / 2]
                                             for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]))
            zlo, zhi = corners[:, 2].min(), corners[:, 2].max()
            if size[2] <= a.slab_thickness and abs(zhi) <= 0.1:
                slabs.append((T, size, zhi))
                continue
            surf = transform(T, box_points(size, a.cloud_step))
            step = min(a.res / 4, max(0.02, min(size[0], size[1]) / 2))
            foot = transform(T, box_volume_xy(size, step))
        elif kind == 'plane':
            # a horizontal ground plane (normal 0 0 1): a slab of no thickness, its top at its z
            sx, sy = [float(v) for v in g.find('plane/size').text.split()]
            top = transform(T, np.zeros((1, 3)))[0, 2]
            slabs.append((T, [sx, sy, 0.0], top))
            continue
        elif kind == 'cylinder':
            r = float(g.find('cylinder/radius').text); L = float(g.find('cylinder/length').text)
            th = np.linspace(0, 2 * math.pi, max(8, int(2 * math.pi * r / a.cloud_step)), endpoint=False)
            zz = np.linspace(-L / 2, L / 2, max(2, int(L / a.cloud_step) + 1))
            TH, ZZ = np.meshgrid(th, zz)
            surf = transform(T, np.stack([r * np.cos(TH).ravel(), r * np.sin(TH).ravel(), ZZ.ravel()], 1))
            rr = np.linspace(0, r, max(2, int(r / (a.res / 4)) + 1))
            R2, T2 = np.meshgrid(rr, th)
            foot = transform(T, np.stack([(R2 * np.cos(T2)).ravel(), (R2 * np.sin(T2)).ravel(), np.zeros(R2.size)], 1))
            zlo, zhi = surf[:, 2].min(), surf[:, 2].max()
        elif kind == 'mesh':
            surf = transform(T, mesh_points(g, res, d, a.cloud_step))
            foot = surf
            zlo, zhi = surf[:, 2].min(), surf[:, 2].max()
        else:
            print('ignored geometry %s' % kind, file=sys.stderr)
            continue
        cloud.append(surf)
        # in the robot's height band: its footprint blocks the cells (the part of it in the band)
        if zhi >= 0.05 and zlo <= a.robot_height:
            if kind == 'mesh':
                band = foot[(foot[:, 2] >= 0.05) & (foot[:, 2] <= a.robot_height)]
            else:
                band = foot
            i, j, _ = cells(band)
            blocked[j, i] = True
    # the ground: the top of the highest slab under each cell centre
    cx = x0 + (np.arange(nx) + 0.5) * a.res
    cy = y0 + (np.arange(ny) + 0.5) * a.res
    CX, CY = np.meshgrid(cx, cy)
    for T, size, top in slabs:
        Ti = np.linalg.inv(T)
        L = np.stack([CX.ravel(), CY.ravel(), np.zeros(CX.size)], 1) @ Ti[:3, :3].T + Ti[:3, 3]
        inside = (np.abs(L[:, 0]) <= size[0] / 2) & (np.abs(L[:, 1]) <= size[1] / 2)
        g = ground.ravel()
        better = inside & (np.isnan(g) | (top > g))
        g[better] = top
        ground = g.reshape(ny, nx)
        # the ground in the cloud, every 1 m, inside the bounds
        gx = np.arange(-size[0] / 2, size[0] / 2 + 1e-9, 1.0)
        gy = np.arange(-size[1] / 2, size[1] / 2 + 1e-9, 1.0)
        GX, GY = np.meshgrid(gx, gy)
        P = transform(T, np.stack([GX.ravel(), GY.ravel(), np.full(GX.size, size[2] / 2)], 1))
        P = P[(P[:, 0] >= x0) & (P[:, 0] <= x1) & (P[:, 1] >= y0) & (P[:, 1] <= y1)]
        cloud.append(P)
    # what a lidar sees and the robot cannot touch: in the cloud only
    nvis = 0
    for T, g, d in CLOUD_VISUALS:
        kind = g[0].tag
        if kind == 'mesh':
            cloud.append(transform(T, mesh_points(g, res, d, a.cloud_step)))
        elif kind == 'box':
            cloud.append(transform(T, box_points([float(v) for v in g.find('box/size').text.split()],
                                                 a.cloud_step)))
        else:
            print('ignored visual geometry %s' % kind, file=sys.stderr)
            continue
        nvis += 1
    if CLOUD_VISUAL_MODELS:
        print('visuals in the cloud only (%s): %d' % (','.join(sorted(CLOUD_VISUAL_MODELS)), nvis),
              file=sys.stderr)
    free = ~blocked & ~np.isnan(ground)
    print('geometry: %s; slabs %d; grid %d x %d at %.2f m: %.1f%% free, %.1f%% blocked, %.1f%% without ground'
          % (kinds, len(slabs), nx, ny, a.res, 100 * free.mean(), 100 * blocked.mean(),
             100 * np.isnan(ground).mean()), file=sys.stderr)

    # --- NavMap: vertices on the grid nodes, two NavCels per free cell
    from navmap_ros_interfaces.msg import NavMap, NavMapSurface
    from navmap_tools.resample import write_navmap
    gz = np.where(np.isnan(ground), 0.0, ground)
    node_z = np.zeros((ny + 1, nx + 1))
    cnt = np.zeros((ny + 1, nx + 1))
    for dj in (0, 1):
        for di in (0, 1):
            node_z[dj:dj + ny, di:di + nx] += np.where(free, gz, 0.0)
            cnt[dj:dj + ny, di:di + nx] += free
    node_z = np.where(cnt > 0, node_z / np.maximum(cnt, 1), 0.0)
    jj, ii = np.meshgrid(np.arange(ny + 1), np.arange(nx + 1), indexing='ij')
    msg = NavMap()
    msg.header.frame_id = 'map'
    msg.positions_x = (x0 + ii * a.res).astype(np.float32).ravel().tolist()
    msg.positions_y = (y0 + jj * a.res).astype(np.float32).ravel().tolist()
    msg.positions_z = node_z.astype(np.float32).ravel().tolist()
    cj, ci = np.nonzero(free)
    vid = lambda i, j: j * (nx + 1) + i  # noqa: E731
    A, B, C, D = vid(ci, cj), vid(ci + 1, cj), vid(ci + 1, cj + 1), vid(ci, cj + 1)
    msg.navcels_v0 = np.stack([A, A], 1).ravel().astype(np.uint32).tolist()
    msg.navcels_v1 = np.stack([B, C], 1).ravel().astype(np.uint32).tolist()
    msg.navcels_v2 = np.stack([C, D], 1).ravel().astype(np.uint32).tolist()
    s = NavMapSurface()
    s.frame_id = 'map'
    s.navcels = list(range(2 * len(ci)))
    msg.surfaces = [s]
    write_navmap(msg, a.out + '.navmap')

    # --- cloud
    P = np.concatenate(cloud)
    P = P[(P[:, 0] >= x0 - 1) & (P[:, 0] <= x1 + 1) & (P[:, 1] >= y0 - 1) & (P[:, 1] <= y1 + 1)].astype(np.float32)
    hdr = ('# .PCD v0.7 - Point Cloud Data file format\n'
           '# world_collision_maps.py: every face of every collision of %s every %.2f m, ground every 1 m\n'
           'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\n'
           'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n') % (os.path.basename(a.world), a.cloud_step, len(P), len(P))
    with open(a.out + '.pcd', 'wb') as f:
        f.write(hdr.encode())
        f.write(P.tobytes())
    print('wrote %s.navmap (%d NavCels) and %s.pcd (%d points)' % (a.out, 2 * len(ci), a.out, len(P)))


if __name__ == '__main__':
    main()
