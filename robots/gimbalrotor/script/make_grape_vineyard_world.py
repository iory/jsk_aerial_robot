#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate the gazebo models and the world of the grape harvest demo.

The glTF assets (Y-up) of a vine row and grape bunches are converted into gazebo
classic models with OBJ meshes (gazebo classic does not load glTF), and
``worlds/grape_vineyard.world`` is written from the layout in
``config/grape_with_arm/GrapeHarvestDemo.yaml``.

Usage
-----
rosrun gimbalrotor make_grape_vineyard_world.py --asset-dir <dir>
rosrun gimbalrotor make_grape_vineyard_world.py   # world only, keep the models

``<dir>`` contains ``objects/grape_vine_t4/grape_vine_t4.glb`` and
``objects/bunch_library/bunch_<nn>_<variety>.glb``.
"""

import argparse
import json
import math
import os
import shutil
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
import rospkg
import trimesh
import yaml

VINE_ROW_MODEL = 'grape_vine_row'
CRATE_MODEL = 'grape_harvest_crate'
BUNCH_MODEL_PREFIX = 'grape_bunch_'
# models of the bunches to harvest are named with this prefix in the world;
# script/grape_harvest_sim_grasp.py looks for it
HARVEST_BUNCH_PREFIX = 'harvest_bunch_'

# glTF is Y-up, gazebo is Z-up
YUP_TO_ZUP = trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0])

# thin leaves and branches are single sheets in the asset
DOUBLE_SIDED_PARTS = ('leaves', 'branches')

BUNCH_MASS = 0.15  # [kg]
# single bunches of the library are about 0.1 m wide, double ones 0.16 m
DOUBLE_BUNCH_WIDTH = 0.13  # [m]
GLTF_COMPONENT_DTYPES = {5126: np.float32, 5121: np.uint8, 5123: np.uint16}


def linear_to_srgb(color):
    """Convert linear RGB in [0, 1] (glTF colors) to sRGB in [0, 1].

    Parameters
    ----------
    color : array_like
        Linear RGB.

    Returns
    -------
    numpy.ndarray
        sRGB.
    """
    color = np.clip(np.asarray(color, dtype=np.float64), 0.0, 1.0)
    return np.where(color <= 0.0031308, 12.92 * color,
                    1.055 * np.power(color, 1.0 / 2.4) - 0.055)


def glb_vertex_colors(path):
    """Read the ``COLOR_0`` vertex colors of every mesh in a GLB file.

    trimesh drops ``COLOR_0`` of a primitive that also has a material.

    Parameters
    ----------
    path : str
        GLB file.

    Returns
    -------
    dict
        ``{mesh name: (N, 4) linear RGBA in [0, 1]}``.
    """
    with open(path, 'rb') as f:
        data = f.read()
    json_length = struct.unpack('<I', data[12:16])[0]
    gltf = json.loads(data[20:20 + json_length])
    bin_start = 20 + json_length + 8
    colors = {}
    for mesh in gltf.get('meshes', []):
        for primitive in mesh.get('primitives', []):
            index = primitive.get('attributes', {}).get('COLOR_0')
            if index is None:
                continue
            accessor = gltf['accessors'][index]
            view = gltf['bufferViews'][accessor['bufferView']]
            n_components = {'VEC3': 3, 'VEC4': 4}[accessor['type']]
            component_type = accessor['componentType']
            raw = np.frombuffer(
                data, dtype=GLTF_COMPONENT_DTYPES[component_type],
                count=accessor['count'] * n_components,
                offset=bin_start + view.get('byteOffset', 0)
                + accessor.get('byteOffset', 0))
            raw = raw.reshape(-1, n_components).astype(np.float64)
            if component_type != 5126:
                raw /= np.iinfo(GLTF_COMPONENT_DTYPES[component_type]).max
            rgba = np.ones((len(raw), 4))
            rgba[:, :n_components] = raw
            colors[mesh['name']] = rgba
            break
    return colors


def load_glb_parts(path):
    """Load the meshes of a GLB file in Z-up coordinates.

    Parameters
    ----------
    path : str
        GLB file.

    Returns
    -------
    list of tuple
        ``(material name, trimesh.Trimesh, vertex colors or None)`` of each
        mesh, with the scene graph transform applied.
    """
    scene = trimesh.load(path, force='scene')
    vertex_colors = glb_vertex_colors(path)
    parts = []
    for node in scene.graph.nodes_geometry:
        transform, geometry_name = scene.graph[node]
        mesh = scene.geometry[geometry_name].copy()
        mesh.apply_transform(YUP_TO_ZUP.dot(transform))
        colors = vertex_colors.get(geometry_name)
        if colors is not None and len(colors) != len(mesh.vertices):
            colors = None
        parts.append((mesh.visual.material.name, mesh, colors))
    return parts


def part_name(material_name):
    """Return a file name for a material, e.g. ``Leaves.006`` -> ``leaves``."""
    return material_name.split('.')[0].lower()


def write_obj(directory, name, mesh, color=None, double_sided=False):
    """Write a mesh as ``<directory>/<name>.obj`` with its material.

    Parameters
    ----------
    directory : str
        Output directory.
    name : str
        Base name of the OBJ, MTL and texture files.
    mesh : trimesh.Trimesh
        Mesh with ``TextureVisuals``.
    color : array_like or None
        sRGB diffuse color in [0, 1]. If None, the base color texture of the
        material is used, or its base color factor without a texture.
    double_sided : bool
        If True, the faces are duplicated with the opposite winding.
    """
    os.makedirs(directory, exist_ok=True)
    vertices = np.asarray(mesh.vertices)
    normals = np.asarray(mesh.vertex_normals)
    faces = np.asarray(mesh.faces)
    material = mesh.visual.material
    uv = getattr(mesh.visual, 'uv', None)
    texture = getattr(material, 'baseColorTexture', None)
    if color is not None or texture is None or uv is None:
        texture = None
        uv = None
    if color is None and texture is None:
        factor = getattr(material, 'baseColorFactor', None)
        if factor is None:
            raise RuntimeError(
                'material {} has neither a texture nor a color'.format(
                    material.name))
        # used as sRGB as it is: converted from linear, the grapes of the
        # vine row turn pale blue and its tendrils neon green in gazebo
        color = np.asarray(factor[:3]) / 255.0
    if double_sided:
        n = len(vertices)
        vertices = np.vstack([vertices, vertices])
        normals = np.vstack([normals, -normals])
        faces = np.vstack([faces, faces[:, ::-1] + n])
        if uv is not None:
            uv = np.vstack([uv, uv])

    lines = ['mtllib {}.mtl'.format(name), 'o {}'.format(name)]
    lines.extend('v {:.5f} {:.5f} {:.5f}'.format(*v) for v in vertices)
    lines.extend('vn {:.4f} {:.4f} {:.4f}'.format(*v) for v in normals)
    if uv is not None:
        lines.extend('vt {:.5f} {:.5f}'.format(*v) for v in uv)
    lines.append('usemtl {}'.format(name))
    one = faces + 1
    if uv is not None:
        lines.extend('f {0}/{0}/{0} {1}/{1}/{1} {2}/{2}/{2}'.format(*f)
                     for f in one)
    else:
        lines.extend('f {0}//{0} {1}//{1} {2}//{2}'.format(*f) for f in one)
    with open(os.path.join(directory, name + '.obj'), 'w') as f:
        f.write('\n'.join(lines) + '\n')

    mtl = ['newmtl {}'.format(name), 'Ks 0.05 0.05 0.05', 'Ns 10', 'd 1']
    if texture is not None:
        texture_file = name + '.png'
        texture.convert('RGB').save(os.path.join(directory, texture_file))
        mtl += ['Ka 1 1 1', 'Kd 1 1 1', 'map_Kd {}'.format(texture_file)]
    else:
        rgb = ' '.join('{:.4f}'.format(c) for c in color)
        mtl += ['Ka ' + rgb, 'Kd ' + rgb]
    with open(os.path.join(directory, name + '.mtl'), 'w') as f:
        f.write('\n'.join(mtl) + '\n')


def indent(element):
    ET.indent(element, space='  ')
    return element


def sub(parent, tag, text=None, **attrib):
    element = ET.SubElement(parent, tag, attrib)
    if text is not None:
        element.text = text
    return element


def fmt(values):
    return ' '.join('{:.6g}'.format(v) for v in values)


def write_model(model_dir, name, description, sdf_model):
    """Write ``model.config`` and ``model.sdf`` of a gazebo model."""
    os.makedirs(model_dir, exist_ok=True)
    config = ET.Element('model')
    sub(config, 'name', name)
    sub(config, 'version', '1.0')
    sub(config, 'sdf', 'model.sdf', version='1.6')
    author = sub(config, 'author')
    sub(author, 'name', 'gimbalrotor')
    sub(config, 'description', description)
    ET.ElementTree(indent(config)).write(
        os.path.join(model_dir, 'model.config'), encoding='unicode',
        xml_declaration=True)
    sdf = ET.Element('sdf', version='1.6')
    sdf.append(sdf_model)
    ET.ElementTree(indent(sdf)).write(
        os.path.join(model_dir, 'model.sdf'), encoding='unicode',
        xml_declaration=True)


def mesh_visual(link, name, uri):
    visual = sub(link, 'visual', name=name)
    sub(sub(sub(visual, 'geometry'), 'mesh'), 'uri', uri)
    return visual


def box_geometry(parent, size):
    sub(sub(sub(parent, 'geometry'), 'box'), 'size', fmt(size))


def make_vine_row_model(asset_dir, models_dir):
    """Convert the vine row asset into a static visual-only model.

    The canopy has no collision: leaves do not stop a real drone either, and
    the demo keeps the rotors out of it.
    """
    path = os.path.join(asset_dir, 'objects', 'grape_vine_t4',
                        'grape_vine_t4.glb')
    model_dir = os.path.join(models_dir, VINE_ROW_MODEL)
    meshes_dir = os.path.join(model_dir, 'meshes')
    shutil.rmtree(meshes_dir, ignore_errors=True)
    model = ET.Element('model', name=VINE_ROW_MODEL)
    sub(model, 'static', 'true')
    link = sub(model, 'link', name='vine')
    for material_name, mesh, _ in load_glb_parts(path):
        name = part_name(material_name)
        write_obj(os.path.join(meshes_dir, name), name, mesh,
                  double_sided=name in DOUBLE_SIDED_PARTS)
        visual = mesh_visual(link, name, 'model://{}/meshes/{}/{}.obj'.format(
            VINE_ROW_MODEL, name, name))
        sub(visual, 'cast_shadows', 'false')
        print('  {}: {} vertices'.format(name, len(mesh.vertices)))
    write_model(model_dir, VINE_ROW_MODEL,
                'A 6.5 m row of trellised grape vines (visual only).', model)


def bunch_variety(file_name):
    """``bunch_06_kyoho_black.glb`` -> ``kyoho_black``."""
    return os.path.splitext(file_name)[0].split('_', 2)[2]


def make_bunch_models(asset_dir, models_dir):
    """Convert the single bunches of the bunch library into models.

    The origin of a model is the middle of the stem, where the gripper
    pinches it. A bunch is a light rigid body without gravity, so that it
    hangs where it is placed until it is released.

    Returns
    -------
    list of str
        Varieties of the generated models.
    """
    library = os.path.join(asset_dir, 'objects', 'bunch_library')
    varieties = []
    for file_name in sorted(os.listdir(library)):
        if not file_name.endswith('.glb'):
            continue
        parts = load_glb_parts(os.path.join(library, file_name))
        berries = [p for p in parts if p[2] is not None]
        stems = [p for p in parts if p[2] is None]
        if len(berries) != 1 or len(stems) != 1:
            print('  skip {}: not a single bunch with one stem'.format(
                file_name))
            continue
        berry_mesh, berry_colors = berries[0][1], berries[0][2]
        stem_mesh = stems[0][1]
        # a double bunch is two clusters side by side
        extent = berry_mesh.bounds[1] - berry_mesh.bounds[0]
        if max(extent[0], extent[1]) > DOUBLE_BUNCH_WIDTH:
            print('  skip {}: double bunch'.format(file_name))
            continue
        variety = bunch_variety(file_name)
        name = BUNCH_MODEL_PREFIX + variety
        model_dir = os.path.join(models_dir, name)
        meshes_dir = os.path.join(model_dir, 'meshes')
        shutil.rmtree(meshes_dir, ignore_errors=True)
        grasp_z = 0.5 * (berry_mesh.bounds[1][2] + stem_mesh.bounds[1][2])
        for mesh in (berry_mesh, stem_mesh):
            mesh.apply_translation([0.0, 0.0, -grasp_z])
        write_obj(meshes_dir, 'berries', berry_mesh,
                  color=linear_to_srgb(berry_colors[:, :3].mean(axis=0)))
        write_obj(meshes_dir, 'stem', stem_mesh)

        model = ET.Element('model', name=name)
        link = sub(model, 'link', name='bunch')
        sub(link, 'gravity', 'false')
        bounds = berry_mesh.bounds
        size = bounds[1] - bounds[0]
        center = 0.5 * (bounds[0] + bounds[1])
        inertial = sub(link, 'inertial')
        sub(inertial, 'pose', fmt(list(center) + [0, 0, 0]))
        sub(inertial, 'mass', '{:.6g}'.format(BUNCH_MASS))
        inertia = sub(inertial, 'inertia')
        for axis, (a, b) in zip(('ixx', 'iyy', 'izz'),
                                ((1, 2), (0, 2), (0, 1))):
            sub(inertia, axis, '{:.6g}'.format(
                BUNCH_MASS * (size[a] ** 2 + size[b] ** 2) / 12.0))
        for axis in ('ixy', 'ixz', 'iyz'):
            sub(inertia, axis, '0')
        collision = sub(link, 'collision', name='berries')
        sub(collision, 'pose', fmt(list(center) + [0, 0, 0]))
        box_geometry(collision, size)
        for part in ('berries', 'stem'):
            mesh_visual(link, part, 'model://{}/meshes/{}.obj'.format(
                name, part))
        write_model(model_dir, name,
                    'A grape bunch ({}); the origin is the grasp point on '
                    'the stem.'.format(variety), model)
        varieties.append(variety)
        print('  {}'.format(name))
    return varieties


def make_crate_model(models_dir, size):
    """Write a static open-top wooden crate made of slats.

    Parameters
    ----------
    models_dir : str
        Directory of the models.
    size : array_like
        Outer size ``[x, y, height]`` [m].
    """
    sx, sy, height = size
    thickness = 0.02
    n_slats = 3
    gap = 0.015
    slat_height = (height - thickness - (n_slats - 1) * gap) / n_slats
    model = ET.Element('model', name=CRATE_MODEL)
    sub(model, 'static', 'true')
    link = sub(model, 'link', name='crate')

    def box(name, pos, box_size, collision):
        visual = sub(link, 'visual', name=name)
        sub(visual, 'pose', fmt(list(pos) + [0, 0, 0]))
        box_geometry(visual, box_size)
        material = sub(visual, 'material')
        script = sub(material, 'script')
        sub(script, 'uri', 'file://media/materials/scripts/gazebo.material')
        sub(script, 'name', 'Gazebo/Wood')
        if collision:
            element = sub(link, 'collision', name=name)
            sub(element, 'pose', fmt(list(pos) + [0, 0, 0]))
            box_geometry(element, box_size)

    box('bottom', (0, 0, thickness / 2.0), (sx, sy, thickness), True)
    walls = (('front', (sx / 2.0 - thickness / 2.0, 0), (thickness, sy)),
             ('back', (-sx / 2.0 + thickness / 2.0, 0), (thickness, sy)),
             ('left', (0, sy / 2.0 - thickness / 2.0), (sx, thickness)),
             ('right', (0, -sy / 2.0 + thickness / 2.0), (sx, thickness)))
    for name, (x, y), (wx, wy) in walls:
        for i in range(n_slats):
            z = thickness + slat_height / 2.0 + i * (slat_height + gap)
            box('{}_slat{}'.format(name, i), (x, y, z),
                (wx, wy, slat_height), False)
        # one collision per wall, the gaps between the slats are not needed
        collision = sub(link, 'collision', name=name)
        sub(collision, 'pose',
            fmt([x, y, thickness + (height - thickness) / 2.0, 0, 0, 0]))
        box_geometry(collision, (wx, wy, height - thickness))
    write_model(os.path.join(models_dir, CRATE_MODEL), CRATE_MODEL,
                'An open-top wooden harvest crate.', model)


def cylinder_pose_between(p0, p1):
    """Return the SDF pose of a cylinder from ``p0`` to ``p1``."""
    p0 = np.asarray(p0, dtype=np.float64)
    d = np.asarray(p1, dtype=np.float64) - p0
    length = np.linalg.norm(d)
    center = p0 + d / 2.0
    pitch = math.acos(np.clip(d[2] / length, -1.0, 1.0))
    yaw = math.atan2(d[1], d[0])
    return list(center) + [0.0, pitch, yaw], length


def nearest_row_direction(point, rows):
    """Horizontal unit vector from ``point`` to the axis of the nearest row."""
    best = None
    for row in rows:
        center = np.asarray(row['center'], dtype=np.float64)
        axis = np.array([math.cos(row['yaw']), math.sin(row['yaw'])])
        offset = center - np.asarray(point[:2])
        perpendicular = offset - offset.dot(axis) * axis
        distance = np.linalg.norm(perpendicular)
        if distance > 1e-6 and (best is None or distance < best[0]):
            best = (distance, perpendicular / distance)
    if best is None:
        raise RuntimeError('no vine row to hang a bunch from')
    return best[1]


def include(world, uri, name, pose, static=None):
    element = sub(world, 'include')
    sub(element, 'uri', uri)
    sub(element, 'name', name)
    sub(element, 'pose', fmt(pose))
    if static is not None:
        sub(element, 'static', 'true' if static else 'false')


def make_world(config, varieties, world_path):
    """Write the vineyard world from the demo layout.

    Parameters
    ----------
    config : dict
        Contents of ``GrapeHarvestDemo.yaml``.
    varieties : list of str
        Varieties of the bunch models.
    world_path : str
        Output world file.
    """
    field = config['field']
    rows = field['vine_rows']
    sdf = ET.Element('sdf', version='1.6')
    world = sub(sdf, 'world', name='grape_vineyard')

    physics = sub(world, 'physics', type='ode')
    sub(physics, 'max_step_size', '0.001')
    sub(physics, 'real_time_factor', '1')
    sub(physics, 'real_time_update_rate', '1000')
    sub(world, 'gravity', '0 0 -9.8')
    sub(world, 'magnetic_field', '34 0 -36')
    spherical = sub(world, 'spherical_coordinates')
    sub(spherical, 'surface_model', 'EARTH_WGS84')
    for tag in ('latitude_deg', 'longitude_deg', 'elevation', 'heading_deg'):
        sub(spherical, tag, '0')

    scene = sub(world, 'scene')
    sub(scene, 'ambient', '0.55 0.55 0.55 1')
    sub(scene, 'background', '0.62 0.78 0.92 1')
    sub(scene, 'shadows', 'true')
    sub(sub(sub(scene, 'sky'), 'clouds'), 'speed', '4')
    gui = sub(world, 'gui', fullscreen='0')
    camera = sub(gui, 'camera', name='user_camera')
    sub(camera, 'pose', '-2.2 -3.4 2.4 0 0.3 0.78')

    sun = sub(world, 'light', name='sun', type='directional')
    sub(sun, 'cast_shadows', 'true')
    sub(sun, 'pose', '0 0 10 0 0 0')
    sub(sun, 'diffuse', '0.9 0.88 0.8 1')
    sub(sun, 'specular', '0.2 0.2 0.2 1')
    sub(sun, 'direction', '-0.4 0.3 -0.85')
    attenuation = sub(sun, 'attenuation')
    sub(attenuation, 'range', '1000')
    sub(attenuation, 'constant', '0.9')
    sub(attenuation, 'linear', '0.01')
    sub(attenuation, 'quadratic', '0.001')

    # ground: grass, with bare soil along the rows
    ground = sub(world, 'model', name='vineyard_ground')
    sub(ground, 'static', 'true')
    link = sub(ground, 'link', name='ground')
    collision = sub(link, 'collision', name='ground')
    plane = sub(sub(collision, 'geometry'), 'plane')
    sub(plane, 'normal', '0 0 1')
    sub(plane, 'size', '200 200')
    friction = sub(sub(sub(collision, 'surface'), 'friction'), 'ode')
    sub(friction, 'mu', '100')
    sub(friction, 'mu2', '50')
    visual = sub(link, 'visual', name='grass')
    plane = sub(sub(visual, 'geometry'), 'plane')
    sub(plane, 'normal', '0 0 1')
    sub(plane, 'size', '200 200')
    script = sub(sub(visual, 'material'), 'script')
    sub(script, 'uri', 'file://media/materials/scripts/gazebo.material')
    sub(script, 'name', 'Gazebo/Grass')
    for i, row in enumerate(rows):
        visual = sub(link, 'visual', name='soil{}'.format(i))
        sub(visual, 'pose', fmt(list(row['center']) + [0.001, 0, 0,
                                                       row['yaw']]))
        box_geometry(visual, (6.8, 1.3, 0.002))
        material = sub(visual, 'material')
        sub(material, 'ambient', '0.36 0.27 0.18 1')
        sub(material, 'diffuse', '0.42 0.32 0.22 1')
        sub(material, 'specular', '0 0 0 1')

    for i, row in enumerate(rows):
        include(world, 'model://' + VINE_ROW_MODEL, 'vine_row{}'.format(i),
                list(row['center']) + [0.0, 0.0, 0.0, row['yaw']])

    for i, crate in enumerate(field['crates']):
        include(world, 'model://' + CRATE_MODEL, 'crate{}'.format(i),
                list(crate['position']) + [0.0, 0.0, crate.get('yaw', 0.0)])

    # bunches, each on a cane reaching into the canopy
    canes = sub(world, 'model', name='bunch_canes')
    sub(canes, 'static', 'true')
    cane_link = sub(canes, 'link', name='canes')
    for i, bunch in enumerate(field['bunches']):
        if bunch['name'] not in varieties:
            raise RuntimeError(
                'no bunch model for {} (available: {})'.format(
                    bunch['name'], varieties))
        stem = np.asarray(bunch['stem'], dtype=np.float64)
        include(world, 'model://' + BUNCH_MODEL_PREFIX + bunch['name'],
                '{}{}_{}'.format(HARVEST_BUNCH_PREFIX, i, bunch['name']),
                list(stem) + [0.0, 0.0, 0.0])
        direction = nearest_row_direction(stem, rows)
        top = stem + [0.0, 0.0, 0.02]
        end = top + np.append(0.35 * direction, 0.12)
        pose, length = cylinder_pose_between(top, end)
        visual = sub(cane_link, 'visual', name='cane{}'.format(i))
        sub(visual, 'pose', fmt(pose))
        cylinder = sub(sub(visual, 'geometry'), 'cylinder')
        sub(cylinder, 'radius', '0.005')
        sub(cylinder, 'length', '{:.4f}'.format(length))
        material = sub(visual, 'material')
        sub(material, 'ambient', '0.25 0.17 0.08 1')
        sub(material, 'diffuse', '0.3 0.2 0.1 1')

    os.makedirs(os.path.dirname(world_path), exist_ok=True)
    header = ('<?xml version="1.0"?>\n'
              '<!-- generated by script/make_grape_vineyard_world.py from '
              'config/grape_with_arm/GrapeHarvestDemo.yaml -->\n')
    with open(world_path, 'w') as f:
        f.write(header + ET.tostring(indent(sdf), encoding='unicode') + '\n')


def parse_args(package_dir):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--asset-dir', default=None,
                        help='directory of the glTF assets; if omitted, '
                        'only the world is written from the existing models')
    parser.add_argument('--config', default=os.path.join(
        package_dir, 'config', 'grape_with_arm', 'GrapeHarvestDemo.yaml'))
    parser.add_argument('--models-dir',
                        default=os.path.join(package_dir, 'models'))
    parser.add_argument('--world', default=os.path.join(
        package_dir, 'worlds', 'grape_vineyard.world'))
    return parser.parse_args()


def main():
    package_dir = rospkg.RosPack().get_path('gimbalrotor')
    args = parse_args(package_dir)
    with open(args.config) as f:
        config = yaml.safe_load(f)

    if args.asset_dir is not None:
        print('vine row')
        make_vine_row_model(args.asset_dir, args.models_dir)
        print('bunches')
        make_bunch_models(args.asset_dir, args.models_dir)
    print('crate')
    make_crate_model(args.models_dir, config['field']['crate_size'])

    varieties = sorted(
        d[len(BUNCH_MODEL_PREFIX):] for d in os.listdir(args.models_dir)
        if d.startswith(BUNCH_MODEL_PREFIX)
        and os.path.exists(os.path.join(args.models_dir, d, 'model.sdf')))
    if not os.path.exists(os.path.join(args.models_dir, VINE_ROW_MODEL,
                                       'model.sdf')) or not varieties:
        sys.exit('models are not found in {}; pass --asset-dir'.format(
            args.models_dir))
    make_world(config, varieties, args.world)
    print('wrote {}'.format(args.world))


if __name__ == '__main__':
    main()
