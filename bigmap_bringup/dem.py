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

"""What BigMap's application and the simulations share: how the DEM of a place is made.

The `dem_generator` block of a parameter file becomes a navmap_gis_tool command (`dem_generator_command`);
the place (lat, lon, radius) of a launch argument becomes the overrides that go on top of that block
(`place_overrides`). When the application follows a simulation (`read_simulation`), the map is made of
what the simulation's world is made of, so that the map of the robot and the world it moves in are of
the same place:

* easynav_playground_geo writes `~/.ros/easynav_geo/last_world.yaml` (the place and the mesh of its
  world): the DEM is rasterised from that mesh (`geo_sim_info`).
* easynav_playground_bigmap (the simulation of the article's campaigns) writes `~/.ros/bigmap/sim/sim.yaml`
  with the `dem_generator` overrides it used (`write_sim_info`, `read_sim_info`).
"""

import os

SIM_DIR = os.path.expanduser('~/.ros/bigmap/sim')
SIM_INFO = os.path.join(SIM_DIR, 'sim.yaml')
GEO_LAST_WORLD = os.path.expanduser('~/.ros/easynav_geo/last_world.yaml')
GEO_KEYS = ('world', 'directory', 'latitude', 'longitude', 'radius', 'ground_elevation')


def initial_navmap_of(params_file):
    """terrain.initial_navmap of the parameter file, as the maps manager reads it: relative to
    terrain.package's share directory if that is set. '' if there is none."""
    import yaml
    from ament_index_python.packages import get_package_share_directory
    with open(params_file) as f:   # a missing or broken file is an error of its own, not "unset"
        data = yaml.safe_load(f)
    try:
        terrain = data['maps_manager_node']['ros__parameters']['bigmap']['terrain']
    except (KeyError, TypeError):
        return ''
    path = os.path.expanduser(terrain.get('initial_navmap') or '')
    if path and terrain.get('package') and not os.path.isabs(path):
        # PackageNotFoundError names the package: a typo in terrain.package shows as such
        path = os.path.join(get_package_share_directory(terrain['package']), path)
    return path


def amcl_start(params_file):
    """(x, y) of localizer_node.amcl.initial_pose in the parameter file, or (0, 0)."""
    import yaml
    with open(params_file) as f:
        data = yaml.safe_load(f)
    try:
        localizer = data['localizer_node']['ros__parameters']
    except (KeyError, TypeError):
        return 0.0, 0.0
    if 'amcl' not in localizer:   # no AMCL (UKF outdoors, external localizer): the origin
        return 0.0, 0.0
    try:
        pose = localizer['amcl']['initial_pose']
        return float(pose['x']), float(pose['y'])
    except (KeyError, TypeError, ValueError):
        print('bigmap.launch.py: %s has amcl but no amcl.initial_pose {x, y}: the robot is spawned '
              'at (0, 0)' % params_file)
        return 0.0, 0.0


# Keys of the parameter file's `dem_generator` block that become options of navmap_gis_tool (the
# DEM generator), with the ones that only matter to the maps manager or are not used any more
DEM_KEYS = {
    'gps_origin.latitude', 'gps_origin.longitude', 'gps_origin.altitude', 'download_radius',
    'grid_resolution', 'elevation_source', 'elevation_resolution', 'use_synthetic_terrain',
    'mesh_file', 'mesh_world_elevation', 'holes_mesh_file', 'imagery.enabled', 'imagery.source',
    'imagery.url_template', 'imagery.zoom', 'imagery.scale', 'gazebo.export',
    'gazebo.world_template', 'gazebo.world_file', 'gazebo.terrain_dir',
    'gazebo.terrain_resolution', 'gazebo.sea_model', 'sea', 'cache_file', 'easynav.overrides_file',
}
DEM_IGNORED_KEYS = {
    'imagery.max_tiles', 'base_navmap.enabled', 'output_navmap_file', 'map_frame',
    'publish_topic', 'exit_when_done', 'use_sim_time',
}


def flatten(block, prefix=''):
    """{'a': {'b': 1}} -> {'a.b': 1}"""
    out = {}
    for k, v in (block or {}).items():
        if isinstance(v, dict):
            out.update(flatten(v, prefix + k + '.'))
        else:
            out[prefix + k] = v
    return out


def dem_parameters(params_file, *overrides):
    """The flattened `dem_generator` block of the parameter file, with `overrides` on top."""
    import yaml
    with open(params_file) as f:
        block = ((yaml.safe_load(f) or {}).get('dem_generator') or {}).get('ros__parameters') or {}
    params = flatten(block)
    for o in overrides:
        params.update(o)
    return params


def native_metres_per_pixel(zoom, lat):
    """Ground resolution of a Web Mercator tile pixel (256 px tiles)."""
    import math
    return 156543.03392804097 * math.cos(math.radians(lat)) / 2.0 ** zoom


def dem_generator_command(p, tool=None):
    """The navmap_gis_tool command line that makes the DEM (and the Gazebo terrain) from the
    flattened `dem_generator` parameters `p`."""
    unknown = sorted(set(p) - DEM_KEYS - DEM_IGNORED_KEYS)
    if unknown:
        raise RuntimeError('dem_generator: unknown parameters %s' % unknown)
    if tool is None:
        from ament_index_python.packages import get_package_prefix
        tool = os.path.join(get_package_prefix('navmap_tools'), 'lib', 'navmap_tools',
                            'navmap_gis_tool')
    lat = float(p.get('gps_origin.latitude', 0.0))
    lon = float(p.get('gps_origin.longitude', 0.0))
    source = str(p.get('elevation_source', 'ign'))
    if source == 'terrain_tiles':
        raise RuntimeError('elevation_source: terrain_tiles is now copernicus30 (the same kind of '
                           'data: worldwide, 30 m, no key)')
    if p.get('use_synthetic_terrain') and source != 'mesh':
        source = 'synthetic'
    if source not in ('ign', 'copernicus30', 'mesh', 'synthetic'):
        raise RuntimeError('elevation_source must be ign, copernicus30, mesh or synthetic, not %r'
                           % source)
    dem_file = os.path.expanduser(str(p.get('cache_file') or ''))
    if not dem_file:
        raise RuntimeError('dem_generator.cache_file is the DEM the maps manager reads: set it')
    cmd = [tool, '--center', '%r,%r' % (lat, lon),
           '--size', repr(2.0 * float(p.get('download_radius', 500.0))),
           '--resolution', repr(float(p.get('grid_resolution', 1.0))),
           '--dem-source', source, '--dem-out', dem_file]
    if float(p.get('elevation_resolution', 0.0)) > 0.0:
        cmd += ['--dem-data-resolution', repr(float(p['elevation_resolution']))]
    if float(p.get('gps_origin.altitude', 0.0)) != 0.0:
        cmd += ['--reference-altitude', repr(float(p['gps_origin.altitude']))]
    if source == 'mesh':
        if not p.get('mesh_file'):
            raise RuntimeError('elevation_source: mesh needs mesh_file')
        cmd += ['--mesh-file', os.path.expanduser(str(p['mesh_file'])),
                '--mesh-world-elevation', repr(float(p.get('mesh_world_elevation', 675.0))),
                '--mesh-cell', '0.5']
    if p.get('holes_mesh_file'):
        # buildings and walls as holes of the DEM (the cells they touch have no ground)
        cmd += ['--dem-holes-mesh', os.path.expanduser(str(p['holes_mesh_file']))]
    if p.get('imagery.enabled'):
        zoom = int(p.get('imagery.zoom', 18))
        imagery = str(p.get('imagery.source', 'auto'))
        if p.get('use_synthetic_terrain'):
            imagery = 'synthetic'
        cmd += ['--dem-imagery', '--zoom', str(zoom)]
        if imagery == 'esri':
            cmd += ['--imagery-source', 'esri']
        elif imagery in ('ign', 'auto'):
            # a tile of `scale` x 256 px: the PNOA orthophoto is asked at that pixel size
            cmd += ['--imagery-source', 'pnoa', '--imagery-resolution',
                    '%.4f' % (native_metres_per_pixel(zoom, lat) / int(p.get('imagery.scale', 2)))]
        elif imagery == 'xyz':
            if not p.get('imagery.url_template'):
                raise RuntimeError('imagery.source: xyz needs imagery.url_template')
            cmd += ['--imagery-source', 'xyz', '--imagery-url', str(p['imagery.url_template'])]
        elif imagery == 'synthetic':
            cmd += ['--imagery-source', 'synthetic']
        else:
            raise RuntimeError('imagery.source must be ign, esri, xyz or auto, not %r' % imagery)
    sea = p.get('sea')
    if isinstance(sea, bool):      # YAML 1.1 reads an unquoted on / off as true / false
        sea = 'on' if sea else 'off'
    if sea is not None and str(sea) != 'auto':
        if str(sea) not in ('on', 'off'):
            raise RuntimeError('dem_generator.sea must be auto, on or off, not %r' % sea)
        if str(sea) == 'on' and not has_sea_floor():
            raise RuntimeError('dem_generator.sea is on, but this navmap_tools has no sea floor '
                               '(navmap_tools.geo.sea): it cannot make the floor of the sea')
        if has_sea_floor():   # without it there is no --sea, and off is what it does anyway
            cmd += ['--sea', str(sea)]
    if p.get('gazebo.export'):
        if p.get('gazebo.sea_model') and has_sea_floor():    # dropped in the water, if any
            cmd += ['--sea-model', 'file://' + os.path.expanduser(str(p['gazebo.sea_model']))]
        if not p.get('gazebo.world_template'):
            raise RuntimeError('gazebo.export is true but gazebo.world_template is empty')
        cmd += ['--terrain-out', os.path.expanduser(str(p.get('gazebo.terrain_dir',
                                                              '~/.ros/bigmap/gazebo'))),
                '--terrain-resolution', repr(float(p.get('gazebo.terrain_resolution', 4.0))),
                '--world-template', os.path.expanduser(str(p['gazebo.world_template'])),
                '--world-out', os.path.expanduser(str(
                    p.get('gazebo.world_file', '~/.ros/bigmap/gazebo/world.sdf')))]
    return cmd


def has_sea_floor():
    """Whether navmap_tools can make the floor of the sea (navmap_tools.geo.sea, EMODnet): the
    NavMap of easynav_bigmap_ws does not have it."""
    import importlib.util
    try:
        return importlib.util.find_spec('navmap_tools.geo.sea') is not None
    except ModuleNotFoundError:
        return False


def write_easynav_overrides(dem_file, overrides_file):
    """The GPS localizer must use the very origin of the DEM (its lat, lon and the altitude of z = 0):
    the overrides file for EasyNav (it comes last in the parameters, so it wins over the file's)."""
    from navmap_tools.geo.dem_file import read_dem_file
    dem = read_dem_file(dem_file)
    # always with a decimal point: the localizer declares these as double, and a value such
    # as 680 would be read from the YAML as an integer
    write_whole(overrides_file,
                '# Generated by bigmap.launch.py: EasyNav parameters derived from the DEM\n'
                'localizer_node:\n  ros__parameters:\n    ukf:\n'
                '      latitude_origin: %.8f\n      longitude_origin: %.8f\n'
                '      altitude_origin: %.8f\n' % (dem.origin_lat, dem.origin_lon,
                                                    dem.reference_altitude))


def write_whole(path, text):
    """Writes `text` to `path` whole or not at all (a temporary file, then renamed): what another
    process reads (EasyNav, the application following a simulation) is never a file cut short."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        f.write(text)
    os.replace(tmp, path)


def spawn_height(dem_file):
    """z at which a ground robot is spawned at (0, 0) of the DEM: the terrain there plus 0.5 m."""
    from navmap_tools.geo.dem_file import read_dem_file
    return float(read_dem_file(dem_file).height_at(0.0, 0.0)) + 0.5


def in_spain(lat, lon):
    """Whether (lat, lon) is where the Spanish services (IGN terrain, Catastro) answer: the
    peninsula (Galicia reaches further west than Portugal's border to the south), the Balearic and
    the Canary Islands, Ceuta and Melilla, by generous boxes. The tool itself says if one is wrong."""
    return (
        (35.0 <= lat <= 44.0 and -7.5 <= lon <= 5.0) or
        (41.7 <= lat <= 44.0 and -9.4 <= lon <= -6.0) or      # Galicia
        (27.0 <= lat <= 29.5 and -19.0 <= lon <= -13.0))      # Canary Islands


def place_overrides(lat, lon, radius, imagery=True):
    """The `dem_generator` parameters of a place given by lat, lon and radius: in Spain what the
    parameter file says (IGN terrain and PNOA imagery), elsewhere Copernicus 30 m heights and Esri
    imagery (the Spanish services answer nowhere else). The altitude of the origin is 0: the DEM
    knows its own."""
    lat, lon = float(lat), float(lon)
    out = {'gps_origin.latitude': lat, 'gps_origin.longitude': lon,
           'download_radius': float(radius), 'gps_origin.altitude': 0.0}
    if not in_spain(lat, lon):
        out['elevation_source'] = 'copernicus30'
        if imagery:
            out.update({'imagery.source': 'esri', 'imagery.zoom': 18})
    return out


def area_overrides(area, holes=False):
    """The `dem_generator` parameters of a generated area (area_world.py): the NavMap is rasterised
    from the mesh it made, z = 0 is the ground at the centre. `area`: its area.yaml as a dict, with
    'dir' the directory of the area. `holes`: the buildings, walls and trunks are holes of the DEM
    (navmap_holes.stl) over the ground alone (navmap_ground.stl), instead of blocks of the mesh
    (navmap.stl) whose walls are steep NavCels."""
    mesh = 'navmap.stl'
    if holes:
        for name in ('navmap_ground.stl', 'navmap_holes.stl'):
            if not os.path.isfile(os.path.join(area['dir'], name)):
                raise RuntimeError('%s has no %s (made by an older generator): remove the '
                                   'directory and launch the simulation again' % (area['dir'], name))
        mesh = 'navmap_ground.stl'
    out = {'gazebo.export': False, 'elevation_source': 'mesh',
           'mesh_file': os.path.join(area['dir'], mesh),
           'mesh_world_elevation': float(area['ground_elevation']),
           'gps_origin.latitude': float(area['latitude']),
           'gps_origin.longitude': float(area['longitude']),
           'download_radius': float(area['radius'])}
    if holes:
        out['holes_mesh_file'] = os.path.join(area['dir'], 'navmap_holes.stl')
    if not in_spain(float(area['latitude']), float(area['longitude'])):
        # the PNOA orthophoto only covers Spain: elsewhere the ground colour (RViz) is Esri's
        out.update({'imagery.source': 'esri', 'imagery.zoom': 18})
    return out


def read_area(area_dir):
    """area.yaml of an area directory as a dict (plus 'dir')."""
    out = {'dir': area_dir}
    with open(os.path.join(area_dir, 'area.yaml')) as f:
        for line in f:
            if ':' in line and not line.startswith('#'):
                k, v = line.split(':', 1)
                out[k.strip()] = v.strip()
    return out


def area_dir_of(lat, lon, radius):
    """Where area_world.py puts the area of a place (the name it gives it)."""
    return os.path.expanduser('~/.ros/bigmap/areas/area_%.5f_%.5f_%d' % (
        float(lat), float(lon), int(float(radius))))


def write_sim_info(overrides, extra=None, path=SIM_INFO):
    """What the simulation made its world from: the `dem_generator` overrides (the place, the sea,
    the mesh of an area...) and, in `extra`, anything else (robot, environment). The application
    that follows the simulation reads it and makes its map from the same."""
    import yaml
    keep = {k: v for k, v in overrides.items() if not k.startswith('gazebo.') and k != 'cache_file'}
    write_whole(path, '# written by the simulation of easynav_playground_bigmap\n' +
                yaml.safe_dump({'dem': keep, **(extra or {})}, default_flow_style=False))
    return path


def read_sim_info(path=SIM_INFO):
    """The dict write_sim_info wrote, with 'written' (when, as text). A clear error if no
    simulation has, or if the file is not what a simulation writes (edited by hand, cut short)."""
    import time
    import yaml
    if not os.path.isfile(path):
        raise RuntimeError(
            'no simulation to follow (neither %s nor %s exists): start one first, for instance '
            'ros2 launch easynav_playground_geo easynav_geo.launch.yaml (or bigmap_geo.launch.py '
            'of this package, which starts both)' % (GEO_LAST_WORLD, path))
    with open(path) as f:
        try:
            info = yaml.safe_load(f) or {}
        except yaml.YAMLError as e:
            raise RuntimeError('%s is not the file a simulation writes (%s)' % (path, e))
    if not isinstance(info, dict) or not isinstance(info.get('dem', {}), dict):
        raise RuntimeError('%s is not the file a simulation writes (a mapping with a `dem` '
                           'mapping): start the simulation again' % path)
    unknown = sorted(set(info.get('dem', {})) - DEM_KEYS)
    if unknown:
        raise RuntimeError('%s has dem_generator keys this version does not know %s: start the '
                           'simulation again' % (path, unknown))
    info.setdefault('written', time.strftime('%Y-%m-%d %H:%M',
                                             time.localtime(os.path.getmtime(path))))
    return info


def read_geo_world(path=None):
    """last_world.yaml of easynav_playground_geo as a dict of strings, or a clear error."""
    path = path or GEO_LAST_WORLD
    out = {}
    with open(path) as f:
        for line in f:
            if ':' in line and not line.lstrip().startswith('#'):
                k, v = line.split(':', 1)
                out[k.strip()] = v.strip()
    missing = [k for k in GEO_KEYS if not out.get(k)]
    if missing:
        raise RuntimeError('%s is not the file easynav_playground_geo writes (no %s): start the '
                           'simulation again' % (path, ', '.join(missing)))
    return out


def geo_sim_info(world, holes=False, path=None):
    """What to follow of a world of easynav_playground_geo (`world`: its last_world.yaml), in the
    shape of read_sim_info: the DEM is rasterised from the world's mesh (navmap.stl, or its ground
    with the buildings, walls and trunks as holes), z = 0 is the ground at the centre."""
    import time
    area = {'dir': world['directory'], 'latitude': world['latitude'],
            'longitude': world['longitude'], 'radius': world['radius'],
            'ground_elevation': world['ground_elevation']}
    if not os.path.isfile(os.path.join(area['dir'], 'navmap.stl')):
        raise RuntimeError('%s has no navmap.stl: the world of the simulation is gone or '
                           'incomplete; launch the simulation again' % area['dir'])
    path = path or GEO_LAST_WORLD
    return {'dem': area_overrides(area, holes=holes), 'robot': 'summit', 'terrain': world['world'],
            'environment': 'land', 'use_sim_time': True, 'simulation': 'easynav_playground_geo',
            'written': time.strftime('%Y-%m-%d %H:%M', time.localtime(os.path.getmtime(path)))}


def read_simulation(holes=False):
    """The simulation to follow: the last one launched of easynav_playground_geo
    (GEO_LAST_WORLD) and easynav_playground_bigmap (SIM_INFO). `holes`: for a geo world, the
    buildings, walls and trunks are holes of the DEM instead of blocks of it."""
    geo = GEO_LAST_WORLD if os.path.isfile(GEO_LAST_WORLD) else None
    old = SIM_INFO if os.path.isfile(SIM_INFO) else None
    if geo and (old is None or os.path.getmtime(geo) >= os.path.getmtime(old)):
        return geo_sim_info(read_geo_world(geo), holes=holes, path=geo)
    info = read_sim_info(SIM_INFO)      # its own error when there is none
    info.setdefault('simulation', 'easynav_playground_bigmap')
    if holes:
        print('bigmap.launch.py: area_holes:= is for a world of easynav_playground_geo; the '
              'simulation of easynav_playground_bigmap decides it itself (its area_holes:=)')
    return info
