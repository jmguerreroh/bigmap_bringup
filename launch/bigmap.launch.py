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

"""BigMap on a robot: the DEM of the place, EasyNav with the BigMap maps manager, and RViz.

This is the application. It runs the same on a real robot and on a simulated one: it knows nothing
of Gazebo. The robot (real or simulated, see easynav_playground_geo) provides the sensors, the
odometry and the TF; this makes the map and navigates. bigmap_geo.launch.py starts both.

* config:=<name> picks the use case, config/params_<name>.yaml: outdoor (GPS, NavMap), indoor (a saved
  NavMap and AMCL), parking_garage, large_warehouse, two_floors. params_file:=<file> is any complete
  parameter file instead.
* A file with a `dem_generator` block (outdoors) gets its map from the DEM of a place:
  navmap_gis_tool makes it (lat:= lon:= radius:= pick the place, else the file's) and EasyNav starts
  with the origin of the DEM. initial_navmap:=<file> starts from a saved NavMap instead.
* sim:=true follows the last simulation launched: easynav_playground_geo (the map is made of the
  mesh of its world, ~/.ros/easynav_geo/last_world.yaml) or easynav_playground_bigmap (the place or
  mesh it wrote to ~/.ros/bigmap/sim/sim.yaml). use_sim_time is true, and EasyNav waits for the
  robot's transform.
"""

import glob
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from bigmap_bringup import dem as bigmap_dem
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

OVERRIDES_FILE = os.path.expanduser('~/.ros/bigmap/easynav_overrides.yaml')


def configs(bringup):
    """The use cases: the names of config/params_<name>.yaml."""
    names = []
    for path in glob.glob(os.path.join(bringup, 'config', 'params_*.yaml')):
        names.append(os.path.basename(path)[len('params_'):-len('.yaml')])
    return sorted(names)


def block(params_file, node, key='ros__parameters'):
    """What the parameter file has under `node`: {} when there is nothing."""
    import yaml
    with open(params_file) as f:
        data = yaml.safe_load(f) or {}
    return (data.get(node) or {}).get(key) or {}


def has_dem(params_file):
    """Whether the file makes its map from the DEM of a place (it has a dem_generator block)."""
    return bool(block(params_file, 'dem_generator'))


def uses_ukf(params_file):
    """Whether the file's localizer is the GPS one, which needs the origin of the DEM."""
    return 'ukf' in block(params_file, 'localizer_node')


def start_when_robot_is_up(easynav):
    """EasyNav starts when the robot's odometry transform exists (tools/wait_for_robot.py), not
    after a fixed delay: a slow simulation (the warehouse) would otherwise start AMCL before it."""
    wait = ExecuteProcess(
        cmd=[os.path.join(get_package_prefix('bigmap_bringup'), 'lib', 'bigmap_bringup',
                          'wait_for_robot.py'), '--frame', 'base_footprint', '--timeout', '120'],
        name='wait_for_robot', output='screen')
    return [wait, RegisterEventHandler(OnProcessExit(target_action=wait, on_exit=[easynav]))]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'config', default_value='outdoor',
            description='The use case: config/params_<config>.yaml of this package (outdoor, indoor, '
                        'parking_garage, large_warehouse, two_floors)'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Complete parameter file (maps manager, EasyNav stack and, outdoors, the '
                        'dem_generator block). Default: config/params_<config>.yaml'),
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='true: follow the last simulation launched (easynav_playground_geo: the mesh '
                        'of its world; easynav_playground_bigmap: ~/.ros/bigmap/sim/sim.yaml); '
                        'use_sim_time true; EasyNav waits for the robot. false: a real robot'),
        DeclareLaunchArgument(
            'area_holes', default_value='false',
            description='sim:=true with a world of easynav_playground_geo: true, its buildings, walls '
                        'and trunks are holes of the DEM over the ground (exact footprints); false, '
                        'blocks of it (steep NavCels one cell wide)'),
        DeclareLaunchArgument('lat', default_value='',
                              description='Latitude of the centre of the map (= its origin). Not given: '
                                          'the parameter file\'s place (or the simulation\'s)'),
        DeclareLaunchArgument('lon', default_value='', description='Longitude of the centre'),
        DeclareLaunchArgument('radius', default_value='',
                              description='Half-side of the square area, m (default: the file\'s)'),
        DeclareLaunchArgument(
            'use_synthetic_terrain', default_value='',
            description='true: offline, fake imagery and terrain. Empty: the parameter file\'s '
                        'dem_generator.use_synthetic_terrain'),
        DeclareLaunchArgument(
            'initial_navmap', default_value='',
            description='Start from this saved NavMap (any resolution) instead of building a DEM. If '
                        '<map>.yaml exists next to it (georeferenced: true) it gives the UKF origin'),
        DeclareLaunchArgument('use_sim_time', default_value='',
                              description='true or false for EasyNav and RViz. Empty: true with sim:=true (false '
                                          'for the kinematic vehicles, which have no /clock), '
                                          'false with sim:=false'),
        DeclareLaunchArgument('wait_for_robot', default_value='',
                              description='true: EasyNav starts when the transform of the robot exists '
                                          '(default with sim:=true); false: right away'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument(
            'rviz_config', default_value='',
            description='RViz2 configuration file. Default: rviz/bigmap.rviz'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=setup)])


def setup(context):
    bringup = get_package_share_directory('bigmap_bringup')

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    sim = arg('sim') == 'true'
    config = arg('config')
    params_file = os.path.expanduser(arg('params_file')) or os.path.join(
        bringup, 'config', 'params_%s.yaml' % config)
    if not os.path.isfile(params_file):
        raise RuntimeError("config must be one of %s, not '%s' (no %s)" % (
            ', '.join(configs(bringup)), config, params_file))
    initial_navmap = os.path.expanduser(arg('initial_navmap'))
    dem = has_dem(params_file)
    if bool(arg('lat')) != bool(arg('lon')):
        raise RuntimeError('lat:= and lon:= go together')
    if not dem and (arg('lat') or arg('use_synthetic_terrain')):
        raise RuntimeError('lat:= lon:= radius:= use_synthetic_terrain:= are for a parameter file '
                           'with a dem_generator block (%s has none: its map is a saved NavMap)' %
                           os.path.basename(params_file))
    if initial_navmap and not dem:
        raise RuntimeError(
            'the NavMap of %s is in the file (terrain.package + terrain.initial_navmap): edit it, or '
            'pass your own params_file:=<yaml>, instead of initial_navmap:=' % os.path.basename(params_file))
    if initial_navmap and arg('lat'):
        raise RuntimeError('initial_navmap:= is the map: it does not go with lat:=')
    if not dem and not bigmap_dem.initial_navmap_of(params_file):
        raise RuntimeError('set maps_manager_node.bigmap.terrain.initial_navmap in %s (or pass a '
                           'parameter file that has it with params_file:=)' % params_file)

    sim_time = arg('use_sim_time')
    if sim_time not in ('', 'true', 'false'):
        raise RuntimeError("use_sim_time must be true or false, not '%s'" % sim_time)
    # The clock is the launch's decision, not the parameter file's (the same file serves a real
    # robot and a simulation): sim:=true is the simulated clock, except for a simulation that is
    # no simulator (the kinematic vehicles have no /clock; sim.yaml says use_sim_time: false);
    # sim:=false is the wall clock. A given use_sim_time:= wins. A ROS clock that never starts
    # ("Waiting for clock to start...", or every EasyNav cycle silently never due) is the result
    # of getting this wrong.
    if arg('area_holes') not in ('true', 'false'):
        raise RuntimeError("area_holes must be true or false, not '%s'" % arg('area_holes'))
    info = (bigmap_dem.read_simulation(holes=arg('area_holes') == 'true')
            if sim and dem and not initial_navmap else {})
    if sim_time:
        time_param = [{'use_sim_time': sim_time == 'true'}]
    else:
        time_param = [{'use_sim_time': sim and info.get('use_sim_time') is not False}]
    wait = arg('wait_for_robot')
    wait = sim if wait == '' else wait == 'true'

    rviz_config = arg('rviz_config') or os.path.join(bringup, 'rviz', 'bigmap.rviz')
    rviz = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2', executable='rviz2', output='screen', arguments=['-d', rviz_config],
        parameters=time_param)

    def easynav(with_overrides):
        return Node(
            package='easynav_system', executable='system_main', output='screen',
            # the overrides file (the DEM's origin, or the saved map) comes last: it wins over the file
            parameters=[params_file] + ([OVERRIDES_FILE] if with_overrides else []) + time_param,
            on_exit=EmitEvent(event=Shutdown()))

    def start(node):
        return (start_when_robot_is_up(node) if wait else [node]) + [rviz]

    if initial_navmap:
        return start_from_navmap(initial_navmap, params_file, easynav(True), start)
    if not dem:
        # a saved NavMap and AMCL: everything is in the file
        return [LogInfo(msg='config %s: the maps manager loads its NavMap, nothing is generated' %
                        os.path.basename(params_file))] + start(easynav(False))

    # ---- the map is the DEM of a place
    overrides = {}
    if sim:
        overrides.update(info.get('dem') or {})
        dem_of_sim = info.get('dem') or {}
        print('bigmap.launch.py: following the simulation of %s written at %s (%s, %s, %s, %s): '
              'its place and terrain' % (
                  info.get('simulation', '?'), info.get('written', '?'), info.get('robot', '?'),
                  info.get('terrain', '?'),
                  dem_of_sim.get('gps_origin.latitude', '?'),
                  dem_of_sim.get('gps_origin.longitude', '?')))
    if arg('lat'):
        radius = arg('radius') or str(bigmap_dem.dem_parameters(params_file, overrides).get(
            'download_radius', 500.0))
        overrides.update(bigmap_dem.place_overrides(arg('lat'), arg('lon'), radius))
        if not bigmap_dem.in_spain(float(arg('lat')), float(arg('lon'))):
            print('bigmap.launch.py: %s, %s is outside Spain: heights from Copernicus (30 m), '
                  'imagery from Esri' % (arg('lat'), arg('lon')))
    elif arg('radius'):
        overrides['download_radius'] = float(arg('radius'))
    if arg('use_synthetic_terrain'):
        overrides['use_synthetic_terrain'] = arg('use_synthetic_terrain') == 'true'
    overrides['gazebo.export'] = False      # the world is the simulation's business
    dem_params = bigmap_dem.dem_parameters(params_file, overrides)
    if dem_params.get('base_navmap.enabled'):
        print('bigmap.launch.py: base_navmap.enabled is not supported any more (the maps manager '
              'builds its NavMap from the DEM file): ignored')
    dem_file = os.path.expanduser(str(dem_params.get('cache_file') or ''))   # '' is refused below
    if dem_params.get('elevation_source') == 'mesh' and os.path.isfile(dem_file) and os.path.isfile(
            os.path.expanduser(str(dem_params.get('mesh_file', '')))) and os.path.getmtime(
            dem_file) < os.path.getmtime(os.path.expanduser(str(dem_params['mesh_file']))):
        # navmap_gis_tool caches the DEM by lat/lon/radius/resolution, not by what the mesh holds: a
        # cache older than the mesh is of another world (the NavMap would not match)
        print('bigmap.launch.py: DEM cache %s is older than its mesh: removed' % dem_file)
        os.remove(dem_file)
    generator = ExecuteProcess(
        cmd=bigmap_dem.dem_generator_command(dem_params), name='dem_generator', output='screen')
    ukf = uses_ukf(params_file)

    def after_generator(event, context):
        if event.returncode != 0:
            return [LogInfo(msg='navmap_gis_tool failed (see its error above): shutting down'),
                    EmitEvent(event=Shutdown())]
        if ukf:
            try:
                bigmap_dem.write_easynav_overrides(dem_file, OVERRIDES_FILE)
            except Exception as e:
                return [LogInfo(msg='the DEM file cannot be read (%s): shutting down' % e),
                        EmitEvent(event=Shutdown())]
        return start(easynav(ukf))

    return [LogInfo(msg='config %s: the map is the DEM of %s, %s' % (
                os.path.basename(params_file), dem_params.get('gps_origin.latitude'),
                dem_params.get('gps_origin.longitude'))),
            generator,
            RegisterEventHandler(OnProcessExit(target_action=generator, on_exit=after_generator))]


def start_from_navmap(navmap_file, params_file, easynav, start):
    """No DEM is generated: the maps manager loads this NavMap and works from it."""
    if not os.path.isfile(navmap_file):
        raise RuntimeError('initial_navmap: no such file: %s' % navmap_file)
    # <map>.yaml next to the map: where its (0, 0, 0) is. Absent: no geolocation, origin 0, 0, 0.
    origin = {}
    yaml_file = os.path.splitext(navmap_file)[0] + '.yaml'
    if os.path.isfile(yaml_file):
        with open(yaml_file) as f:
            for line in f:
                line = line.split('#')[0]
                if ':' in line:
                    k, v = line.split(':', 1)
                    origin[k.strip()] = v.strip()
    geo = origin.get('georeferenced') == 'true' and 'origin_lat' in origin and 'origin_lon' in origin
    # a generated overrides file carries the map and its geolocation (UKF origin)
    lines = ['maps_manager_node:', '  ros__parameters:', '    bigmap:', '      terrain:',
             '        initial_navmap: "%s"' % navmap_file]
    if not geo:
        print('bigmap.launch.py: %s has no georeferenced <map>.yaml: the UKF keeps the origin of '
              'the parameter file (localizer_node.ukf.*_origin)' % navmap_file)
    else:
        try:
            lines += ['localizer_node:', '  ros__parameters:', '    ukf:',
                      '      latitude_origin: %.8f' % float(origin['origin_lat']),
                      '      longitude_origin: %.8f' % float(origin['origin_lon']),
                      '      altitude_origin: %.8f' % float(origin.get('reference_altitude', 0.0))]
        except ValueError as e:
            raise RuntimeError('%s: origin_lat, origin_lon and reference_altitude must be numbers '
                               '(%s)' % (yaml_file, e))
    os.makedirs(os.path.dirname(OVERRIDES_FILE), exist_ok=True)
    with open(OVERRIDES_FILE, 'w') as f:
        f.write('# Generated by bigmap.launch.py: the NavMap and its geolocation\n')
        f.write('\n'.join(lines) + '\n')
    info = LogInfo(msg='initial NavMap: %s (%s)' % (
        navmap_file, 'georeferenced' if geo else 'no geolocation'))
    return [info] + start(easynav)
