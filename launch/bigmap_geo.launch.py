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

"""BigMap in the world of any real place: the simulation and the application in one launch.

    ros2 launch bigmap_bringup bigmap_geo.launch.py lat:=46.0207 lon:=7.7491 radius:=300
    ros2 launch bigmap_bringup bigmap_geo.launch.py lat:=40.4168 lon:=-3.7038 radius:=250 world:=area

First easynav_playground_geo makes the world of the place (once; it is kept) and starts Gazebo with
the Summit XL in it; then bigmap.launch.py with sim:=true makes the map of that very world (the mesh
the playground wrote to ~/.ros/easynav_geo/last_world.yaml) and starts EasyNav, with the BigMap maps
manager, when the robot is up, and RViz. The two can also be launched apart, in that order.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# the arguments of easynav_playground_geo's launch, with its defaults (a test keeps them equal)
GEO_ARGS = {
    'world': ('terrain', 'terrain: the terrain and its imagery. area: plus buildings, walls and trees'),
    'lat': ('40.284796', 'Latitude of the centre (WGS84 degrees): the origin of the world and of '
                         'the map'),
    'lon': ('-3.821407', 'Longitude of the centre (WGS84 degrees)'),
    'radius': ('300', 'Half side of the square world, m (up to 2000)'),
    'trees': ('true', 'world:=area: trees (crowns found in the imagery)'),
    'walls': ('true', 'world:=area: walls, fences and hedges from OpenStreetMap'),
    'roof': ('ortho', 'world:=area: ortho, the roofs show the aerial imagery; flat, one colour'),
    'lidar': ('', "Spain only: PNOA-LiDAR .laz/.las file or directory ('' = "
                  '~/.ros/easynav_geo/lidar if it has sheets)'),
    'spawn_x': ('', "x of the Summit ('' = a free spot near the origin)"),
    'spawn_y': ('', 'y of the Summit'),
    'spawn_z': ('', "z of the Summit ('' = 0.5 m above the ground there)"),
    'gui': ('true', 'Gazebo GUI'),
    'gpu': ('auto', 'auto, nvidia or none: render on the NVIDIA GPU of a laptop'),
}

# the arguments of bigmap.launch.py that matter here (sim is always true)
APP_ARGS = {
    'config': ('outdoor', 'The use case, config/params_<config>.yaml: one that makes its map from '
                          'a DEM (a dem_generator block): outdoor'),
    'params_file': ('', 'A complete parameter file instead (with a dem_generator block)'),
    'area_holes': ('false', 'world:=area: true, the buildings, walls and trunks are holes of the map '
                            'over the ground (exact footprints); false, blocks of it'),
    'rviz': ('true', 'Start RViz2'),
    'rviz_config': ('', 'RViz2 configuration file. Default: rviz/bigmap.rviz'),
}


def params_file_of(bringup, config, params_file):
    """The parameter file the application will use, refused if it does not make its map from a
    DEM: in a world of a place the map must be that place's."""
    from bigmap_bringup import dem as bigmap_dem
    path = os.path.expanduser(params_file) or os.path.join(bringup, 'config',
                                                           'params_%s.yaml' % config)
    if not os.path.isfile(path):
        raise RuntimeError('no parameter file %s (config:=%s)' % (path, config))
    if not bigmap_dem.dem_parameters(path):
        raise RuntimeError(
            '%s has no dem_generator block: its map is a saved NavMap, not the world of the place. '
            'Use config:=outdoor or a file made from it' % os.path.basename(path))
    return path


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=d, description=t)
            for k, (d, t) in list(GEO_ARGS.items()) + list(APP_ARGS.items())]
    return LaunchDescription(args + [OpaqueFunction(function=setup)])


def setup(context):
    bringup = get_package_share_directory('bigmap_bringup')
    geo = get_package_share_directory('easynav_playground_geo')

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    params_file = params_file_of(bringup, arg('config'), arg('params_file'))
    # lat, lon, radius of this launch are the playground's: given to the application they would be
    # a place of their own (a DEM downloaded for it) instead of the mesh of the world
    app = {'params_file': params_file, 'sim': 'true', 'area_holes': arg('area_holes'),
           'rviz': arg('rviz'), 'rviz_config': arg('rviz_config'),
           'lat': '', 'lon': '', 'radius': ''}
    # In this order: the playground's launch makes the world and writes last_world.yaml while it
    # is set up, before the application's is, which reads it
    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(geo, 'launch', 'easynav_geo.launch.yaml')),
            launch_arguments={k: arg(k) for k in GEO_ARGS}.items()),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(bringup, 'launch', 'bigmap.launch.py')),
            launch_arguments=app.items()),
    ]
