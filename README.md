# bigmap_bringup

**BigMap on a robot, real or simulated.** It launches EasyNav with the BigMap maps manager
(`easynav_bigmap_maps_manager/BigMapMapsManager`, in `easynav_plugins`: a NavMap whose triangles are fine
around the robot and coarse far away), the DEM of the place the map is made of, and RViz; and it has the
complete parameter files of the use cases. The application knows nothing of Gazebo: the robot, real or
simulated, gives the sensors, the odometry and the TF. To use the plugin on your own robot, without this
package, see the README of `easynav_bigmap_maps_manager`: how to give it a map (indoors and outdoors) and
two example parameter files.

**BigMap in any place, in one launch.** `bigmap_geo.launch.py` starts the simulation of a real place
(`easynav_playground_geo`: the Gazebo world of its coordinates, with the Summit XL) and then the
application on that very world:

```bash
ros2 launch bigmap_bringup bigmap_geo.launch.py lat:=46.0207 lon:=7.7491 radius:=300                # terrain (Zermatt)
ros2 launch bigmap_bringup bigmap_geo.launch.py lat:=40.4168 lon:=-3.7038 radius:=250 world:=area   # + buildings, walls, trees
```

See [In any place: `bigmap_geo.launch.py`](#in-any-place-bigmap_geolaunchpy).

**The packages and how they meet.**

| Package | What it is | Talks to the others through |
|---|---|---|
| `easynav_bigmap_maps_manager` ([easynav_plugins](https://github.com/EasyNavigation/easynav_plugins)) | the plugin: LOD mesh (grid / hierarchy engine), obstacles, memory of obstacles, save services | its parameters `maps_manager_node.bigmap.*`, the NavState (`map.navmap`, `map.dynamic`), topics `/maps_manager_node/bigmap/*` |
| `bigmap_bringup` (this repository) | the application: `launch/bigmap.launch.py`, `launch/bigmap_geo.launch.py`, `config/params_*.yaml`, `maps/`, `rviz/`, and the Python module `bigmap_bringup.dem` (how a DEM is made) | runs `navmap_gis_tool` (the DEM generator, package `navmap_tools`) and EasyNav's `system_main` with one parameter file; nothing of the plugin is linked |
| `easynav_playground_geo` ([easynav_playgrounds](https://github.com/EasyNavigation/easynav_playgrounds)) | the simulation of any real place: the Gazebo world of its coordinates (terrain, optionally buildings, walls and trees), with the Summit XL | writes `~/.ros/easynav_geo/last_world.yaml` (the place and the mesh of its world), which the application reads with `sim:=true` |
| `navmap_tools` ([NavMap](https://github.com/EasyNavigation/NavMap)) | outdoors: `navmap_gis_tool --dem-out` makes the DEM (`terrain.dem_file`), from a place or from the mesh of a simulated world, then exits | the DEM cache file |

The launch only chooses the parameter file and starts the processes; every behaviour of the plugin is in
that file (see [Parameters](#parameters)).

The indoor simulations of the use cases below (the parking garage, the large warehouse, a world of
your own) and the article's campaigns use `easynav_playground_bigmap`, the earlier simulation package
of the BigMap workspace, which is not published: with `sim:=true` the application also follows it
(`~/.ros/bigmap/sim/sim.yaml`), whichever of the two simulations was launched last.

## Use cases: `config:=`

`config:=<name>` is `config/params_<name>.yaml`; `params_file:=<file>` is any complete parameter file instead.

| `config:=` | Map | Terrain | Localization | Meant for |
|---|---|---|---|---|
| `outdoor` (default) | NavMap | DEM of a place (IGN, or Copernicus elsewhere) | GPS + odometry + IMU (UKF) | an outdoor ground robot; the Summit in simulation |
| `indoor` | NavMap | your NavMap file (floors over each other, ramps) | external: your own localization gives `map -> odom` | an indoor real robot (a template to fill) |
| `parking_garage` | NavMap | the playground's two-floor garage | AMCL on its Bonxai map | simulation |
| `large_warehouse` | NavMap | built from the point cloud of `large_warehouse_world` (700 x 450 m, one floor) | AMCL | simulation |
| `two_floors` | NavMap | two floors, one start | AMCL | simulation |

The files have the same sections, names and order (see [Parameters](#parameters)).

## Build (ROS 2 Rolling)

Clone this repository in the `src/` of a workspace that has EasyNav, `easynav_plugins`, NavMap and
`easynav_playgrounds`. Outdoors the application needs `navmap_gis_tool --dem-out` (the DEM file and
the mesh source), which is in the pull request `navmap-tools-dem` of NavMap: until it is merged, use
that branch. One distro per shell, and always a shell where only Rolling is sourced (a binary built
with Rolling does not run under Jazzy: `undefined symbol ... rclcpp ...`):

```bash
cd <ws>/src && git clone https://github.com/jmguerreroh/bigmap_bringup.git && cd ..
source /opt/ros/rolling/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to bigmap_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PROJECT_navmap_rviz_plugin_INCLUDE=$PWD/src/bigmap_bringup/cmake/rolling_rviz_qt.cmake
source install/setup.bash
```

With `--symlink-install` the YAMLs, launch files and `maps/` are links to `src`: edit them without
rebuilding (the module `bigmap_bringup.dem` too). `package.xml` declares what the launch runs
(EasyNav's `system_main`, the plugins the YAMLs name, `easynav_playground_geo`), so `--packages-up-to`
builds them and rosdep brings the system ones.
`rolling_rviz_qt.cmake` is a workaround for the Rolling packages released for Ubuntu 24.04 (no
longer updated): their `rviz_common` is built against Qt5 but exports a bare `Qt5` dependency and
looks for Qt6 first. It only affects `navmap_rviz_plugin` and edits no third-party code; it is not
needed on a consistent Rolling (Ubuntu 26.04). Do not run two stacks on the same `ROS_DOMAIN_ID` /
`GZ_PARTITION`.

`maps/` is 180 MB: the NavMaps and the point clouds of the indoor use cases (AMCL localizes on the
clouds).

## Launch the application

```bash
# a real robot, outdoors: the DEM of its place, then EasyNav with the UKF origin there
ros2 launch bigmap_bringup bigmap.launch.py config:=outdoor lat:=40.4168 lon:=-3.7038 radius:=400
# a real robot, indoors: your NavMap is in the file (terrain.initial_navmap)
ros2 launch bigmap_bringup bigmap.launch.py params_file:=<yours.yaml>
# a simulated robot: start a simulation first, then follow it (or bigmap_geo.launch.py: both)
ros2 launch easynav_playground_geo easynav_geo.launch.yaml lat:=46.0207 lon:=7.7491 radius:=300
ros2 launch bigmap_bringup bigmap.launch.py config:=outdoor sim:=true
```

| Argument | Default | What it is |
|---|---|---|
| `config` | `outdoor` | the use case: `config/params_<config>.yaml` |
| `params_file` | `''` | a complete parameter file instead |
| `sim` | `false` | `true`: follow the last simulation launched: the mesh of the world of `easynav_playground_geo` (`~/.ros/easynav_geo/last_world.yaml`) or what `easynav_playground_bigmap` wrote (`~/.ros/bigmap/sim/sim.yaml`); `use_sim_time: true`, and EasyNav waits for the robot's transform (`tools/wait_for_robot.py`) |
| `area_holes` | `false` | `sim:=true` with a world of `easynav_playground_geo`: `true`, its buildings, walls and tree trunks are holes of the DEM over the ground (their exact footprints); `false`, blocks of it (steep NavCels one cell wide) |
| `lat`, `lon`, `radius` | `''` (the file's place) | the place of the map (the DEM): centre (= the origin of the map and of the UKF) and half-side, m. Outside Spain the heights are Copernicus 30 m and the imagery Esri |
| `use_synthetic_terrain` | `''` (the file's) | `true`: offline, fake imagery and terrain |
| `initial_navmap` | `''` | start from this saved NavMap instead of making a DEM (see below) |
| `use_sim_time` | `''` | `true` / `false` for EasyNav and RViz; empty: `true` with `sim:=true`, `false` with `sim:=false`: the clock is the launch's decision, not the file's |
| `wait_for_robot` | `''` | `true`: EasyNav starts when the robot's `odom -> base_footprint` exists (default with `sim:=true`); `false`: right away |
| `rviz`, `rviz_config` | `true`, `''` | RViz2 and its configuration (`rviz/bigmap.rviz`) |

What the launch does with a file that has a `dem_generator` block (outdoors): `navmap_gis_tool`
downloads the DEM (elevation and, for a NavMap, satellite imagery) once, caches it where the file says
(`dem_generator.cache_file`, also `terrain.dem_file` of the maps manager) and exits; then EasyNav
(`system_main`) starts with the file and an overrides file (`~/.ros/bigmap/easynav_overrides.yaml`: the UKF
origin, the lat, lon and altitude of the DEM's origin, when the file has a GPS localizer) and RViz2 opens. A
file without that block (indoors) has its NavMap in `terrain.initial_navmap`: nothing is generated. Refused
with a message, before anything starts: a place (`lat:=`...) for a file without `dem_generator`,
`initial_navmap:=` for a file that has its own map, `lat:=` without `lon:=`.

With `sim:=true` the DEM is made from what the simulation wrote, on top of the file's `dem_generator`
block, so the map of the robot and the world it is in are of the same place: for a world of
`easynav_playground_geo`, its mesh (`navmap.stl`: the terrain, and the buildings, walls and trunks of
an area) rasterised with the origin of the world; for `easynav_playground_bigmap`, its place or the
mesh of its area. The launch says which simulation it follows and when it was written. `lat:= lon:=`
given on top of it win. The application never writes a world.

## In any place: `bigmap_geo.launch.py`

```bash
ros2 launch bigmap_bringup bigmap_geo.launch.py lat:=46.0207 lon:=7.7491 radius:=300                # terrain
ros2 launch bigmap_bringup bigmap_geo.launch.py lat:=40.4168 lon:=-3.7038 radius:=250 world:=area   # + buildings, walls, trees
ros2 launch bigmap_bringup bigmap_geo.launch.py world:=area area_holes:=true gui:=false
```

It is the two launches above in one: first `easynav_playground_geo` makes the world of the place
(once: it is kept in `~/.ros/easynav_geo/worlds/`) and starts Gazebo with the Summit XL in it; then
`bigmap.launch.py sim:=true` makes the DEM of that world and starts EasyNav, when the robot is up,
and RViz. Send goals from RViz.

| Argument | Default | What it is |
|---|---|---|
| `world`, `lat`, `lon`, `radius`, `trees`, `walls`, `roof`, `lidar`, `spawn_x`, `spawn_y`, `spawn_z`, `gui`, `gpu` | those of `easynav_playground_geo` (the URJC campus at Fuenlabrada, 300 m, `terrain`) | the world: see the README of `easynav_playground_geo` |
| `config` | `outdoor` | the use case; it must make its map from a DEM (a `dem_generator` block): the others are refused with a message |
| `params_file` | `''` | a complete parameter file instead (with a `dem_generator` block) |
| `area_holes` | `false` | `world:=area`: the buildings, walls and trunks as holes of the map (see above) |
| `rviz`, `rviz_config` | `true`, `''` | RViz2 and its configuration |

The place of the map is the world's: `lat`, `lon`, `radius` go to the simulation only, and the
application takes the world's mesh, origin and elevation from `last_world.yaml`. Outside Spain the
map's colours are Esri's, as the world's imagery.

## Outdoor

Notes for `config:=outdoor`:
- Keep `gps_origin.altitude: 0.0` (z = 0 at the origin, where the robot spawns).
- The UKF runs in 3D with the wheel odometry's vz = 0 as an anchor; its lat/lon/altitude origin
  comes from the generated overrides file, not from `params_outdoor.yaml` (with `initial_navmap:=`
  only if the map has a georeferenced `<map>.yaml`; otherwise the file's `ukf.*_origin` are used
  and the launch says so).
- `use_synthetic_terrain:=true|false` overrides the file's value only when given.
- GPS localization drifts a few metres over tens of metres of path in simulation.
- Elevation: `elevation_source: ign` (Spain only, 5 m, LiDAR-derived, free, no key) or
  `copernicus30` (the whole world, 30 m, free, no key: how to use it outside Spain is in the
  README of `easynav_bigmap_maps_manager`). Imagery: `imagery.source: ign` (PNOA orthophoto, Spain) or `xyz`
  (a tile server of your choice). Outdoors needs no account or API key. The elevation is
  requested every `elevation_resolution` and interpolated (bicubic) to `grid_resolution`: small NavCels give a smooth surface, not new real detail.
- In simulation the map is made of the mesh of the simulated world: see
  [In any place](#in-any-place-bigmap_geolaunchpy) and the README of `easynav_playground_geo`.

## Indoor, multi-level

```bash
# real robot, no Gazebo: everything is in config/params_indoor.yaml (use_sim_time false, external
# localization; set terrain.initial_navmap there, or copy the file and pass params_file:=<yours>)
ros2 launch bigmap_bringup bigmap.launch.py config:=indoor

# simulation: the playground's parking garage (params_parking_garage.yaml has the maps manager, the
# NavMap, the Bonxai map, AMCL and its start pose; the playground spawns the robot there)
ros2 launch easynav_playground_bigmap easynav_bigmap_scenario.launch.yaml scenario:=parking_garage
ros2 launch bigmap_bringup bigmap.launch.py config:=parking_garage sim:=true
```

**Where the values are.** Every use case has its own complete parameter file and the launch loads
that file as it is: there is no hidden override file indoors. The simulation starts Gazebo and the
robot (at `localizer_node.amcl.initial_pose`, which it reads from the file); the application starts
EasyNav and RViz with the NavMap the file names: `terrain.package: bigmap_bringup` +
`terrain.initial_navmap: maps/<scenario>.navmap` (relative to that package's share directory, as the
Bonxai map's `package` + `bonxai_path_file`; with `--symlink-install` it is the one in `src`). Nothing
is copied or generated: indoors there is no overrides file, everything is in the YAML. Written to
`~/.ros/bigmap`: only the GPS maps, DEM caches and areas. Refused with a message, before anything
starts: `initial_navmap:=` indoors (edit the YAML), a place (`lat:=`...) for a file without a DEM.
`spawn_x:=` / `spawn_y:=` of the simulation move only the Gazebo robot (AMCL starts at the file's `initial_pose`).
Set the robot's lidar topic, `robot_geometry` (0.5 m for the
Summit XL: 0.7 m does not fit a 2 m ramp between curbs) and the LOD in the file.

For buildings the height grid of the outdoor mode (one height per x, y) cannot hold two floors, so
`multilevel.enabled: true` makes the LOD work on the NavMap's **own triangles** by hierarchical
vertex clustering (`SurfaceLodMesher`):

* level 0 = the map's vertices; level k merges those that fall in one cell of `cell_size.min *
  2^(k-1)`, are joined by the map's own edges inside that cell, and whose heights span at most
  `multilevel.cluster_height` (two floors are never merged, and the sides of a gap, a wall's
  footprint or a ledge are never joined, however coarse the mesh);
* per re-mesh each vertex climbs the tree while the policy allows (distance in 3D to the robot, with
  height weighing `vertical_weight` times, to the driven-over ground, and the cluster's error). The
  result is a cut of the tree: closed everywhere, no seams between floors and ramps;
* the border of the map (sides of a ramp, holes) only merges with border, so it keeps its place;
* the slope that makes a NavCel lethal is the **source's**, not the simplified geometry's;
* `terrain.keep_narrow_width` (works in both engines, multilevel or not; `multilevel.keep_narrow_width`
  is still read if the first is absent): ramps, corridors and doors narrower than this stay at the
  map's full resolution however far the robot is. Without it the planner cannot find the way through
  a ramp from far away (coarse NavCels leave no room for the robot's inflation). Hierarchy engine:
  width between the mesh's borders. Grid engine: width of the drivable ground (slope <=
  `max_slope_deg`, data present) between nodes that are not; those nodes and the ones around them
  are marked and propagated up the RTIN tree, so any triangle that holds one is split down to the
  DEM resolution (crack-free like the error criterion). Where passages are everywhere (aisles,
  forests) it keeps most of the map fine: leave it 0 there;
* `terrain.drop_unreachable_ground` (both engines): the roof and any piece not joined to the main
  map are dropped, so a goal picked on them (the topmost surface under the mouse) cannot happen. With the grid engine the piece is the
  largest connected ground with slope <= `max_slope_deg`; the faces of walls and slopes that touch
  it stay, as obstacles;
* driven-over ground is per floor (`vertical_tolerance`); the dynamic map and the ground fit use
  the NavCel at the robot's height; `dynamic_map.max_height` (1.5 m indoors) keeps ceilings and
  the floor above out; `lengths.radii.obstacle_clearance` is small (0.3 m) because walls are the
  map's borders already.

**Choosing a floor in RViz.** The goal tool takes the first surface under the mouse, i.e. the
topmost one. Hide the floors above with
`ros2 param set /maps_manager_node bigmap.output.view_max_z 3.0` (only the displayed map is cut; the
planner's and the saved ones are complete) and click on the floor you want; `1000.0` shows
everything. Or publish the goal by hand, with the floor's z:
`ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: map}, pose:
{position: {x: -1.8, y: -4.1, z: 2.35}, orientation: {w: 1.0}}}'`.

Cost: every re-mesh rebuilds the NavMap's adjacency and BVH, about 15 us per NavCel (a few hundred
ms for 10-15 thousand NavCels): keep `lengths.radii.full_res` small and `cell_size.min` at the
map's resolution (0.4 m for the playground's maps). Verified in the Gazebo parking garage: the
robot localizes with AMCL, plans between floors and climbs the ramp to the upper floor.
## Point clouds to a NavMap: `pointcloud_to_navmap` (tens of millions of points)

The NavMap that bigmap starts from is the finest one it will ever use: bigmap only coarsens it
(`cell_size.max`) and refines it again near the robot. `pointcloud_to_navmap` (package `navmap_tools`, in NavMap) makes it
from one or several `.pcd` files, however big:

```bash
ros2 run navmap_tools pointcloud_to_navmap --input cloud.pcd --output map.navmap --resolution 1.0 \
    --min-clearance 1.0 --recenter [--origin lat,lon,alt] [--crop x0,y0,x1,y1] [--zmax 2.5]
```

- **`--resolution`** is the finest NavCel: use `lengths.cell_size.min` of your parameter file. Finer is
  wasted with the grid engine (it rasterises at `cell_size.min`) and coarser throws detail away.
- **Big clouds.** ascii and binary files are streamed (a block of 1 M points at a time, never the whole
  file) and clouds above 5 M points are reduced by voxel first (`--voxel`, default resolution / 4:
  a voxel is replaced by the centroid of its points, so planes stay planes), so memory follows the
  *extent* of the map, not the number of points. `binary_compressed` cannot be streamed and is loaded
  whole by PCL. Several `--input` are joined.
- **Meshing** is `navmap_ros::from_points`: XY cells, one vertex per floor in a cell (several floors,
  `--min-clearance` = the free height that separates two of them; it must be BELOW the headroom of the
  lowest ramp: in the garage (floors 2.2 m apart, slabs 0.4 m) 2.0 cuts the ramp up to the top floor
  and that floor is left as a separate piece, 1.0-1.5 keeps all of them joined. The tool prints the
  heights where it found surface), surfaces never steeper than
  `--max-slope-deg` (walls stay as gaps), holes up to `--max-hole-area` interpolated.
- **Holes** (`--fill-holes`, 2 m): a SLAM cloud has no floor points where it did not see the
  floor, the mesher cannot invent them, and the ground the robot could drive over is cut into
  pieces. A stretch of empty cells up to that wide between two bits of the same floor, in a row
  or a column, gets floor points at the height in between and the cloud is meshed again, so the new
  triangles join the old ones (a straight border does not grow; 0 leaves the holes). On a SLAM map
  of two floors (346 000 points 0.2 m apart) this added 22 % of walkable area and made the largest
  connected piece 30 % bigger. What the cloud lacks entirely (a ramp nobody scanned, a room) stays
  missing: scan it again. `--max-hole-area` (1 m^2) is the mesher's own interpolation and does
  little by itself. `--min-surface-area` (m^2) drops the specks that are not connected to anything.
- **Resolution the data supports.** The tool prints "this data holds X m": below that, a cell has
  fewer than 3 points and holes and rough ground follow. A cloud with points 0.2 m apart holds
  0.37 m: use `--resolution 0.4`, not 0.3. The slope limit (`--max-slope-deg`) and `--max-dz` are
  not what cuts a SLAM floor: they changed nothing in the tests.
- **Colour** from the `rgb` field (or `r g b`): each vertex takes the colour of the nearest point of
  its own height, not of the floor below or above.
- **It tells you if the data holds the resolution**: the median points per cell and the finest
  resolution with at least 3 per cell (`WARNING` otherwise; a cloud that is already a grid of that
  resolution is fine with 1).
- **Output**: `map.navmap` and `map.yaml` (where the map's (0, 0, 0) is: `--origin` makes it
  georeferenced; `--recenter` shifts the map so that its centre is (0, 0) and writes the shift as
  `offset_x` / `offset_y`: bigmap rasterises around its origin, so a map in the first quadrant would
  waste memory). Keep the cloud in local coordinates: a PCD stores float32, which is 0.5 m of precision
  at UTM northings.

Measured with the 2024 PNOA-LiDAR of the campus (4 sheets, 32 M points with colour, 2 x 2 km, 1 m,
Release): 84 s and 5.9 GB peak, 4.2 M vertices / 7.2 M triangles (93 MB), and bigmap's hierarchy engine
loads it in 5 s (6 levels) and re-meshes in 0.12 s. A cloud with *everything* (canopies, cars, roofs)
gives a 1 m map with a lot of steep surface and gaps: that is what the data holds, so give it the
ground or a height window (`--zmax`) if you want drivable terrain only.

## A simulated robot on your own map

You have a NavMap and the point cloud (and its Bonxai map) from a SLAM with a real robot, and want
a simulated robot to drive in it. Gazebo needs a world of that building: BigMap does not make one
(a `map_to_world` tool existed and was removed, its walls were not good enough), so bring your own
(the mesh of the building, or a world modelled from the plan) in the same frame as the NavMap.

Then a parameter file for a simulation with AMCL: copy `config/params_parking_garage.yaml` and set
`bonxai.package` + `bonxai_path_file` (the Bonxai plugin needs the cloud inside a package's
`share/`: `maps/` of this package works), `terrain.package` + `initial_navmap`, and
`localizer_node.amcl.initial_pose.x / y` (where the robot starts). Launch the simulation with the world and the
height of the floor you start on, and the application with the file (the NavMap's height there, plus ~0.2 m):

```bash
ros2 launch easynav_playground_bigmap bigmap_sim.launch.py world_file:=<your world.sdf> \
    spawn_x:=<x> spawn_y:=<y> spawn_z:=0.1
ros2 launch bigmap_bringup bigmap.launch.py params_file:=<yours.yaml> sim:=true
```

**Where to start.** AMCL starts from x, y only and takes the topmost floor under that point, so
where two floors are stacked Gazebo (the floor `spawn_z` says) and AMCL (the upper one) can
disagree on the level. Start where there is one floor. `config/params_two_floors.yaml` starts at (-6.8, -7.8), a single floor at
z = -0.12 with walls all around (3.7 m to the nearest, so the lidar has something to match):
there Gazebo and AMCL agree on the level and the pose is within ~0.5 m.

## Large warehouse (`large_warehouse_world`) and point clouds to NavMap

```bash
git clone https://github.com/juanscelyg/large_warehouse_world.git src/large_warehouse_world
colcon build --packages-select large_warehouse_world bigmap_bringup   # (Rolling shell, see Build)
ros2 launch easynav_playground_bigmap easynav_bigmap_scenario.launch.yaml scenario:=large_warehouse
ros2 launch bigmap_bringup bigmap.launch.py config:=large_warehouse sim:=true
# elsewhere in the warehouse: change localizer_node.amcl.initial_pose in the file (the robot is
# spawned there and AMCL starts there; the package suggests 100 300, 300 230, 587 140, 300 405,
# 100 190, 300 50; the robot spawns at z = 0.5)
```

The package ships the NavMap (`maps/large_warehouse.navmap`, with its `.yaml` and the `.pcd` the
Bonxai map is built from). To make another one from a cloud,
`ros2 run navmap_tools pointcloud_to_navmap --input cloud.pcd --output map.navmap --resolution 1.0`, put it
in `maps/` (or anywhere, with `terrain.package: ""` and an absolute path) and point
`terrain.initial_navmap` of a copy of the file to it (`params_file:=<the copy>`). Finer resolutions
grow fast (0.5 m: 2.6 M NavCels). AMCL localizes on the same cloud (`bonxai`); the world's origin is
the south-west corner of the plan, so map and world frames coincide. Differences with
`params_indoor.yaml`: the **grid engine** (`multilevel.enabled: false`: one floor), `cell_size.min`
1.0 / `max` 32 (the map's resolution), `full_res` 15 m, `low_res_start` 60 m, `traversed_keep` 40 m,
`remesh_step` 5 m, `keep_narrow_width` 0 (the aisles between racks are all narrow),
`dynamic_map.enabled` false (the memory of obstacles still analyses the sensors), and the
simulation's AMCL, Bonxai map and `use_sim_time: true`. Loading takes ~1 s and a
re-mesh ~1.4 s (12 k NavCels) on this machine. The world is heavy (5 400 visuals, 800 collisions) and needs the GPU:
The world is heavy and needs the GPU; how the simulation copes (`gpu:=`, `lite:=`, the slow spawner) is in the
README of `easynav_playground_bigmap`, "A large warehouse and its cost".

**AMCL cost.** Each correction casts one ray per point and particle through the Bonxai map (DDA,
one step per voxel): about particles x points x range / voxel. With the defaults (100 particles,
1500 points, 0.3 m voxels) the EasyNav thread that runs AMCL, the maps manager and the planner sat
at 100 % of a core in the garage, the maps manager only kept 1-3 Hz of its 10 Hz and an AMCL update
went over 100 ms in the timing table. Tuned in `localizer_node.amcl` of the parameter file:

| | particles | points | cloud voxel | Bonxai voxel | freq |
|---|---|---|---|---|---|
| garage (default AMCL: 100 / 1500 / 0.3 m / 0.3 m / 5 Hz) | 40 | 300 | 0.5 m | 0.3 m | 5 Hz |
| large warehouse (lidar sees 100 m) | 40 | 250 | 0.6 m | 0.6 m (`bonxai.resolution`) | 3 Hz |

In the garage that thread dropped to about 49 % and the maps manager kept 10 Hz (one measurement
each, same machine). If the pose drifts, raise `num_particles` / `correct_max_points` first; if it
is still slow, lower `freq`. The warehouse values are not measured (Gazebo at 1-35 % of real time
leaves EasyNav mostly idle): compare the `[localizer]` row of the timing table before and after.

Verified at that speed: AMCL localizes at the start (100, 300), A\* plans and the robot starts
driving to a goal 30 m away.

## Resolution: circles and the ground driven over

- **Small circle** (blue in RViz) = `lengths.radii.full_res`: every NavCel at full resolution
  (`cell_size.min`). Beyond it NavCels grow geometrically with the distance and reach
  `cell_size.max` at `lengths.radii.low_res_start`; the surface is simplified within a height
  tolerance `lod.base_error` + `lod.error_growth` per metre beyond the circle.
- **Ground driven over** (`traversed`, radii in `lengths`): the cells the robot has driven over
  (within `traversed_mark`) are known to be navigable and keep full resolution in a strip
  `traversed_full_res` wide (plus the crack-free slack of about 1.8 x the cell size), fading over
  `traversed_fade`. (Before this was fixed the strip was `traversed_full_res` divided by the fading
  factor, a few centimetres, whatever the value.) The map's `traversed` layer flags them.
  - `mode: circles`: between the small circle and a **large circle** (orange,
    `lengths.radii.traversed_keep`) only the ground driven over is at full resolution; beyond the
    large circle everything is low resolution again and what was driven over there is forgotten
    (also in the layer and in saved maps): it becomes fine again only if the robot drives over it
    again.
  - `mode: persistent`: outside the small circle only the ground driven over stays at full
    resolution, wherever it is, for good (the map grows with the path).
- The mesh is re-made when the robot moves `lengths.remesh_step`.

## How the NavMap is reduced (two engines, one rule)

Both engines build a hierarchy **once** (coarsening up to `cell_size.max`) and then, at every re-mesh
(first pose and every `remesh_step`), choose a cut through it: split near the robot and the driven-over
ground, merged farther away. Merge and split are the same hierarchy read in both directions, so going
back to a place gives the same map (the cut depends only on the robot position and the traversed
ground, not on history).

| | `grid` (default) | `hierarchy` (`multilevel.enabled: true`) |
|---|---|---|
| source | a height grid: the DEM, or any `initial_navmap` rasterised at `cell_size.min` | the NavMap's own triangles (any resolution, several floors) |
| merge | RTIN: a node is merged while the height error stays under `lod.base_error + lod.error_growth * (d - full_res)` and the size under the cap that grows from `cell_size.min` to `cell_size.max` between `full_res` and `low_res_start` | vertex clustering by cells of size `min * 2^k` whose heights span <= `cluster_height`; border vertices merge only with border vertices; narrow passages are pinned |
| obstacle | the NavCel's own plane steeper than `max_slope_deg` | the steepest source NavCel it stands for |

Measured on the campus (2 000 000 NavCels at 0.5 m, robot at the centre, `full_res` 20, `low_res_start` 80,
Release build; reference = the map itself, `test_real_maps` with `BIGMAP_COMPARE=1`). Measured on
2026-10-20, before two fixes: the hierarchy engine no longer joins vertices the map does not join
(gaps, ledges): on the garage map the same number of triangles (6 954 instead of 6 961 around the
robot) and fewer non-manifold edges (0 instead of 5; 3 instead of 8 with the robot 18 m away); and
the grid built from a NavMap no longer grows its borders by 2 nodes:

| | grid | hierarchy |
|---|---|---|
| triangles | 16 255 | 24 189 |
| build / extract | 0.1 s / 1.3 ms | 1.1 s / 22 ms |
| obstacles kept (0-20 / 20-50 / 50-100 / >100 m) | 88 / 79 / 36 / 35 % | 77 / 79 / 55 / 51 % |
| false obstacles (same bands) | 0.4 / 2.4 / 1.4 / 6.8 % | 0.8 / 1.2 / 8.3 / 11.6 % |
| height error p95 (>100 m) | 3.3 m | 2.5 m |

So the grid engine is smaller and faster; the hierarchy engine is the one for maps that are not a height
field (floors over each other). Far away (>50 m) neither keeps most thin obstacles: the planner needs them
only near the robot.

**Build in Release.** A re-mesh costs ~50 ms in Release and ~600 ms without optimisation (colcon's
default): the NavMap geometry rebuild alone goes from 468 ms to 7 ms for 21 000 NavCels, and EasyNav's
loop falls to 1-3 Hz. Always pass `-DCMAKE_BUILD_TYPE=Release` (see Build).

## Dynamic map and obstacles

What the sensors see more than `dynamic_map.min_height` above a NavCel, measured along that
NavCel's normal, raises it to the height measured (relative to the robot and referred to the
terrain under it, not through the map -> robot TF, whose z drifts). The robot's roll/pitch/yaw are
applied to the points and the distance is to the plane of the NavCel, so a ramp the robot can climb
is not an obstacle. More rules keep the ground from being taken for obstacles: the points of a cell
must span `min_vertical_extent` in height, the detection must persist for `confirm_time`, and the
`ground_fit` window fits a plane to the lowest points ahead (the ramp being climbed). Only sensor
points inside `lengths.radii.sensor_range` (0 = the small circle) and below `max_height` are used.
Raised NavCels are in `/maps_manager_node/bigmap/dynamic_map` (NavState `map.dynamic`, layer
`obstacle_height`); a detection is forgotten after `forget_time`. The static map is not touched.

`obstacles.detector: bigmap_dynamic` (default): the NavCels with a detection are the obstacles.
`navmap_filter` uses the ObstacleFilter plugin instead, which measures through the map -> robot TF
and so needs a good z (list it in `filters`, see the table below). With `obstacles.removal.enabled`
those NavCels and all whose centre is within `lengths.radii.obstacle_clearance` are REMOVED from the
published NavMap (holes in RViz; they close when the obstacle goes away), stay LETHAL in the NavState
map, and `inflation.inflation_radius` spreads the cost around them in `inflated_obstacles`, which A\*
reads. The static layer `occupancy` marks the terrain steeper than `terrain.max_slope_deg`.
`obstacles.tint` paints the `obstacles` NavCels red over the imagery. The sensor obstacles are
put in the NavState map *before* the filters run in each cycle, so the inflation (what A\* reads) is
built on this cycle's obstacles, also right after a re-mesh. The clearance ring is found through a
grid of NavCel centroids (no limit on the number of obstacles).

**Which option controls what (obstacles and filters).** The obstacle options are applied in this
order, and at start the node checks that they agree and logs what is on (an impossible combination
stops it with a message; a doubtful one is a warning):

| Step | Option | Owner | Needs |
|---|---|---|---|
| 1 who detects | `obstacles.detector`: `bigmap_dynamic` \| `navmap_filter` | bigmap | `navmap_filter`: an obstacle filter in `filters` |
| 2 which detections count | `obstacles.memory.enabled`, `min_time`, `max_hits` | bigmap | `bigmap_dynamic` (with `navmap_filter` it is off) |
| 3 into the planner's layer | `obstacles.removal.enabled`, `lengths.radii.obstacle_clearance` | bigmap | without it detections never reach `obstacles` |
| 4 filters | `filters: [...]`: a filter runs if and only if it is listed | easynav_navmap_maps_manager | each listed name needs `<name>.plugin` |
| 5 display | `obstacles.tint.*` | bigmap | nothing: it does not change what the planner sees |

`filters` is the only switch of its filters, and they are independent of each other: `inflation`
(layer `inflated_obstacles`) works with either detector, `obstacle_filter` (layer `obstacles`) is
only for `obstacles.detector: navmap_filter`. Name an obstacle filter `obstacle_filter`, never
`obstacles` (that is bigmap's own section: an entry called `obstacles` has no plugin and is skipped
with a warning). The four combinations:

| `filters` | `detector` | Result |
|---|---|---|
| `[inflation]` | `bigmap_dynamic` | default: bigmap detects, the cost is inflated |
| `[obstacle_filter, inflation]` | `navmap_filter` | the plugin detects (no memory), the cost is inflated |
| `[]` | `bigmap_dynamic` | bigmap detects, no inflation: A\* reads `obstacles` and keeps no distance |
| `[inflation]` | `navmap_filter` | **error at start**: nobody detects; the reverse mix is a warning |

The sensors are analysed by bigmap if `dynamic_map.enabled` or `obstacles.memory.enabled` (and the
detector is `bigmap_dynamic`); with both false there are no sensor obstacles, only the steep terrain.
`dynamic_map.enabled: false` therefore does not mean "no obstacles" while the memory is on.

**Memory of obstacles (`obstacles.memory`, layer `obstacle_hits`).** A uint8 layer of the NavMap
with, per NavCel, how many times an obstacle has been detected there. An obstacle is a detection
that stays over the same NavCel for `min_time` seconds (5 by default; a gap over 1 s starts the count
again), or ground steeper than `max_slope_deg` (obstacle at once, shown with `max_hits`). The first
`min_time` seconds are one hit, and every further `min_time` seconds the detection goes on, or
comes back later, is another. Only detections inside `lengths.radii.sensor_range` (the small circle)
count. With fewer than `max_hits` the NavCel blocks only while the obstacle is detected; at
`max_hits` it is FIXED: it stays blocked for good (removed from the published map, LETHAL for the
planner, inflated) whether it is seen again or not. A spot gets at most one hit per `min_time`
(its detection cells report at different moments), so fixing it takes about `max_hits x min_time`
seconds of detection. A fixed obstacle blocks only NavCels of its own scale (legs up to 4 x
`cell_size.min`): far from the robot, where one NavCel can be tens of metres, it waits until the
mesh around it is fine, instead of closing a whole corridor to plans made from afar. The counts live on a grid of their own (cells of
`lengths.cell_size.min`, a band per metre of height), so they survive the re-meshes and are read back
onto the NavCels of every new mesh. With the memory on, the obstacle effect needs those `min_time`
seconds, not `dynamic_map.confirm_time`; `dynamic_map.enabled` may stay false (the sensors are then
processed only for the memory). `obstacles.memory.enabled` can be changed while running
(`ros2 param set /maps_manager_node bigmap.obstacles.memory.enabled false`), for a recovery that
wants to deal with obstacles itself: off, the fixed obstacles are released in the same cycle, the
layer shows only the steep ground, and detections block as without memory (`dynamic_map.enabled`:
after `confirm_time`, gone after `forget_time`; with it false, nothing is analysed). The counts are
kept and come back when it is switched on again. To forget them (fixed obstacles included), call
`ros2 service call /maps_manager_node/bigmap/clear_obstacles std_srvs/srv/Trigger`: update() empties
the memory in its next cycle, the layer goes back to the steep ground only and the obstacles are
released at once (counting starts again from zero). Needs `obstacles.detector: bigmap_dynamic`. The layer is also in the published map and in saved
maps, and a saved map used as `initial_navmap` brings its counts back into the memory (steep ground
excepted: it is steep anyway).

## Start from an existing NavMap, save, profile

```bash
ros2 launch easynav_playground_geo easynav_geo.launch.yaml                       # a world to drive in
ros2 launch bigmap_bringup bigmap.launch.py sim:=true \
    initial_navmap:=~/.ros/bigmap/saved/map_all_20261019_120000.navmap
```

- Any NavMap works, whatever the size of its NavCels. With the grid engine it is resampled onto a grid
  of `terrain.initial_navmap_resolution` m (0 = `cell_size.min`): a node is ground if a NavCel covers
  it; a missing node inside the surface (sampling) is filled, but the borders do not grow and a gap of
  2 nodes or more stays a gap; NaN heights are no data. In simulation it needs a Gazebo world
  (`terrain:=excavation` or `world_file:=...` of the playground, and `spawn_z:=...`); with `multilevel` its own
  triangles are used. Ground outside the map is no data.
- A `traversed` layer, if present, starts that ground as known; otherwise it is built during the run.
- `<map>.yaml` next to the file (written by the save services) holds the geolocation of the map's
  (0, 0, 0). Without it the map has no geolocation and its origin is 0, 0, 0.
- Not restored from the file: the `obstacles` / `occupancy` layers (recomputed from the slope; the `obstacle_hits` counts are), and,
  outdoors, surfaces a height grid cannot hold (bridges, overhangs).

Save the map as it is now (current resolutions; to `output.save_directory`):

```bash
ros2 service call /maps_manager_node/bigmap/save_map std_srvs/srv/Trigger          # all of it
ros2 service call /maps_manager_node/bigmap/save_traversed std_srvs/srv/Trigger    # only what was driven over
```

`output.profile.enabled: true` logs every `period` s the time spent in each section of the maps
manager (mean / worst ms per call and ms per second): the first rows are what costs most.
The plugin publishes `/maps_manager_node/bigmap/map` (NavMap, layers `occupancy`, `obstacles`,
`inflated_obstacles`, `obstacle_hits`, `elevation`, `lod_size`, `traversed`, plus vertex colours),
`.../map_updates`, `.../dynamic_map` and `.../markers`, and sets `map.navmap` in the NavState.

## Parameters

Complete configurations with the same sections, names and order (what only makes sense in one
is marked `[OUTDOOR ONLY]` / `[INDOOR ONLY]`): `config/params_outdoor.yaml`,
`config/params_indoor.yaml` (real robot), `config/params_parking_garage.yaml`, `params_two_floors.yaml` and
`config/params_large_warehouse.yaml` (simulations: they add the Bonxai map, AMCL and the NavMap
path). The header of each file has the table of lengths. The maps manager
(`maps_manager_node.bigmap`) groups its parameters by function:

| Group | What it holds |
|---|---|
| `terrain` | where the map comes from (`dem_file`, `initial_navmap`...), `max_slope_deg` |
| `multilevel` | indoor mode: floors over each other, ramps |
| `lengths` | **every radius, cell size and distance**: `radii.*` (small to large), `cell_size.{min,max}`, `traversed_fade`, `remesh_step` |
| `lod` | height tolerance of the simplification (`base_error`, `error_growth`), `hole_error` (the border of the map's holes counts as that error), `observed.enabled` (obstacle borders seen from near stay fine), `async_remesh` (re-mesh in a worker thread) |
| `recovery` | with a goal and no path for `no_path_cycles` cycles, `full_res` is doubled up to `max_full_res_factor` times, until the goal is reached or dropped |
| `traversed` | `enabled`, `mode` (circles / persistent) of the driven-over ground |
| `obstacles` | `detector`, `removal.enabled`, `memory.*`, `tint.*` |
| `dynamic_map` | what the sensors add, `ground_fit` (its sizes stay there: heights, cell, voxel) |
| `filters`, `inflation`, `obstacle_filter` | NavMap filters of easynav_navmap_maps_manager (not bigmap's own: their parameters are the plugin's) |
| `output` | `publish_markers`, `view_max_z`, `save_directory`, `profile.*` |

Radii, in order: `traversed_mark < full_res <= low_res_start`, `sensor_range <= full_res <=
traversed_keep`; `obstacle_clearance`, `traversed_full_res` and `observed_full_res` are independent small lengths;
`inflation.inflation_radius >= system_node.robot_geometry.inscribed_radius`. Invalid values stop the
node at start with a message naming the parameters: `low_res_start < full_res`, `traversed_keep <
full_res` in `circles` mode, `cell_size.min <= 0`, `remesh_step <= 0`, `ground_fit.min_cells < 3`,
`profile.period <= 0`, `memory.min_time <= 0`, `memory.max_hits` outside 1..255, and the obstacle
combinations of the table above. At start the log lists the radii and the obstacle pipeline in use.

The names used before are still accepted (the log warns which ones): `lod_settings.high_res_radius`
= `lengths.radii.full_res`, `lod_settings.low_res_distance` = `lengths.radii.low_res_start`,
`lod_settings.min/max_triangle_size` = `lengths.cell_size.min/max`, `lod_settings.hysteresis` =
`lengths.remesh_step`, `lod_settings.base_error/error_growth` = `lod.*`, `traversed_ground.enabled/mode`
= `traversed.*`, `traversed_ground.mark_radius/keep_radius/radius` = `lengths.radii.traversed_mark/
traversed_keep/traversed_full_res`, `traversed_ground.falloff_distance` = `lengths.traversed_fade`,
`dynamic_map.max_range` = `lengths.radii.sensor_range`, `obstacle_source` and `obstacles.source` (`dynamic`/`filter`) = `obstacles.detector` (`bigmap_dynamic`/`navmap_filter`),
`obstacle_removal.enabled/radius` = `obstacles.removal.enabled` / `lengths.radii.obstacle_clearance`,
`obstacle_tint.*` = `obstacles.tint.*`, `dem_file`, `dem_wait_timeout`, `initial_navmap*`,
`max_slope_deg` = `terrain.*`, `multilevel.keep_narrow_width` = `terrain.keep_narrow_width`,
`publish_markers`, `view_max_z`, `save.directory`, `profile.*` = `output.*`. The generator's
parameters (`dem_generator`) keep their names.

## Tests and diagnosis

```bash
colcon test --packages-select easynav_bigmap_maps_manager bigmap_bringup && colcon test-result --verbose
```

- `easynav_bigmap_maps_manager`: unit tests of the engines (`test_lod_mesher`: crack-free meshes, also on
  DEMs that are not 2^k+1 with driven-over ground, narrow passages, the full-resolution strip;
  `test_surface_lod`: floors, ramps, gaps and ledges never bridged; `test_navmap_dem`; the obstacle
  memory; the pipeline check), `test_bigmap_plugin` (the plugin as EasyNav runs it: parameters,
  `initialize()`, `update()` with a pose and a box of points: obstacle after `min_time`, fixed,
  released when the memory is switched off, inflation of the same cycle after a re-mesh, NaN pose,
  invalid values), and the ament linters (cpplint, uncrustify, cppcheck...). `test_real_maps` runs
  only with `BIGMAP_TEST_NAVMAP=<file>` (and `BIGMAP_COMPARE=1` for the engine comparison).
- `bigmap_bringup`: `test/test_config.py` checks the `params_*.yaml` against each other and
  against the code: no key that no code reads (typos, also in `dem_generator`), every key
  the plugin reads is in every file, the same structure in all, lengths in order, filters and
  obstacle options consistent (the table above), plugin ids that exist, the map files they point to,
  `use_sim_time` per use case, the AMCL start of the simulations, goal tolerances equal in system and
  controller, `terrain.dem_file` = the generator's `cache_file`. `test/test_dem.py`: the `dem_generator`
  block into a `navmap_gis_tool` command, a place into overrides, what a simulation leaves for the
  application, a world of `easynav_playground_geo`. `test/test_launch.py` calls the launch's `setup()` with
  arguments, without starting anything: which file and map each use case gets, following either
  simulation (the last one launched), and every refused combination with its message.
  `test/test_geo_launch.py`: `bigmap_geo.launch.py` includes the playground first and then the
  application, with the playground's arguments and defaults. `test/test_config.py` looks for the
  source of `easynav_plugins` upwards from this package (or `EASYNAV_PLUGINS_SRC`) and skips the checks
  against the plugin when it is not there.
- The simulation's tests are in `easynav_playground_geo` (`easynav_playgrounds`).
- At start the plugin logs the obstacle pipeline (`obstacles: detector ..., memory ..., filters
  [...]; A* reads '...'`), the lengths, the engine and, with `output.profile.enabled`, a timing table.

## Known limitations

- Hierarchy engine: a merge that would fold the surface is undone while the levels are built, but a
  few coarse triangles still face down or share an edge with more than two others, mostly far from
  the robot (garage map, 6 meshes: 39 triangles facing down of ~40 000 before the check, 33 after;
  `test_garage_lod`). The planner treats such edges as open. The grid engine has none.
- Grid engine, DEM not 2^k+1 nodes per side: coarse triangles at the high-side border can reach into
  the padding (heights replicated from the edge) by up to `cell_size.max`.
- Cost that grows with the map, not the robot's surroundings: the grid built from a NavMap is centred
  on (0, 0) (keep maps centred: `pointcloud_to_navmap --recenter`), the narrow-passage search is linear in
  the nodes but with a window of `keep_narrow_width` (seconds for 2000 x 2000 nodes, only with
  `keep_narrow_width > 0`), and publishing the map tests `traversed` per NavCel.
- The memory of obstacles only grows (cells where obstacles were seen) until
  `clear_obstacles` is called; no slow forgetting of unconfirmed hits.
- In a simulation EasyNav starts when the robot's odometry transform (`odom -> base_footprint`)
  appears (`tools/wait_for_robot.py`), or after 120 s with a warning if it never does.

## Author and license

Copyright 2026 José Miguel Guerrero (<josemiguel.guerrero@urjc.es>), author and maintainer.
Licensed under the Apache License, Version 2.0 (see `LICENSE` in the repository root).
