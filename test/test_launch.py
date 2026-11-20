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

"""The decisions of launch/bigmap.launch.py, the application (which parameter file, how the map is
made, which combinations are refused), taken by calling its setup() with launch arguments, without
starting any process. Needs the workspace built and sourced (ament index, bigmap_bringup.dem)."""

import importlib.util
import os

import pytest

launch = pytest.importorskip('launch')
pytest.importorskip('ament_index_python')
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from launch_ros.actions import Node  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
LAUNCH_FILE = os.path.join(ROOT, 'launch', 'bigmap.launch.py')


def module():
    pytest.importorskip('bigmap_bringup.dem')
    spec = importlib.util.spec_from_file_location('bigmap_launch', LAUNCH_FILE)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def context(**given):
    """A launch context with the declared defaults and `given` on top."""
    m = module()
    ctx = launch.LaunchContext()
    for action in m.generate_launch_description().entities:
        if isinstance(action, launch.actions.DeclareLaunchArgument):
            ctx.launch_configurations[action.name] = given.pop(action.name, None) or ''.join(
                s.perform(ctx) for s in action.default_value)
    assert not given, 'unknown launch arguments %s' % given
    return m, ctx


def nodes(actions):
    """Every launch_ros Node, also those inside event handlers and timers."""
    out = []
    for a in actions:
        if isinstance(a, Node):
            out.append(a)
        elif isinstance(a, launch.actions.TimerAction):
            out += nodes(a.actions)
        elif isinstance(a, launch.actions.RegisterEventHandler):   # started on an event
            out += nodes(vars(a.event_handler)['_OnActionEventBase__actions_on_event'])
    return out


def processes(actions):
    """The processes that are not nodes (the DEM generator, the waiting for the robot)."""
    return [a for a in actions if isinstance(a, launch.actions.ExecuteProcess)
            and not isinstance(a, Node)]


def after_generator(actions, ctx, returncode=0):
    """What the launch does when the DEM generator exits: the actions it then starts."""
    handler = [a for a in actions if isinstance(a, launch.actions.RegisterEventHandler)][0]
    on_event = vars(handler.event_handler)['_OnActionEventBase__on_event']
    event = type('ProcessExited', (), {'returncode': returncode})()
    return on_event(event, ctx)


def name_of(process):
    return ''.join(part.text for part in process.name)


def command_of(process):
    return [part[0].text if hasattr(part[0], 'text') else str(part[0]) for part in process.cmd]


def option(cmd, name):
    return cmd[cmd.index(name) + 1]


def setup(**given):
    m, ctx = context(rviz='false', **given)
    return m, ctx, m.setup(ctx)


def flat_dem(tmp_path):
    from navmap_tools.geo.dem_file import build_local_dem, DemSource, write_dem_file

    class Flat:
        def sample_array(self, lon, lat):
            return 650.0 + 0.0 * lon

    path = tmp_path / 'dem.bin'
    dem = build_local_dem(Flat(), 40.3, -3.8, 10.0, 1.0, DemSource.SYNTHETIC)
    write_dem_file(dem, path)
    return str(path), dem


def needs(pkg):
    try:
        get_package_share_directory(pkg)
    except Exception:
        pytest.skip('%s is not installed' % pkg)


def text(parts, ctx):
    return parts if isinstance(parts, str) else ''.join(
        s if isinstance(s, str) else s.perform(ctx) for s in parts)


def params_of(node, ctx):
    """The parameters of a node: the parameter files (paths) and the dicts ({name: value})."""
    out = []
    for p in node._Node__parameters:
        if hasattr(p, 'param_file'):
            out.append(text(p.param_file, ctx))
        else:
            out.append({text(k, ctx): v for k, v in p.items()})
    return out


def arguments_of(node, ctx):
    return [text(a, ctx) for a in node._Node__arguments]


@pytest.fixture(autouse=True)
def no_geo_world(monkeypatch, tmp_path):
    """No test follows the user's own last world of easynav_playground_geo: only the ones a test
    writes."""
    try:
        import bigmap_bringup.dem as dem
    except ImportError:
        return
    monkeypatch.setattr(dem, 'GEO_LAST_WORLD', str(tmp_path / 'no_geo' / 'last_world.yaml'))


def geo_world(tmp_path, holes_files=False):
    """A world of easynav_playground_geo as it leaves it: its directory and last_world.yaml."""
    d = tmp_path / 'area_40.41680_-3.70380_250'
    d.mkdir()
    for name in ['navmap.stl'] + (['navmap_ground.stl', 'navmap_holes.stl'] if holes_files else []):
        (d / name).write_bytes(b'\0' * 84)
    last = tmp_path / 'last_world.yaml'
    last.write_text('# written by easynav_playground_geo_worlds\nworld: area\ndirectory: %s\n'
                    'latitude: 40.41680000\nlongitude: -3.70380000\nradius: 250.0\n'
                    'ground_elevation: 648.015\nmesh: %s\n' % (d, d / 'navmap.stl'))
    return str(last), str(d)


def generator_wrote_its_dem(monkeypatch):
    """after_generator reads the DEM the generator wrote (the UKF origin): no test has one, so the
    overrides file is not written (a test must not depend on the user's DEM cache)."""
    import bigmap_bringup.dem as dem
    written = []
    monkeypatch.setattr(dem, 'write_easynav_overrides', lambda *a: written.append(a))
    return written


def test_the_use_cases_are_the_parameter_files_of_config():
    m = module()
    names = m.configs(os.path.dirname(HERE) and ROOT)
    assert set(names) == {'outdoor', 'indoor', 'parking_garage', 'large_warehouse', 'two_floors'}
    for name in names:
        assert os.path.isfile(os.path.join(ROOT, 'config', 'params_%s.yaml' % name))


def test_it_knows_nothing_of_the_simulator():
    source = open(LAUNCH_FILE).read()
    for word in ('ros_gz', 'gz_gui', 'world.launch', 'summit.launch', 'spawn_', 'robot_state_publisher'):
        assert word not in source, word


def test_outdoors_the_dem_is_made_first_and_easynav_starts_after_it(monkeypatch):
    generator_wrote_its_dem(monkeypatch)
    needs('navmap_tools')
    m, ctx, actions = setup()
    generators = processes(actions)
    assert [name_of(g) for g in generators] == ['dem_generator']
    cmd = command_of(generators[0])
    assert os.path.basename(cmd[0]) == 'navmap_gis_tool'
    assert option(cmd, '--dem-out').endswith('elevation_cache.bin')
    assert '--world-template' not in cmd and '--terrain-out' not in cmd    # no world: not its business
    assert not [a for a in actions if isinstance(a, Node)], \
        'EasyNav starts after the generator exits, not before'
    started = after_generator(actions, ctx)
    assert sorted(n.node_executable for n in nodes(started)) == ['rviz2', 'system_main']


def test_a_place_outside_spain_uses_copernicus_and_esri_and_one_in_spain_keeps_ign():
    needs('navmap_tools')
    m, ctx, actions = setup(lat='48.8562', lon='2.2976', radius='400')
    cmd = command_of(processes(actions)[0])
    assert option(cmd, '--center') == '48.8562,2.2976' and float(option(cmd, '--size')) == 800.0
    assert option(cmd, '--dem-source') == 'copernicus30' and option(cmd, '--imagery-source') == 'esri'
    m, ctx, actions = setup(lat='40.4168', lon='-3.7038', radius='300')
    cmd = command_of(processes(actions)[0])
    assert option(cmd, '--dem-source') == 'ign' and option(cmd, '--imagery-source') == 'pnoa'


def test_without_a_place_the_parameter_files_is_kept():
    needs('navmap_tools')
    m, ctx, actions = setup()
    cmd = command_of(processes(actions)[0])
    assert option(cmd, '--center') == '40.284796,-3.821407' and float(option(cmd, '--size')) == 1000.0
    m, ctx, actions = setup(radius='200')
    assert float(option(command_of(processes(actions)[0]), '--size')) == 400.0


def test_following_a_simulation_takes_its_map_from_what_it_wrote(monkeypatch):
    needs('navmap_tools')
    import bigmap_bringup.dem as dem
    info = {'robot': 'summit', 'terrain': 'area', 'environment': 'land',
            'dem': {'gps_origin.latitude': 40.4168, 'gps_origin.longitude': -3.7038,
                    'download_radius': 300.0, 'elevation_source': 'mesh',
                    'mesh_file': '/a/area/navmap.stl', 'mesh_world_elevation': 651.5}}
    monkeypatch.setattr(dem, 'read_sim_info', lambda *a: info)
    m, ctx, actions = setup(config='outdoor', sim='true')
    cmd = command_of(processes(actions)[0])
    assert option(cmd, '--center') == '40.4168,-3.7038' and float(option(cmd, '--size')) == 600.0
    assert option(cmd, '--dem-source') == 'mesh' and option(cmd, '--mesh-file') == '/a/area/navmap.stl'
    assert '--world-template' not in cmd               # the world is the simulation's business


def test_following_a_simulation_that_is_not_there_says_so(monkeypatch, tmp_path):
    needs('navmap_tools')
    import bigmap_bringup.dem as dem

    def missing(*a):
        raise RuntimeError('no simulation to follow')
    monkeypatch.setattr(dem, 'read_sim_info', missing)
    with pytest.raises(RuntimeError, match='no simulation to follow'):
        setup(sim='true')


def test_with_sim_true_easynav_waits_for_the_robot_and_runs_on_sim_time():
    needs('navmap_tools')
    for given in (dict(config='parking_garage', sim='true'),):
        m, ctx, actions = setup(**given)
        waiters = [a for a in processes(actions) if name_of(a) == 'wait_for_robot']
        assert len(waiters) == 1
        assert not any(isinstance(a, launch.actions.TimerAction) for a in actions)
        easynav = [n for n in nodes(actions) if n.node_executable == 'system_main']
        assert len(easynav) == 1
        assert {'use_sim_time': True} in params_of(easynav[0], ctx)


def test_a_real_robot_does_not_wait_and_runs_on_the_wall_clock_whatever_the_file_says(monkeypatch):
    """params_outdoor.yaml has use_sim_time true (it serves the simulation too): on a real robot
    (sim:=false) EasyNav must not wait for a /clock that never comes (seen: the UKF stuck in
    "Waiting for clock to start...")."""
    generator_wrote_its_dem(monkeypatch)
    m, ctx, actions = setup(config='parking_garage')
    assert not [a for a in processes(actions) if name_of(a) == 'wait_for_robot']
    easynav = [n for n in nodes(actions) if n.node_executable == 'system_main'][0]
    assert {'use_sim_time': False} in params_of(easynav, ctx)
    needs('navmap_tools')
    m, ctx, actions = setup(config='outdoor', lat='40.4168', lon='-3.7038', radius='300')
    easynav = [n for n in nodes(after_generator(actions, ctx)) if n.node_executable == 'system_main'][0]
    assert {'use_sim_time': False} in params_of(easynav, ctx)
    m, ctx, actions = setup(config='parking_garage', use_sim_time='true')
    easynav = [n for n in nodes(actions) if n.node_executable == 'system_main'][0]
    assert {'use_sim_time': True} in params_of(easynav, ctx)


def test_an_indoor_use_case_runs_easynav_with_its_file_only():
    m, ctx, actions = setup(config='parking_garage')
    easynav = [n for n in nodes(actions) if n.node_executable == 'system_main']
    assert len(easynav) == 1
    files = [p for p in params_of(easynav[0], ctx) if isinstance(p, str)]
    assert len(files) == 1, 'indoors nothing but the parameter file (no overrides file)'
    assert files[0].endswith('params_parking_garage.yaml')
    assert not processes(actions), 'nothing is generated'


def test_a_gps_file_gets_the_overrides_file_after_the_generator(tmp_path):
    needs('navmap_tools')
    m, ctx, actions = setup()
    # a generator that failed: shutdown, no EasyNav
    failed = after_generator(actions, ctx, returncode=1)
    assert any(isinstance(r, launch.actions.EmitEvent) for r in failed)
    assert not nodes(failed)
    # one that made a DEM: the overrides file with its origin, which EasyNav reads last
    dem_file, _ = flat_dem(tmp_path)
    m.OVERRIDES_FILE = str(tmp_path / 'overrides.yaml')
    m.bigmap_dem.dem_parameters  # (the launch reads the DEM where the file says: fake it)
    original = m.bigmap_dem.write_easynav_overrides
    seen = []
    m.bigmap_dem.write_easynav_overrides = lambda dem, out: seen.append(out)
    try:
        started = after_generator(actions, ctx)
    finally:
        m.bigmap_dem.write_easynav_overrides = original
    assert seen == [str(tmp_path / 'overrides.yaml')]
    easynav = [n for n in nodes(started) if n.node_executable == 'system_main']
    files = [p for p in params_of(easynav[0], ctx) if isinstance(p, str)]
    assert files[-1] == str(tmp_path / 'overrides.yaml')      # last: it wins over the file


@pytest.mark.parametrize('given, message', [
    (dict(config='nowhere'), 'config must be one of'),
    (dict(config='indoor'), 'terrain.initial_navmap'),         # params_indoor.yaml is a template to fill
    (dict(config='parking_garage', lat='40.0', lon='-3.0'), 'dem_generator block'),
    (dict(config='parking_garage', initial_navmap='/tmp/x.navmap'), 'is in the file'),
    (dict(lat='40.0'), 'go together'),
    (dict(lon='-3.0'), 'go together'),
    (dict(use_sim_time='maybe'), 'true or false'),
    (dict(initial_navmap='/tmp/x.navmap', lat='40.0', lon='-3.0'), 'does not go with lat'),
    (dict(initial_navmap='/tmp/does_not_exist.navmap'), 'no such file'),
])
def test_combinations_that_cannot_work_are_refused_with_a_message(given, message):
    with pytest.raises(RuntimeError, match=message):
        setup(**given)


def test_a_saved_navmap_goes_to_the_overrides_file_with_its_origin(tmp_path):
    import yaml
    navmap = tmp_path / 'site.navmap'
    navmap.write_bytes(b'x')
    (tmp_path / 'site.yaml').write_text('georeferenced: true\norigin_lat: 40.5\norigin_lon: -3.5\n'
                                        'reference_altitude: 650\n')
    m, ctx = context(rviz='false', initial_navmap=str(navmap))
    m.OVERRIDES_FILE = str(tmp_path / 'sub' / 'overrides.yaml')
    actions = m.setup(ctx)
    data = yaml.safe_load((tmp_path / 'sub' / 'overrides.yaml').read_text())
    assert data['maps_manager_node']['ros__parameters']['bigmap']['terrain']['initial_navmap'] == \
        str(navmap)
    ukf = data['localizer_node']['ros__parameters']['ukf']
    assert ukf == {'latitude_origin': 40.5, 'longitude_origin': -3.5, 'altitude_origin': 650.0}
    easynav = [n for n in nodes(actions) if n.node_executable == 'system_main'][0]
    files = [p for p in params_of(easynav, ctx) if isinstance(p, str)]
    assert files[-1] == str(tmp_path / 'sub' / 'overrides.yaml')
    assert not processes(actions), 'no DEM is made from a saved NavMap'


def test_a_saved_navmap_without_a_yaml_keeps_the_files_origin(tmp_path):
    import yaml
    navmap = tmp_path / 'site.navmap'
    navmap.write_bytes(b'x')
    m, ctx = context(rviz='false', initial_navmap=str(navmap))
    m.OVERRIDES_FILE = str(tmp_path / 'overrides.yaml')
    m.setup(ctx)
    assert 'localizer_node' not in yaml.safe_load((tmp_path / 'overrides.yaml').read_text())


def rviz_of(config, **given):
    m, ctx = context(config=config, **given)
    actions = m.setup(ctx)
    if m.has_dem(os.path.join(ROOT, 'config', 'params_%s.yaml' % config)):
        actions = after_generator(actions, ctx)      # RViz starts with EasyNav, after the DEM
    return [n for n in nodes(actions) if n.node_executable == 'rviz2'], ctx, actions


def test_rviz_shows_the_navmap(monkeypatch):
    import bigmap_bringup.dem as dem
    monkeypatch.setattr(dem, 'write_easynav_overrides', lambda *a: None)
    needs('navmap_tools')
    for config in ('outdoor', 'parking_garage'):
        rviz, ctx, actions = rviz_of(config)
        assert len(rviz) == 1, config
        assert arguments_of(rviz[0], ctx)[-1].endswith('bigmap.rviz'), config


def test_rviz_config_given_wins():
    rviz, ctx, actions = rviz_of('parking_garage', rviz_config='/tmp/my.rviz')
    assert arguments_of(rviz[0], ctx)[-1] == '/tmp/my.rviz'


def test_the_helpers_read_what_the_files_have():
    m = module()
    outdoor = os.path.join(ROOT, 'config', 'params_outdoor.yaml')
    garage = os.path.join(ROOT, 'config', 'params_parking_garage.yaml')
    assert m.has_dem(outdoor) and not m.has_dem(garage)
    assert m.uses_ukf(outdoor) and not m.uses_ukf(garage)


def test_a_file_with_a_dem_generator_block_but_no_cache_file_is_refused_with_its_message(tmp_path):
    needs('navmap_tools')
    params = tmp_path / 'p.yaml'
    params.write_text('dem_generator:\n  ros__parameters:\n    gps_origin:\n      latitude: 40.0\n'
                      '      longitude: -3.0\n')
    with pytest.raises(RuntimeError, match='cache_file'):
        setup(params_file=str(params))


def test_a_sim_file_that_is_not_a_simulations_gives_its_message_not_a_traceback(monkeypatch,
                                                                                tmp_path):
    needs('navmap_tools')
    import bigmap_bringup.dem as dem
    bad = tmp_path / 'sim.yaml'
    bad.write_text('- not\n- a mapping\n')
    real = dem.read_sim_info
    monkeypatch.setattr(dem, 'read_sim_info', lambda *a: real(str(bad)))
    with pytest.raises(RuntimeError, match='not the file a simulation writes'):
        setup(sim='true')


def test_a_saved_navmap_whose_yaml_has_no_numbers_is_refused_with_a_message(tmp_path):
    navmap = tmp_path / 'site.navmap'
    navmap.write_bytes(b'x')
    (tmp_path / 'site.yaml').write_text(
        'georeferenced: true\norigin_lat: north\norigin_lon: -3.5\n')
    m, ctx = context(rviz='false', initial_navmap=str(navmap))
    m.OVERRIDES_FILE = str(tmp_path / 'overrides.yaml')
    with pytest.raises(RuntimeError, match='must be numbers'):
        m.setup(ctx)


def test_following_a_geo_world_makes_the_map_of_its_mesh(monkeypatch, tmp_path):
    needs('navmap_tools')
    import bigmap_bringup.dem as dem
    last, d = geo_world(tmp_path, holes_files=True)
    monkeypatch.setattr(dem, 'GEO_LAST_WORLD', last)
    monkeypatch.setattr(dem, 'SIM_INFO', str(tmp_path / 'no_sim' / 'sim.yaml'))
    m, ctx, actions = setup(config='outdoor', sim='true')
    cmd = command_of(processes(actions)[0])
    assert option(cmd, '--center') == '40.4168,-3.7038' and float(option(cmd, '--size')) == 500.0
    assert option(cmd, '--dem-source') == 'mesh'
    assert option(cmd, '--mesh-file') == os.path.join(d, 'navmap.stl')
    assert float(option(cmd, '--mesh-world-elevation')) == 648.015
    assert '--dem-holes-mesh' not in cmd and '--world-template' not in cmd
    # its buildings, walls and trunks as holes of the ground
    m, ctx, actions = setup(config='outdoor', sim='true', area_holes='true')
    cmd = command_of(processes(actions)[0])
    assert option(cmd, '--mesh-file') == os.path.join(d, 'navmap_ground.stl')
    assert option(cmd, '--dem-holes-mesh') == os.path.join(d, 'navmap_holes.stl')


def test_the_last_simulation_launched_is_the_one_followed(monkeypatch, tmp_path):
    needs('navmap_tools')
    import bigmap_bringup.dem as dem
    last, d = geo_world(tmp_path)
    sim = tmp_path / 'sim.yaml'
    dem.write_sim_info({'gps_origin.latitude': 43.38, 'gps_origin.longitude': -3.0,
                        'download_radius': 300.0}, {'robot': 'summit'}, str(sim))
    monkeypatch.setattr(dem, 'GEO_LAST_WORLD', last)
    monkeypatch.setattr(dem, 'SIM_INFO', str(sim))
    os.utime(last, (1000, 1000))
    os.utime(sim, (2000, 2000))        # the other playground's, launched later
    cmd = command_of(processes(setup(config='outdoor', sim='true')[2])[0])
    assert option(cmd, '--center') == '43.38,-3.0'
    os.utime(last, (3000, 3000))       # then the geo one
    cmd = command_of(processes(setup(config='outdoor', sim='true')[2])[0])
    assert option(cmd, '--center') == '40.4168,-3.7038'


def test_a_geo_world_that_is_gone_says_so(monkeypatch, tmp_path):
    needs('navmap_tools')
    import bigmap_bringup.dem as dem
    last, d = geo_world(tmp_path)
    os.remove(os.path.join(d, 'navmap.stl'))
    monkeypatch.setattr(dem, 'GEO_LAST_WORLD', last)
    with pytest.raises(RuntimeError, match='launch the simulation again'):
        setup(config='outdoor', sim='true')


def test_area_holes_is_true_or_false():
    with pytest.raises(RuntimeError, match='area_holes'):
        setup(config='outdoor', sim='true', area_holes='yes')
