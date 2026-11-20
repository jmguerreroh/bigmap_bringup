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

"""How the DEM of a place is made (bigmap_bringup/dem.py): the parameter file's `dem_generator` block
into a navmap_gis_tool command, the place of a launch argument into overrides, and what the simulation
leaves for the application to follow. Nothing is downloaded or started."""

import importlib.util
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PARAMS = os.path.join(ROOT, 'config', 'params_outdoor.yaml')


def module():
    spec = importlib.util.spec_from_file_location(
        'bigmap_dem_under_test', os.path.join(ROOT, 'bigmap_bringup', 'dem.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def command(*overrides, params=PARAMS):
    m = module()
    return m.dem_generator_command(m.dem_parameters(params, *overrides), tool='TOOL')


def option(cmd, name):
    return cmd[cmd.index(name) + 1]


def test_the_outdoor_file_becomes_a_navmap_gis_tool_command():
    cmd = command()
    assert cmd[0] == 'TOOL'
    assert option(cmd, '--center') == '40.284796,-3.821407'
    assert float(option(cmd, '--size')) == 1000.0   # twice download_radius
    assert option(cmd, '--resolution') == '1.0'
    assert option(cmd, '--dem-source') == 'ign'
    assert option(cmd, '--dem-out').endswith('.ros/bigmap/elevation_cache.bin')
    assert '--dem-imagery' in cmd and option(cmd, '--imagery-source') == 'pnoa'
    assert option(cmd, '--zoom') == '18'
    # a 512 px tile at zoom 18: half of the 256 px pixel (0.455 m at this latitude)
    assert float(option(cmd, '--imagery-resolution')) == pytest.approx(0.2278, abs=1e-3)
    assert '--terrain-out' not in cmd   # only when the launch asks for the Gazebo terrain


def test_the_gazebo_terrain_options_come_with_gazebo_export():
    cmd = command({'gazebo.export': True, 'gazebo.world_template': '/t/w.in',
                   'gazebo.world_file': '/tmp/w.sdf'})
    assert option(cmd, '--world-template') == '/t/w.in'
    assert option(cmd, '--world-out') == '/tmp/w.sdf'
    assert option(cmd, '--terrain-resolution') == '4.0'
    assert option(cmd, '--terrain-out').endswith('.ros/bigmap/gazebo')


def test_a_mesh_source_passes_the_mesh_and_its_elevation():
    cmd = command({'elevation_source': 'mesh', 'mesh_file': '/m/site.stl',
                   'mesh_world_elevation': 700.0, 'gps_origin.latitude': 41.0,
                   'gps_origin.longitude': 2.0, 'download_radius': 120.0})
    assert option(cmd, '--dem-source') == 'mesh'
    assert option(cmd, '--mesh-file') == '/m/site.stl'
    assert option(cmd, '--mesh-world-elevation') == '700.0'
    assert option(cmd, '--center') == '41.0,2.0' and float(option(cmd, '--size')) == 240.0


def test_a_reference_altitude_is_passed_only_when_the_origin_is_not_at_sea_level():
    assert '--reference-altitude' not in command()                      # the file has 0.0
    cmd = command({'gps_origin.altitude': 648.5})
    assert option(cmd, '--reference-altitude') == '648.5'
    assert '--reference-altitude' not in command({'gps_origin.altitude': 0.0})


def test_synthetic_terrain_is_offline_for_heights_and_imagery():
    cmd = command({'use_synthetic_terrain': True})
    assert option(cmd, '--dem-source') == 'synthetic'
    assert option(cmd, '--imagery-source') == 'synthetic'


def test_xyz_imagery_needs_its_url_and_passes_it():
    cmd = command({'imagery.source': 'xyz', 'imagery.url_template': 'https://t/{z}/{x}/{y}.jpg'})
    assert option(cmd, '--imagery-source') == 'xyz'
    assert option(cmd, '--imagery-url') == 'https://t/{z}/{x}/{y}.jpg'
    with pytest.raises(RuntimeError, match='url_template'):
        command({'imagery.source': 'xyz'})


def test_no_imagery_means_no_colour_options():
    cmd = command({'imagery.enabled': False})
    assert '--dem-imagery' not in cmd and '--imagery-source' not in cmd


@pytest.mark.parametrize('overrides, message', [
    ({'elevation_source': 'terrain_tiles'}, 'now copernicus30'),
    ({'elevation_source': 'moon'}, 'must be ign'),
    ({'elevation_source': 'mesh'}, 'needs mesh_file'),
    ({'cache_file': ''}, 'cache_file'),
    ({'surprise': 1}, 'unknown parameters'),
])
def test_a_wrong_generator_setting_is_refused_with_a_message(overrides, message):
    with pytest.raises(RuntimeError, match=message):
        command(overrides)


def needs_sea_floor():
    if not module().has_sea_floor():
        pytest.skip('this navmap_tools has no sea floor (navmap_tools.geo.sea)')


def test_without_the_sea_floor_sea_off_and_auto_pass_and_on_is_refused(monkeypatch):
    m = module()
    monkeypatch.setattr(m, 'has_sea_floor', lambda: False)
    for sea in ('off', 'auto'):
        cmd = m.dem_generator_command(m.dem_parameters(PARAMS, {'sea': sea}), tool='TOOL')
        assert '--sea' not in cmd
    with pytest.raises(RuntimeError, match='no sea floor'):
        m.dem_generator_command(m.dem_parameters(PARAMS, {'sea': 'on'}), tool='TOOL')


def test_the_sea_options_go_to_the_generator():
    needs_sea_floor()
    cmd = command({'gazebo.export': True, 'gazebo.world_template': '/t/w.in',
                   'gazebo.world_file': '/tmp/w.sdf', 'gazebo.sea_model': '/m/my_model/model.sdf'})
    assert option(cmd, '--sea-model') == 'file:///m/my_model/model.sdf'
    assert '--sea' not in cmd                  # auto is the generator's default
    assert option(command({'sea': 'off'}), '--sea') == 'off'
    assert option(command({'sea': 'on'}), '--sea') == 'on'
    assert '--sea' not in command({'sea': 'auto'})
    with pytest.raises(RuntimeError, match='must be auto, on or off'):
        command({'sea': 'maybe'})


def test_the_sea_model_is_only_dropped_where_there_is_a_world():
    needs_sea_floor()
    assert '--sea-model' not in command({'gazebo.sea_model': '/m/my_model/model.sdf'})


def flat_dem(tmp_path, ground=650.0, reference=None, source=None, name='dem.bin'):
    pytest.importorskip('navmap_tools')
    from navmap_tools.geo.dem_file import build_local_dem, DemSource, write_dem_file

    class Flat:
        def sample_array(self, lon, lat):
            return ground + 0.0 * lon

    extra = {} if reference is None else {'reference_altitude': reference}
    dem = build_local_dem(Flat(), 40.3, -3.8, 10.0, 1.0, source or DemSource.SYNTHETIC, **extra)
    path = tmp_path / name
    write_dem_file(dem, path)
    return str(path), dem


def test_the_overrides_file_has_the_origin_of_the_dem(tmp_path):
    import yaml
    dem_file, dem = flat_dem(tmp_path)
    overrides = tmp_path / 'sub' / 'easynav_overrides.yaml'
    module().write_easynav_overrides(dem_file, str(overrides))
    ukf = yaml.safe_load(overrides.read_text())['localizer_node']['ros__parameters']['ukf']
    assert ukf == {'latitude_origin': 40.3, 'longitude_origin': -3.8, 'altitude_origin': 650.0}
    assert 'spawn_z' not in ''.join(os.listdir(tmp_path / 'sub'))     # that is the simulation's


def test_the_spawn_height_is_the_terrain_at_the_origin_plus_half_a_metre(tmp_path):
    dem_file, dem = flat_dem(tmp_path)
    assert module().spawn_height(dem_file) == pytest.approx(0.5)


@pytest.mark.parametrize('lat, lon, spain', [
    (40.4168, -3.7038, True),      # Madrid
    (28.1235, -15.4363, True),     # Las Palmas
    (39.5696, 2.6502, True),       # Palma
    (48.8562, 2.2976, False),      # Paris
    (46.0207, 7.7491, False),      # Zermatt
    (38.7223, -9.1393, False),     # Lisbon
])
def test_in_spain(lat, lon, spain):
    assert module().in_spain(lat, lon) is spain


def test_a_place_in_spain_keeps_the_parameter_files_sources_and_one_outside_changes_them():
    m = module()
    spain = m.place_overrides(40.4168, -3.7038, 300)
    assert spain == {'gps_origin.latitude': 40.4168, 'gps_origin.longitude': -3.7038,
                     'download_radius': 300.0, 'gps_origin.altitude': 0.0}
    cmd = command(spain)
    assert option(cmd, '--center') == '40.4168,-3.7038' and float(option(cmd, '--size')) == 600.0
    assert option(cmd, '--dem-source') == 'ign' and option(cmd, '--imagery-source') == 'pnoa'
    cmd = command(m.place_overrides(48.8562, 2.2976, 400))
    assert option(cmd, '--dem-source') == 'copernicus30'
    assert option(cmd, '--imagery-source') == 'esri' and float(option(cmd, '--size')) == 800.0
    # the sea and the air have no colour: no imagery to choose
    assert 'imagery.source' not in m.place_overrides(48.8562, 2.2976, 400, imagery=False)


def test_an_area_is_made_from_its_mesh_with_the_ground_of_its_centre():
    m = module()
    area = {'dir': '/a/area_1', 'ground_elevation': '651.5', 'latitude': '40.4168',
            'longitude': '-3.7038', 'radius': '400.0'}
    over = m.area_overrides(area)
    assert over['elevation_source'] == 'mesh' and over['gazebo.export'] is False
    assert over['mesh_file'] == '/a/area_1/navmap.stl' and over['mesh_world_elevation'] == 651.5
    cmd = command(over)
    assert option(cmd, '--mesh-file') == '/a/area_1/navmap.stl'
    assert option(cmd, '--center') == '40.4168,-3.7038'
    outside = m.area_overrides(dict(area, latitude='48.8562', longitude='2.2976'))
    assert outside['imagery.source'] == 'esri'


def test_an_area_with_holes_is_made_from_its_ground_with_its_buildings_as_holes(tmp_path):
    m = module()
    area = {'dir': str(tmp_path), 'ground_elevation': '651.5', 'latitude': '40.4168',
            'longitude': '-3.7038', 'radius': '400.0'}
    assert 'holes_mesh_file' not in m.area_overrides(area)
    assert '--dem-holes-mesh' not in command(m.area_overrides(area))
    with pytest.raises(RuntimeError, match='navmap_ground.stl'):
        m.area_overrides(area, holes=True)   # an area made before the ground and holes meshes
    (tmp_path / 'navmap_ground.stl').write_bytes(b'')
    (tmp_path / 'navmap_holes.stl').write_bytes(b'')
    over = m.area_overrides(area, holes=True)
    assert over['mesh_file'] == str(tmp_path / 'navmap_ground.stl')
    cmd = command(over)
    assert option(cmd, '--mesh-file') == str(tmp_path / 'navmap_ground.stl')
    assert option(cmd, '--dem-holes-mesh') == str(tmp_path / 'navmap_holes.stl')


def test_an_area_directory_is_named_as_area_world_names_it():
    m = module()
    assert m.area_dir_of(43.3848, -3.0098, 300).endswith('.ros/bigmap/areas/area_43.38480_-3.00980_300')
    assert m.area_dir_of('40.4168', '-3.7038', '400.0').endswith('area_40.41680_-3.70380_400')
    # the directory area_world.py makes: the same format string
    tool = os.path.join(os.path.dirname(os.path.dirname(ROOT)), 'easynav_playgrounds',
                        'playground_bigmap', 'easynav_playground_bigmap', 'tools', 'area_world.py')
    if not os.path.isfile(tool):
        pytest.skip('easynav_playground_bigmap source not found')
    source = open(tool).read()
    assert "'area_%.5f_%.5f_%d'" in source


def test_area_yaml_is_read_as_a_dict(tmp_path):
    (tmp_path / 'area.yaml').write_text('# written by area_world.py\nkey: abc\nradius: 400.0\n'
                                        'sea: true\n')
    area = module().read_area(str(tmp_path))
    assert area == {'dir': str(tmp_path), 'key': 'abc', 'radius': '400.0', 'sea': 'true'}


def test_what_the_simulation_made_its_world_from_is_what_the_application_reads(tmp_path):
    needs_sea_floor()
    m = module()
    path = str(tmp_path / 'sim' / 'sim.yaml')
    overrides = {'gps_origin.latitude': 43.38, 'gps_origin.longitude': -3.0, 'download_radius': 300.0,
                 'sea': 'on', 'gazebo.export': True, 'gazebo.world_file': '/w.sdf',
                 'cache_file': '/c/sim_dem.bin'}
    m.write_sim_info(overrides, {'robot': 'boat'}, path)
    info = m.read_sim_info(path)
    assert info['robot'] == 'boat'
    # the world and the simulation's own cache are not the application's business
    assert info['dem'] == {'gps_origin.latitude': 43.38, 'gps_origin.longitude': -3.0,
                           'download_radius': 300.0, 'sea': 'on'}
    # and they make the very same DEM command
    assert option(command(info['dem']), '--sea') == 'on'
    assert option(command(info['dem']), '--center') == '43.38,-3.0'


def test_there_is_no_simulation_to_follow_without_its_file(tmp_path):
    with pytest.raises(RuntimeError, match='no simulation to follow'):
        module().read_sim_info(str(tmp_path / 'nothing.yaml'))


@pytest.mark.parametrize('text, message', [
    ('- a\n- b\n', 'a mapping with a `dem` mapping'),
    ('hello\n', 'a mapping with a `dem` mapping'),
    ('dem: [1, 2]\nrobot: x\n', 'a mapping with a `dem` mapping'),
    ('dem:\n  surprise: 1\n', 'does not know'),
    ('dem: {\n', 'not the file a simulation writes'),
])
def test_a_sim_file_that_is_not_what_a_simulation_writes_is_refused_with_a_message(
        tmp_path, text, message):
    path = tmp_path / 'sim.yaml'
    path.write_text(text)
    with pytest.raises(RuntimeError, match=message):
        module().read_sim_info(str(path))


def test_the_sim_file_says_when_it_was_written(tmp_path):
    m = module()
    path = str(tmp_path / 'sim.yaml')
    m.write_sim_info({'gps_origin.latitude': 43.0, 'gps_origin.longitude': -3.0},
                     {'robot': 'boat'}, path)
    info = m.read_sim_info(path)
    assert info['robot'] == 'boat' and len(info['written']) == 16    # YYYY-MM-DD HH:MM
    (tmp_path / 'empty.yaml').write_text('')
    assert m.read_sim_info(str(tmp_path / 'empty.yaml')).get('dem', {}) == {}   # empty: nothing


def write_geo_world(tmp_path, text=None):
    d = tmp_path / 'terrain_46.02070_7.74910_300'
    d.mkdir()
    (d / 'navmap.stl').write_bytes(b'\0' * 84)
    last = tmp_path / 'last_world.yaml'
    last.write_text(text if text is not None else (
        'world: terrain\ndirectory: %s\nlatitude: 46.02070000\nlongitude: 7.74910000\n'
        'radius: 300.0\nground_elevation: 1605.437\nmesh: %s\n' % (d, d / 'navmap.stl')))
    return str(last), str(d)


def test_a_geo_world_is_followed_by_its_mesh_and_its_place(tmp_path):
    m = module()
    last, d = write_geo_world(tmp_path)
    info = m.geo_sim_info(m.read_geo_world(last), path=last)
    assert info['simulation'] == 'easynav_playground_geo' and info['terrain'] == 'terrain'
    assert info['use_sim_time'] is True and info['written']
    assert info['dem']['elevation_source'] == 'mesh'
    assert info['dem']['mesh_file'] == os.path.join(d, 'navmap.stl')
    assert info['dem']['mesh_world_elevation'] == 1605.437
    assert info['dem']['download_radius'] == 300.0
    # Zermatt is outside Spain: the colours of the map are Esri's
    assert info['dem']['imagery.source'] == 'esri'
    cmd = command(info['dem'])
    assert option(cmd, '--center') == '46.0207,7.7491' and option(cmd, '--dem-source') == 'mesh'


def test_a_last_world_that_is_not_the_playgrounds_is_refused_with_a_message(tmp_path):
    m = module()
    last, d = write_geo_world(tmp_path, text='world: terrain\nlatitude: 1\n')
    with pytest.raises(RuntimeError, match='not the file easynav_playground_geo writes'):
        m.read_geo_world(last)


def test_without_any_simulation_the_message_names_both_playgrounds(tmp_path, monkeypatch):
    m = module()
    monkeypatch.setattr(m, 'GEO_LAST_WORLD', str(tmp_path / 'a.yaml'))
    monkeypatch.setattr(m, 'SIM_INFO', str(tmp_path / 'b.yaml'))
    with pytest.raises(RuntimeError, match='easynav_playground_geo'):
        m.read_simulation()
