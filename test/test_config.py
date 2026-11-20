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

"""Checks of the parameter files (config/params_*.yaml) against each other and against the
code that reads them: every key is one the plugin knows (no typos), every key the plugin reads is
in every file (nothing is left to a default by accident), the options that depend on each other
agree, and the files they point to exist. Run: python3 -m pytest test/test_config.py"""

import glob
import os
import re

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CONFIGS = sorted(glob.glob(os.path.join(ROOT, 'config', 'params_*.yaml')))


def plugins_src():
    """The source of easynav_plugins, looked for upwards from this package (the workspace's
    src/easynav_plugins, whatever the depth this repository is cloned at), or EASYNAV_PLUGINS_SRC."""
    if os.environ.get('EASYNAV_PLUGINS_SRC'):
        return os.environ['EASYNAV_PLUGINS_SRC']
    d = ROOT
    for _ in range(4):
        d = os.path.dirname(d)
        for candidate in (os.path.join(d, 'easynav_plugins'), os.path.join(d, 'src', 'easynav_plugins')):
            if os.path.isdir(os.path.join(candidate, 'maps_managers')):
                return candidate
    return os.path.join(os.path.dirname(ROOT), 'easynav_plugins')


def plugin_dir():
    """The source of the plugin, with the maps managers of easynav_plugins."""
    return os.path.join(plugins_src(), 'maps_managers', 'easynav_bigmap_maps_manager')


PLUGIN = plugin_dir()
SOURCE = os.path.join(PLUGIN, 'src', 'BigMapMapsManager.cpp')
# what each file is: real robot or simulation (use_sim_time), AMCL (its start is the spawn)
REAL_ROBOT = {'params_indoor.yaml', 'params_indoor_real.yaml'}
SIM_WITH_AMCL = {'params_parking_garage.yaml', 'params_large_warehouse.yaml',
                 'params_two_floors.yaml'}

# keys that exist in some scenarios only, and why
SCENARIO_ONLY = {
    'terrain.dem_file': 'outdoor: the DEM the generator writes',
    'terrain.dem_wait_timeout': 'outdoor: how long to wait for it',
}
# blocks that belong to a borrowed plugin: bigmap does not read their keys, the plugin does
BORROWED = ('inflation', 'obstacle_filter')


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def bigmap(path):
    return load(path)['maps_manager_node']['ros__parameters']['bigmap']


def flat(d, prefix=''):
    out = {}
    for k, v in d.items():
        key = prefix + k
        if isinstance(v, dict):
            out.update(flat(v, key + '.'))
        else:
            out[key] = v
    return out


def keys_read_by_the_code():
    """The first (= current) name of every parameter BigMapMapsManager.cpp reads."""
    with open(SOURCE) as f:
        code = f.read()
    keys = set(re.findall(r'param\(\s*"([^"]+)"', code))
    for group in re.findall(r'param_any\(\s*\{([^}]*)\}', code):
        keys.add(re.findall(r'"([^"]+)"', group)[0])
    return keys


needs_source = pytest.mark.skipif(not os.path.isfile(SOURCE), reason='easynav_bigmap_maps_manager source not found')


@pytest.mark.parametrize('path', CONFIGS, ids=os.path.basename)
def test_every_file_parses_and_has_all_the_nodes(path):
    d = load(path)
    for node in ('maps_manager_node', 'controller_node', 'localizer_node', 'planner_node',
                 'sensors_node', 'system_node'):
        assert node in d, node
        assert 'ros__parameters' in d[node]
    assert 'bigmap' in d['maps_manager_node']['ros__parameters']['map_types']


@needs_source
@pytest.mark.parametrize('path', CONFIGS, ids=os.path.basename)
def test_no_key_the_code_does_not_know(path):
    known = keys_read_by_the_code() | {'plugin', 'freq', 'filters'}
    b = {k: v for k, v in bigmap(path).items() if k not in BORROWED}
    unknown = sorted(k for k in flat(b) if k not in known)
    assert not unknown, 'keys no code reads (typo, or removed option): %s' % unknown


@needs_source
@pytest.mark.parametrize('path', CONFIGS, ids=os.path.basename)
def test_every_key_the_code_reads_is_in_the_file(path):
    present = set(flat({k: v for k, v in bigmap(path).items() if k not in BORROWED}))
    missing = sorted(k for k in keys_read_by_the_code() - present
                     if k not in SCENARIO_ONLY)
    assert not missing, 'options left to their default (document them in the file): %s' % missing


def test_the_files_have_the_same_structure():
    sets = {os.path.basename(p): set(flat(bigmap(p))) for p in CONFIGS}
    union = set().union(*sets.values())
    for name, keys in sets.items():
        gap = sorted(k for k in union - keys if k not in SCENARIO_ONLY)
        assert not gap, '%s lacks %s' % (name, gap)
    # the scenario-only keys appear where the DEM is (outdoor) and nowhere else
    for name, keys in sets.items():
        has = {k for k in SCENARIO_ONLY if k in keys}
        assert has == (set(SCENARIO_ONLY) if 'outdoor' in name else set()), name


def test_the_old_option_names_are_gone_from_the_files():
    for path in CONFIGS:
        b = bigmap(path)
        assert 'source' not in b['obstacles'], 'obstacles.source is now obstacles.detector'
        assert 'multilevel' in b and 'keep_narrow_width' not in b['multilevel']


@pytest.mark.parametrize('path', CONFIGS, ids=os.path.basename)
def test_lengths_agree_with_each_other(path):
    L = bigmap(path)['lengths']
    r, c = L['radii'], L['cell_size']
    assert r['traversed_mark'] < r['full_res'] <= r['low_res_start']
    assert r['sensor_range'] <= r['full_res']
    assert r['full_res'] <= r['traversed_keep']
    assert c['min'] <= c['max']
    assert L['traversed_fade'] > 0 and L['remesh_step'] > 0


def instance_kinds(b):
    """name -> 'obstacle' | 'inflation' for every entry of `filters`, from its plugin."""
    kinds = {}
    for name in b['filters']:
        assert name in b and 'plugin' in b[name], "'%s' is in filters but has no plugin" % name
        plugin = b[name]['plugin']
        assert plugin.endswith(('ObstaclesFilter', 'InflationFilter')), plugin
        kinds[name] = 'obstacle' if plugin.endswith('ObstaclesFilter') else 'inflation'
    return kinds


@pytest.mark.parametrize('path', CONFIGS, ids=os.path.basename)
def test_filters_and_obstacle_options_depend_on_each_other_as_documented(path):
    b = bigmap(path)
    kinds = instance_kinds(b)
    assert 'obstacles' not in b['filters'], "'obstacles' is bigmap's own section: obstacle_filter"
    detector = b['obstacles']['detector']
    assert detector in ('bigmap_dynamic', 'navmap_filter')
    has_obstacle_filter = 'obstacle' in kinds.values()
    # one detector: the filter is listed if and only if it is the detector
    assert has_obstacle_filter == (detector == 'navmap_filter')
    if detector == 'navmap_filter':
        assert not b['obstacles']['memory']['enabled'], 'the memory needs bigmap_dynamic'
    # the inflation never depends on the detector; it has to be bigger than the robot
    if 'inflation' in kinds.values():
        inscribed = load(path)['system_node']['ros__parameters']['robot_geometry']['inscribed_radius']
        assert b['inflation']['inflation_radius'] >= inscribed
    mem = b['obstacles']['memory']
    assert mem['min_time'] > 0 and 1 <= mem['max_hits'] <= 255
    if b['obstacles']['removal']['enabled'] is False:
        assert not (mem['enabled'] or b['dynamic_map']['enabled']), 'detections would not reach A*'


@pytest.mark.parametrize('path', CONFIGS, ids=os.path.basename)
def test_the_hierarchy_engine_has_a_map_and_the_files_exist(path):
    b = bigmap(path)
    t = b['terrain']
    if b['multilevel']['enabled'] and 'indoor' not in os.path.basename(path):
        # (params_indoor.yaml is the real robot's template: its map is the one thing to fill in)
        assert t['initial_navmap'], 'multilevel.enabled needs terrain.initial_navmap'
    if t['package'] and t['initial_navmap']:
        # relative to the package's share directory, which is this source tree once installed
        assert t['package'] == 'bigmap_bringup', 'only maps of this package are checked here'
        assert os.path.isfile(os.path.join(ROOT, t['initial_navmap'])), t['initial_navmap']
    bonxai = load(path)['maps_manager_node']['ros__parameters'].get('bonxai')
    if bonxai and bonxai.get('package') == 'bigmap_bringup':
        assert os.path.isfile(os.path.join(ROOT, bonxai['bonxai_path_file']))


@pytest.mark.parametrize('path', CONFIGS, ids=os.path.basename)
def test_simulation_flag_is_the_same_in_every_node(path):
    d = load(path)
    values = {n: v['ros__parameters']['use_sim_time'] for n, v in d.items()
              if isinstance(v, dict) and 'use_sim_time' in v.get('ros__parameters', {})}
    assert len(set(values.values())) == 1, values


FILTER_XML = os.path.join(plugins_src(), 'maps_managers', 'easynav_navmap_maps_manager')


@pytest.mark.skipif(not os.path.isdir(FILTER_XML), reason='easynav_navmap_maps_manager not found')
@pytest.mark.parametrize('path', CONFIGS, ids=os.path.basename)
def test_every_filter_plugin_is_declared_by_easynav_navmap_maps_manager(path):
    declared = set()
    for xml in glob.glob(os.path.join(FILTER_XML, '*filter.xml')):
        with open(xml) as f:
            declared.update(re.findall(r'<class\s+name="([^"]+)"', f.read()))
    b = bigmap(path)
    for name in ('inflation', 'obstacle_filter'):   # declared in the file, listed or not
        assert b[name]['plugin'] in declared, (name, b[name]['plugin'], sorted(declared))


@pytest.mark.parametrize('path', CONFIGS, ids=os.path.basename)
def test_simulation_or_real_robot_as_the_file_is_meant(path):
    d = load(path)
    sim = d['system_node']['ros__parameters']['use_sim_time']
    assert sim == (os.path.basename(path) not in REAL_ROBOT)


@pytest.mark.parametrize('path', [p for p in CONFIGS if os.path.basename(p) in SIM_WITH_AMCL],
                         ids=os.path.basename)
def test_simulations_say_where_the_robot_starts(path):
    # the launch spawns the robot at localizer_node.amcl.initial_pose: without it, at (0, 0)
    pose = load(path)['localizer_node']['ros__parameters']['amcl']['initial_pose']
    float(pose['x'])
    float(pose['y'])


@pytest.mark.parametrize('path', CONFIGS, ids=os.path.basename)
def test_goal_tolerances_agree_between_system_and_controller(path):
    d = load(path)
    system = d['system_node']['ros__parameters']
    ctrl = d['controller_node']['ros__parameters']
    for name in ctrl['controller_types']:
        c = ctrl[name]
        if 'xy_goal_tolerance' in c:
            assert c['xy_goal_tolerance'] == system['position_tolerance'], name
            assert c['yaw_goal_tolerance'] == system['angle_tolerance'], name


def outdoor():
    return [p for p in CONFIGS if 'dem_generator' in load(p)]


@pytest.mark.parametrize('path', outdoor(), ids=os.path.basename)
def test_the_maps_manager_reads_the_dem_the_generator_writes(path):
    d = load(path)
    gen = d['dem_generator']['ros__parameters']
    b = d['maps_manager_node']['ros__parameters']['bigmap']
    assert b['terrain']['dem_file'] == gen['cache_file']
    # the smallest NavCel is the DEM's node spacing (finer would only repeat nodes)
    assert b['lengths']['cell_size']['min'] == gen['grid_resolution']


def dem_module():
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'bigmap_dem', os.path.join(ROOT, 'bigmap_bringup', 'dem.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('path', outdoor(), ids=os.path.basename)
def test_no_generator_key_the_launch_does_not_read(path):
    """The `dem_generator` block is read by the launch (dem_generator_command): a key it does
    not know would be silently ignored."""
    launch = dem_module()
    gen = load(path)['dem_generator']['ros__parameters']
    unknown = sorted(k for k in flat(gen) if k not in launch.DEM_KEYS | launch.DEM_IGNORED_KEYS)
    assert not unknown, 'keys the launch does not read: %s' % unknown


@pytest.mark.parametrize('example, original', [
    ('example_indoor.yaml', 'params_indoor.yaml'),
    ('example_outdoor.yaml', 'params_outdoor.yaml'),
])
def test_the_plugins_examples_are_copies_of_these_files(example, original):
    """easynav_bigmap_maps_manager/config/example_*.yaml is params_indoor/outdoor.yaml with its own header
    (everything from "# WHO OWNS WHAT" on is equal): copy the body again when one changes."""
    path = os.path.join(PLUGIN, 'config', example)
    if not os.path.isfile(path):
        pytest.skip('easynav_bigmap_maps_manager source not found')

    def body(p):
        with open(p) as f:
            text = f.read()
        return text[text.index('# WHO OWNS WHAT'):]

    assert body(path) == body(os.path.join(ROOT, 'config', original)), \
        'copy the body of config/%s into easynav_bigmap_maps_manager/config/%s' % (original, example)
