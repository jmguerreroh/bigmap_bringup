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

"""bigmap_geo.launch.py: the playground of the place and the application that follows it, in that
order, without starting anything."""

import importlib.util
import os

import pytest

launch = pytest.importorskip('launch')
pytest.importorskip('bigmap_bringup.dem')
from ament_index_python.packages import get_package_share_directory  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH_FILE = os.path.join(os.path.dirname(HERE), 'launch', 'bigmap_geo.launch.py')


def module():
    spec = importlib.util.spec_from_file_location('bigmap_geo_launch', LAUNCH_FILE)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def needs(pkg):
    try:
        return get_package_share_directory(pkg)
    except Exception:
        pytest.skip('%s is not installed' % pkg)


def context(**given):
    m = module()
    ctx = launch.LaunchContext()
    for action in m.generate_launch_description().entities:
        if isinstance(action, launch.actions.DeclareLaunchArgument):
            ctx.launch_configurations[action.name] = given.pop(action.name, None) or ''.join(
                s.perform(ctx) for s in action.default_value)
    assert not given, 'unknown launch arguments %s' % given
    return m, ctx


def text(parts, ctx):
    if isinstance(parts, str):
        return parts
    if not isinstance(parts, (list, tuple)):
        parts = [parts]
    return ''.join(p if isinstance(p, str) else p.perform(ctx) for p in parts)


def includes(actions, ctx):
    """(file, {argument: value}) of each launch file included, in order."""
    out = []
    for a in actions:
        assert isinstance(a, launch.actions.IncludeLaunchDescription)
        # .location is the text of the substitutions until the launch is run: the source's own
        path = text(a.launch_description_source._LaunchDescriptionSource__location, ctx)
        args = {text(k, ctx): text(v, ctx) for k, v in a.launch_arguments}
        out.append((os.path.basename(path), args))
    return out


def test_the_arguments_of_the_playground_are_its_own_with_its_defaults():
    yaml = pytest.importorskip('yaml')
    geo = needs('easynav_playground_geo')
    with open(os.path.join(geo, 'launch', 'easynav_geo.launch.yaml')) as f:
        declared = {e['arg']['name']: str(e['arg']['default'])
                    for e in yaml.safe_load(f)['launch'] if 'arg' in e}
    m = module()
    assert set(m.GEO_ARGS) == set(declared)
    for name, (default, _) in m.GEO_ARGS.items():
        assert default == declared[name], name


def test_the_playground_is_launched_first_and_the_application_follows_it():
    needs('easynav_playground_geo')
    needs('bigmap_bringup')
    m, ctx = context(world='area', lat='40.4168', lon='-3.7038', radius='250', gui='false')
    (geo, geo_args), (app, app_args) = includes(m.setup(ctx), ctx)
    assert geo == 'easynav_geo.launch.yaml' and app == 'bigmap.launch.py'
    assert geo_args['world'] == 'area' and geo_args['lat'] == '40.4168'
    assert geo_args['radius'] == '250' and geo_args['gui'] == 'false'
    assert app_args['sim'] == 'true'
    assert app_args['params_file'].endswith(os.path.join('config', 'params_outdoor.yaml'))
    # the place of the playground is not a place of the application's (it follows the world)
    assert app_args['lat'] == app_args['lon'] == app_args['radius'] == ''


def test_a_use_case_whose_map_is_a_saved_navmap_is_refused():
    needs('easynav_playground_geo')
    needs('bigmap_bringup')
    m, ctx = context(config='parking_garage')
    with pytest.raises(RuntimeError, match='no dem_generator block'):
        m.setup(ctx)
