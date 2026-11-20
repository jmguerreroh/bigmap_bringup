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

"""tools/wait_for_robot.py: what it says while the robot is not up (nothing is started)."""

import importlib.util
import os

import pytest

pytest.importorskip('rclpy')
TOOL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'tools',
                    'wait_for_robot.py')


def tool():
    spec = importlib.util.spec_from_file_location('wait_for_robot_under_test', TOOL)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_it_tells_nobody_publishes_tf_from_nothing_arrives_from_the_wrong_frame():
    m = tool()
    nobody = m.report('base_footprint', 10.0, 0, set())
    assert 'after 10 s' in nobody and 'nobody publishes /tf' in nobody
    # seen end to end: publishers in the graph, no message delivered to the new process
    lost = m.report('base_footprint', 20.0, 3, set())
    assert '3 publisher(s)' in lost and 'restart this launch' in lost
    wrong = m.report('base_footprint', 30.0, 2, {'base_link', 'wheel'})
    assert 'base_link, wheel' in wrong and "not 'base_footprint'" in wrong
