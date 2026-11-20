#!/usr/bin/env python3
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

"""Exits when the simulated robot is up: its odometry transform (a /tf whose child frame is
--frame, odom -> base_footprint by default) is being published. bigmap.launch.py starts EasyNav
when this exits, instead of after a fixed delay. After --timeout seconds it exits anyway (with a
warning), so a robot without that transform does not block the launch."""

import argparse
import sys
import time

import rclpy
from tf2_msgs.msg import TFMessage


def report(frame, waited, publishers, frames):
    """Why the robot is not up yet, from what /tf has and has not brought."""
    head = "no transform to '%s' after %.0f s: " % (frame, waited)
    if publishers == 0:
        return head + 'nobody publishes /tf (is the robot, or its simulation, running?)'
    if not frames:
        return head + ('%d publisher(s) of /tf, but no message has reached this process: if the '
                       'robot is up for other processes (ros2 topic echo /tf), restart this '
                       'launch' % publishers)
    return head + "/tf brings %s but not '%s' (is it the robot's odometry frame?)" % (
        ', '.join(sorted(frames)[:8]), frame)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frame', default='base_footprint')
    parser.add_argument('--timeout', type=float, default=120.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('wait_for_robot')
    seen = []
    frames = set()   # the child frames that did arrive, to tell what is missing

    def on_tf(msg):
        for t in msg.transforms:
            frames.add(t.child_frame_id.lstrip('/'))
        if any(t.child_frame_id.lstrip('/') == args.frame for t in msg.transforms):
            seen.append(True)

    node.create_subscription(TFMessage, '/tf', on_tf, 10)
    node.get_logger().info("waiting for the transform to '%s'" % args.frame)
    start = time.monotonic()
    end = start + args.timeout
    next_report = start + 10.0
    while rclpy.ok() and not seen and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if not seen and time.monotonic() >= next_report:
            next_report += 10.0
            node.get_logger().warning(report(
                args.frame, time.monotonic() - start, node.count_publishers('/tf'), frames))
    if seen:
        node.get_logger().info('the robot is up: starting EasyNav')
    else:
        node.get_logger().warning(
            "no transform to '%s' after %.0f s: starting EasyNav anyway" % (
                args.frame, args.timeout))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
