#!/usr/bin/env bash
set -e

source /opt/ros/lyrical/setup.bash

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_POLICY_VERSION_MINIMUM=3.5
