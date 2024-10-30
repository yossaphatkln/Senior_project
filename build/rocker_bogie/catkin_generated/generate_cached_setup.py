# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/home/europazas/miniforge3/envs/ros_env/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/home/europazas/miniforge3/envs/ros_env/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/europazas/Desktop/Code/Senior_project/devel;/home/europazas/Desktop/Code/ROS/catkin_ws/devel;/home/europazas/miniforge3/envs/ros_env'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3.11/site-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/europazas/Desktop/Code/Senior_project/devel/.private/rocker_bogie/env.sh')

output_filename = '/home/europazas/Desktop/Code/Senior_project/build/rocker_bogie/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
