#!/usr/bin/env python3
"""ROS 1 alias for the former ROS 2 Nav2 fleet agent.

The 332 stack uses the repository-native Gazebo Transport Twist controller and
keeps the external fleet command, state and acknowledgement message contract.
"""

import os
import runpy


if __name__ == '__main__':
    runpy.run_path(
        os.path.join(os.path.dirname(__file__), 'usv_gz_fleet_agent.py'),
        run_name='__main__',
    )
