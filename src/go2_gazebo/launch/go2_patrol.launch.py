# go2_patrol.launch.py -- bring up the whole Go2 patrol simulation:
#   optional PGM floor-plan import (go2_scene.py) -> URDF->SDF conversion
#   (go2.urdf + dae + scenery), gz sim, robot_state_publisher (static TF),
#   go2_driver (odom + gait animation), ros_gz_bridge (parameter_bridge),
#   scan-noise relay, optional RViz, optional auto-demo.
#
# Usage:
#   ros2 launch go2_gazebo go2_patrol.launch.py
#   ros2 launch go2_gazebo go2_patrol.launch.py demo:=20.0     # drive itself
#   ros2 launch go2_gazebo go2_patrol.launch.py gui:=false     # no RViz/GUI
#   ros2 launch go2_gazebo go2_patrol.launch.py \
#       scene_map:=/path/to/floorplan.pgm scene_res:=0.05      # own floor plan
#   ros2 launch go2_gazebo go2_patrol.launch.py scan_noise:=0.0  # raw laser
#
# Drive it from a terminal:
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard
# (or `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...`).

import os
import shutil
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- share dirs ---------------------------------------------------------
    desc_share = get_package_share_directory('go2_description')
    gz_share = get_package_share_directory('go2_gazebo')

    urdf_path = os.path.join(desc_share, 'urdf', 'go2.urdf')
    scenery_path = os.path.join(gz_share, 'worlds', 'go2_patrol.sdf')
    converter = os.path.join(gz_share, 'scripts', 'go2_urdf2sdf.py')
    scene_tool = os.path.join(gz_share, 'scripts', 'go2_scene.py')
    ros_home = os.path.join(os.path.expanduser('~'), '.ros')
    world_out = os.path.join(ros_home, 'go2_patrol_generated.sdf')

    # ---- args ---------------------------------------------------------------
    demo_arg = DeclareLaunchArgument(
        'demo', default_value='0.0',
        description='seconds of an automatic /cmd_vel patrol demo (0 = off)')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='launch RViz + gz GUI window (false = headless -s only)')
    scene_map_arg = DeclareLaunchArgument(
        'scene_map', default_value='',
        description='path to an occupancy-grid PGM to use as the scene instead '
                    'of the default warehouse (empty = default)')
    scene_res_arg = DeclareLaunchArgument(
        'scene_res', default_value='0.30',
        description='metres per pixel when importing scene_map')
    scan_noise_arg = DeclareLaunchArgument(
        'scan_noise', default_value='0.015',
        description='gaussian stddev (m) added to /scan (0 = pass-through)')

    demo_s = LaunchConfiguration('demo')
    gui = LaunchConfiguration('gui')

    def convert_then_gz(context):
        """Import a custom floor plan (if any), run the URDF->SDF converter,
        then start gz sim against the generated world.  Runs synchronously so
        gz only starts once the world file exists."""
        map_pgm = context.perform_substitution(
            LaunchConfiguration('scene_map')).strip()
        res = float(context.perform_substitution(
            LaunchConfiguration('scene_res')))
        headless = context.perform_substitution(gui).strip().lower() == 'false'

        scenery = scenery_path
        if map_pgm:
            tmp_world = os.path.join(ros_home, 'go2_scene_custom.sdf')
            shutil.copyfile(scenery_path, tmp_world)
            r = subprocess.run(
                [sys.executable, scene_tool, 'import',
                 '--pgm', map_pgm, '--res', str(res),
                 '--apply-to', tmp_world],
                capture_output=True, text=True)
            if r.returncode != 0:
                raise RuntimeError('go2_scene.py import failed:\n' + r.stderr)
            scenery = tmp_world

        r = subprocess.run(
            [sys.executable, converter,
             '--urdf', urdf_path,
             '--share', desc_share,
             '--scenery', scenery,
             '--out', world_out,
             '--spawn-z', '0.32'],
            capture_output=True, text=True)
        if r.returncode != 0:
            raise RuntimeError('go2_urdf2sdf.py failed:\n' + r.stderr)

        cmd = ['gz', 'sim', '-r']
        if headless:
            cmd.append('-s')
        cmd += ['-v', '2', world_out]
        return [ExecuteProcess(cmd=cmd, output='screen')]

    gz_action = OpaqueFunction(function=convert_then_gz)

    with open(urdf_path, 'r', encoding='utf-8') as fh:
        robot_desc = fh.read()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}])

    driver = Node(
        package='go2_gazebo',
        executable='go2_driver',
        name='go2_driver',
        output='screen')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{'config_file': os.path.join(
            gz_share, 'config', 'go2_patrol_bridge.yaml')}])

    # gz-sim GPU lidar has no native noise -> ROS-side relay adds gaussian
    # range noise to /scan (raw echo stays on /scan_raw).
    scan_noise = ExecuteProcess(
        cmd=[sys.executable,
             os.path.join(gz_share, 'scripts', 'go2_scan_noise.py'),
             '--ros-args', '-p',
             ['scan_noise_std:=', LaunchConfiguration('scan_noise')]],
        output='screen')

    # Exits immediately when demo:=0.
    demo = ExecuteProcess(
        cmd=[sys.executable,
             os.path.join(gz_share, 'scripts', 'go2_demo.py'), demo_s],
        output='screen')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(gz_share, 'config', 'go2_patrol.rviz')],
        output='screen', condition=IfCondition(gui))

    hint = LogInfo(msg=(
        '>>> Go2 patrol sim running (indoor scene).\n'
        'Drive:    ros2 run teleop_twist_keyboard teleop_twist_keyboard\n'
        'Topics:   /odom /scan /scan_raw /imu/data /joint_states\n'
        '          /camera/color/image_raw /camera/depth/image_raw\n'))

    return LaunchDescription([
        demo_arg, gui_arg, scene_map_arg, scene_res_arg, scan_noise_arg,
        gz_action, rsp, driver, bridge, scan_noise, demo, rviz, hint,
    ])
