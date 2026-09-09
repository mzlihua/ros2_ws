# go2_patrol.launch.py -- bring up the whole Go2 patrol simulation:
#   URDF -> SDF conversion (go2.urdf + dae -> go2_patrol world), gz sim,
#   robot_state_publisher (static TF), go2_driver (odom + gait animation),
#   ros_gz_bridge (parameter_bridge), optional RViz, optional auto-demo.
#
# Usage:
#   ros2 launch go2_gazebo go2_patrol.launch.py
#   ros2 launch go2_gazebo go2_patrol.launch.py demo:=20.0     # drive itself
#   ros2 launch go2_gazebo go2_patrol.launch.py gui:=false     # no RViz/GUI
#
# Drive it from a terminal:
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard
# (or `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...`).

import os
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- share dirs ---------------------------------------------------------
    desc_share = get_package_share_directory('go2_description')
    gz_share = get_package_share_directory('go2_gazebo')

    urdf_path = os.path.join(desc_share, 'urdf', 'go2.urdf')
    scenery_path = os.path.join(gz_share, 'worlds', 'go2_patrol.sdf')
    converter = os.path.join(gz_share, 'scripts', 'go2_urdf2sdf.py')
    world_out = os.path.join(os.path.expanduser('~'), '.ros',
                             'go2_patrol_generated.sdf')

    # ---- regenerate the world (URDF model injected into the scenery) --------
    r = subprocess.run(
        [sys.executable, converter,
         '--urdf', urdf_path,
         '--share', desc_share,
         '--scenery', scenery_path,
         '--out', world_out,
         '--spawn-z', '0.32'],
        capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError('go2_urdf2sdf.py failed:\n' + r.stderr)

    # ---- args ---------------------------------------------------------------
    demo_arg = DeclareLaunchArgument(
        'demo', default_value='0.0',
        description='seconds of an automatic /cmd_vel patrol demo (0 = off)')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='launch RViz + gz GUI window (false = headless -s only)')

    gui = LaunchConfiguration('gui')
    demo_s = LaunchConfiguration('demo')

    # ---- processes ----------------------------------------------------------
    # gz sim. Headless mode (-s) is used when gui:=false.
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '2', world_out],
        output='screen', condition=IfCondition(gui))
    gz_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '-v', '2', world_out],
        output='screen', condition=UnlessCondition(gui))

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
        '>>> Go2 patrol sim running.\n'
        'Drive:    ros2 run teleop_twist_keyboard teleop_twist_keyboard\n'
        'Topics:   /odom /scan /imu/data /joint_states\n'
        '          /camera/color/image_raw /camera/depth/image_raw\n'))

    return LaunchDescription([
        demo_arg, gui_arg,
        gz_gui, gz_headless, rsp, driver, bridge, demo, rviz, hint,
    ])
