import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('aagv_description')
    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'aagv.urdf.xacro'])
    bridge_config = PathJoinSubstitution([pkg, 'config', 'gz_bridge.yaml'])
    world = PathJoinSubstitution([pkg, 'worlds', 'empty_sensors.sdf'])

    # Let Gazebo resolve model:// mesh URIs by pointing at the package share parents.
    share_parents = []
    for p in ('open_manipulator_description', 'realsense2_description', 'aagv_description'):
        try:
            share_parents.append(os.path.dirname(get_package_share_directory(p)))
        except Exception:
            pass
    gz_resource = os.pathsep.join(share_parents)
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        gz_resource = gz_resource + os.pathsep + os.environ['GZ_SIM_RESOURCE_PATH']

    set_resource = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource)

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str
        )
    }

    ros_gz_sim = FindPackageShare('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': [' -r -v4 ', world]}.items(),
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'aagv', '-z', '0.0'],
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
    )

    return LaunchDescription([set_resource, gz_sim, rsp, spawn, bridge])
