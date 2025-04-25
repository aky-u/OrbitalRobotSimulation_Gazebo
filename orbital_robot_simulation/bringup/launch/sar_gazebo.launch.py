import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro

pkg_dir = get_package_share_directory("orbital_robot_simulation")
sar_xacro_path = os.path.join(
    pkg_dir, "description", "urdf", "SingleArmRobot.xacro")
sar_urdf_path = os.path.join(
    pkg_dir, "description", "urdf", "SingleArmRobot.urdf")
rviz_path = os.path.join(pkg_dir, "description", "rviz", "sar_config.rviz")
world_path = os.path.join(pkg_dir, "bringup", "worlds", "on_orbit.world")
use_sim_time = LaunchConfiguration('use_sim_time', default='true')


def generate_launch_description():
    # Single Arm Robot
    # make urdf from xacro
    # load xacro
    doc = xacro.process_file(sar_xacro_path)
    # make urdf
    robot_desc = doc.toprettyxml(indent=' ')
    # export urdf to urdf path
    f = open(sar_urdf_path, 'w')
    f.write(robot_desc)
    f.close()

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager",
                   "/controller_manager"]
    )
    joint_state_command_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        remappings=[
            ('/joint_states', '/joint_state_command')],
        arguments=[sar_urdf_path]
    )

    joint_state_to_trajectory_node = Node(
        package="orbital_robot_simulation",
        executable="joint_state_to_trajectory_node"
    )

    joint_effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_effort_controller", "--controller-manager",
                   "/controller_manager"]
    )

    joint_position_controller_spawner = Node( 
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager",
                     "/controller_manager"]
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["feedback_effort_controller", "--controller-manager",
                    "/controller_manager"]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        arguments=[sar_urdf_path]
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_path]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("gazebo_ros"), "launch"),
            "/gazebo.launch.py"]),
        launch_arguments={'world': world_path}.items()
    )

    robot_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="urdf_spawner",
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=["-topic", "/robot_description", "-entity", "sar_simple"]
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        # joint_effort_controller_spawner,
        rviz2,
        gazebo,
        joint_state_command_publisher_gui,
        joint_state_to_trajectory_node,
        robot_state_publisher_node,
        robot_spawner

        # For debagging
        # rviz2,
        # joint_state_publisher_gui,
        # robot_state_publisher_node,
        # end_effector_trajectory_controller_spawner
    ])
