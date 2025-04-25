# export urdf from xacro
# vis model in rviz
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():

  pkg_dir = get_package_share_directory("orbital_robot_simulation")
  urdf_path= os.path.join(pkg_dir, "urdf", "dar.urdf")
  xacro_path = os.path.join(pkg_dir, 'urdf', 'dar.urdf.xacro')
  rviz_path = os.path.join(pkg_dir, "rviz", "dar_config.rviz")

  #### make urdf from xacro
  # load xacro
  doc = xacro.process_file(xacro_path)
  # make urdf
  robot_desc = doc.toprettyxml(indent=' ')
  # export urdf to urdf path
  f = open(urdf_path, 'w')
  f.write(robot_desc)
  f.close()

  joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_path])
  
  rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_path])
  
  # gazebo
  # gazebo = IncludeLaunchDescription(
  #   PythonLaunchDescriptionSource([os.path.join(
  #       get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
  #       launch_arguments={'world': world_path}.items(),)
  
  load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_state_broadcaster'],
        output='screen')
  
  # load_joint_trajectory_controller = ExecuteProcess(
  #       cmd=['ros2', 'control', 'load_start_controller', 'joint_trajectory_controller'],
  #       output='screen')

  robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        arguments=[urdf_path])

  # spawn_entity = Node(
  #   package="gazebo_ros",
  #   executable="spawn_entity.py",
  #   name="urdf_spawner",
  #   parameters=[{'use_sim_time': use_sim_time}],
  #   arguments=["-topic", "/robot_description", "-entity", "dar_simple"])

  return LaunchDescription([
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     arguments=[urdf_path]
        # ),

        joint_state_publisher_gui,
        rviz2,

        #gazebo settings
        # ExecuteProcess(
        #     cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"],
        #     ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_entity,
        #         on_exit=[load_joint_state_controller],
        #     )
        # ),

        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_controller,
        #         on_exit=[
        #             load_joint_trajectory_controller,
        #         ],
        #     )
        # ),

        # gazebo,
        robot_state_pub_node,
        # spawn_entity,
  ])