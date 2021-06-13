import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import xacro
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions.execute_process import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_file_from_pack(package_path, file_path):
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # planning_context
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("pick_place_moveit2"),
            "panda_description/urdf",
            "panda.urdf.xacro",
        ), 
    )
    robot_description = {"robot_description": robot_description_config.toxml(), "use_sim_time": True}
    
    install_dir = get_package_prefix('pick_place_moveit2')
	
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share"
        
    print(os.environ['GAZEBO_MODEL_PATH'])
    
    try:
        envs = {}
        for key in os.environ.__dict__["_data"]:
            key = key.decode("utf-8")
            if (key.isupper()):
                envs[key] = os.environ[key]
    except Exception as e:
        print("Error with Envs: " + str(e))
        return None
        
    # Static TF
    static_tf_robot = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="to_robot",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    static_tf_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="to_laser",
        output="log",
        arguments=["0.2", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "hokuyo_link"],
    )
       
    # Publish TF
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='screen',
                                 parameters=[robot_description])
    
    # gazebo
    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen',
            env=envs)

    gazebo = ExecuteProcess(
        cmd=['gazebo', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen', env=envs)
        
    # spawn robot
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'panda_arm'],
                        output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'joint_state_controller'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'panda_arm_controller'],
        output='screen'
    )
	        
    load_forward_command_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'initial_arm_controller'],
        output='screen'
    )
    
    move_initial_position = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-t 5', 'initial_arm_controller/commands', 'std_msgs/msg/Float64MultiArray', '{data: [0,-0.785,0,-2.356,0,1.571, 0.3]}'],
        output='screen'
    )
    
    stop_forward_command_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'switch_controllers', '--stop-controllers', 'initial_arm_controller'],
        output='screen'
    )
    
    return LaunchDescription([
      RegisterEventHandler(
          event_handler=OnProcessExit(
              target_action=spawn_entity,
              on_exit=[load_joint_state_controller],
          )
      ),
      RegisterEventHandler(
          event_handler=OnProcessExit(
              target_action=load_joint_state_controller,
              on_exit=[load_forward_command_controller],
          )
      ), 
      RegisterEventHandler(
          event_handler=OnProcessExit(
              target_action=load_forward_command_controller,
              on_exit=[move_initial_position],
          )
      ), 
      RegisterEventHandler(
          event_handler=OnProcessExit(
              target_action=move_initial_position,
              on_exit=[stop_forward_command_controller],
          )
      ),
      RegisterEventHandler(
          event_handler=OnProcessExit(
              target_action=stop_forward_command_controller,
              on_exit=[load_joint_trajectory_controller],
          )
      ),  
      gazebo,
      robot_state_publisher,
      static_tf_robot,
      static_tf_laser,
      spawn_entity
    ])
          
    
