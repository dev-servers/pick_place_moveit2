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
        ), mappings={"use_sim": "true"}
    )
    robot_description = {"robot_description": robot_description_config.toxml()}
    
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
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
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
    
    
    robot_description_semantic_config = load_file(
        "moveit_resources_panda_moveit_config", "config/panda.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/kinematics.yaml"
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/panda_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 10.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )
    
    # RViz
    rviz_config_file = (
        get_package_share_directory("run_move_group") + "/launch/run_move_group.rviz"
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
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
      RegisterEventHandler(
          event_handler=OnProcessExit(
              target_action=load_joint_trajectory_controller,
              on_exit=[rviz_node, run_move_group_node],
          )
      ),
      gazebo,
      robot_state_publisher,
      static_tf,
      spawn_entity
    ])
          
    
