from launch import LaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

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
    load_joint_state_controller
    ])
          
    
