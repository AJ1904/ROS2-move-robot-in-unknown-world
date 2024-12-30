from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable, LaunchConfiguration, LocalSubstitution, PythonExpression)
import sys
import json
from launch_ros.parameter_descriptions import ParameterValue
from project4c.disc_robot import load_disc_robot, disc_robot_urdf
import yaml
        
def generate_launch_description():
    # Extracting robot_file from command line arguments
    for arg in sys.argv:
        if arg.startswith("robot_file:="):
            robot_file = str(arg.split(":=")[1])
            break
        else:
            robot_file = 'src/project4c/robot_files/normal.robot'
            
    # Load robot parameters from the specified file
    robot = load_disc_robot(robot_file)
    robot_desc = robot['urdf']
    radius = robot['body']['radius']
    height = robot['body']['height']
    distance = robot['wheels']['distance']
    error_variance_left = robot['wheels']['error_variance_left']
    error_variance_right = robot['wheels']['error_variance_right']
    error_update_rate = robot['wheels']['error_update_rate']
    
    # Parameters for laser
    laser_rate = robot['laser']['rate']
    laser_count = robot['laser']['count']
    laser_angle_min = robot['laser']['angle_min']
    laser_angle_max = robot['laser']['angle_max']
    laser_range_min = robot['laser']['range_min']
    laser_range_max = robot['laser']['range_max']
    laser_error_variance = robot['laser']['error_variance']
    laser_fail_probability  = robot['laser']['fail_probability']
    
    # LaunchDescription object
    ld = LaunchDescription(
        [
        # declares the rosbag_in parameter as a command line argument
        # DeclareLaunchArgument(
        #     'bag_in',
        #     default_value='src/project4c/input_bags/input',
        #     description='Input bag file'
        # ),
        # declares the rosbag_out parameter as a command line argument
        DeclareLaunchArgument(
            'bag_out',
            default_value='src/project4c/output_bags/output',
            description='Output bag file'
        ),

        DeclareLaunchArgument(
            'world_file',
            default_value='src/project4c/world_files/brick.world',
            description='World file'
        ),
        # declares the robot_file parameter as a command line argument
        DeclareLaunchArgument(
            'robot_file',
            default_value='src/project4c/robot_files/normal.robot',
            description='Input robot file'
        ),
        ]
    )

    #navigation_controller_node

    navigation_controller_node = Node(
        package='project4c',
        executable='navigation_controller_node',
        parameters=[{
            'robot_description': robot_desc, 
        	'radius': radius,
        	'height': height,
        	}]
    )
  
    # Simulator Node
    simulator_node = Node(
        package='project4c',
        executable='simulator_node',
        parameters=[{
            'world_file': LaunchConfiguration('world_file'),
            'robot_description': robot_desc, 
        	'radius': radius,
        	'height': height,
        	'distance': distance,
        	'error_variance_left': error_variance_left,
        	'error_variance_right': error_variance_right,
        	'error_update_rate': error_update_rate,
        	'laser_rate': laser_rate,
        	'laser_count': laser_count,
        	'laser_angle_min': laser_angle_min,
        	'laser_angle_max': laser_angle_max,
        	'laser_range_min': laser_range_min,
        	'laser_range_max': laser_range_max,
        	'laser_error_variance': laser_error_variance,
        	'laser_fail_probability': laser_fail_probability
        	}]
        )

    # # Velocity Translator Node
    velocity_translator_node = Node(
         package='project4c',
         executable='velocity_translator_node',
         parameters=[{'robot_description': robot_desc,
         	'radius': radius,
        	'distance': distance}]
         )
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
     	package='robot_state_publisher',
     	executable='robot_state_publisher',
     	name='robot_state_publisher',
     	namespace='',
     	output='screen',
     	parameters=[{'robot_description': robot_desc}]
     	)

    # # ExecuteProcess to play a bag file
    # play_bag = ExecuteProcess(
    #       cmd=[[
    #           'ros2 bag play ',
    #           LaunchConfiguration('bag_in'),
    #       ]],
    #       shell=True
    #   )

    # ExecuteProcess to record a bag file
    record_bag = ExecuteProcess(
         name="record_bag",
         cmd=[[
             'ros2 bag record -a --storage sqlite3 -o ',
             LaunchConfiguration('bag_out')
         ]],
         shell=True
     )
    
    # Add actions to the LaunchDescription
    # ld.add_action(play_bag)
    ld.add_action(record_bag)
    ld.add_action(simulator_node)
    ld.add_action(velocity_translator_node)
    ld.add_action(navigation_controller_node)
    ld.add_action(robot_state_publisher_node)

    return ld

    
