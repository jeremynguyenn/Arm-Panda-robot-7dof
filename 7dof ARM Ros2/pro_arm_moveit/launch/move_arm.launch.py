#!/usr/bin/env -S ros2 launch
"""Configure and setup move group for planning with MoveIt 2"""

from os import path
from typing import List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    description_package = LaunchConfiguration("description_package")
    description_filepath = LaunchConfiguration("description_filepath")
    moveit_config_package = "pro_arm_moveit"
    name = LaunchConfiguration("name")
    prefix = LaunchConfiguration("prefix")
    dof = LaunchConfiguration("dof")
    size = LaunchConfiguration("size")
    gripper = LaunchConfiguration("gripper")
    finger = LaunchConfiguration("finger")
    # collision = LaunchConfiguration("collision")
    # safety_limits = LaunchConfiguration("safety_limits")
    # safety_position_margin = LaunchConfiguration("safety_position_margin")
    ros2_control = LaunchConfiguration("ros2_control")
    ros2_control_plugin = LaunchConfiguration("ros2_control_plugin")
    # gazebo_preserve_fixed_joint = LaunchConfiguration("gazebo_preserve_fixed_joint")
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    # URDF
    _robot_description_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), description_filepath]
            ),
            " ",
            "name:=",
            name,
            " ",
            "prefix:=",
            prefix,
            " ",
            "dof:=",
            dof,
            " ",
            "size:=",
            size,
            " ",
            "gripper:=",
            gripper,
            " ",
            "finger:=",
            finger,
            " ",
            "ros2_control:=",
            ros2_control,
            " ",
            "ros2_control_plugin:=",
            ros2_control_plugin
        ]
    )
    robot_description = {"robot_description": _robot_description_xml}

    # SRDF
    _robot_description_semantic_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", "pro_arm.srdf.xacro"]
            ),
            " ",
            "name:=",
            name,
            " ",
            "prefix:=",
            prefix,
            " ",
            "dof:=",
            dof,
            " ",
            "size:=",
            size,
            " ",
            "gripper:=",
            gripper,
            " ",
            "finger:=",
            finger
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": _robot_description_semantic_xml
    }

    # Kinematics
    kinematics = load_yaml(
        moveit_config_package, path.join("config", "kinematics.yaml")
    )

    # Joint limits
    joint_limits = {
        "robot_description_planning": load_yaml(
            moveit_config_package, path.join("config", "joint_limits.yaml")
        )
    }

    # Planning pipeline
    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        },
    }
    _ompl_yaml = load_yaml(
        moveit_config_package, path.join("config", "ompl_planning.yaml")
    )
    planning_pipeline["ompl"].update(_ompl_yaml)

    # Planning scene
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # MoveIt controller manager
    # Define the parameters for the controllers
    
    
    moveit_controller_manager_no_gripper = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        
        # List of controller names
        'moveit_simple_controller_manager.controller_names': ['arm_trajectory_controller'],
        
        # Parameters specific to each controller
        'moveit_simple_controller_manager.arm_trajectory_controller.action_ns': 'follow_joint_trajectory',
        'moveit_simple_controller_manager.arm_trajectory_controller.type': 'FollowJointTrajectory',
        'moveit_simple_controller_manager.arm_trajectory_controller.joints': [
            'pro_arm_joint_1', 'pro_arm_joint_2', 'pro_arm_joint_3',
            'pro_arm_joint_4', 'pro_arm_joint_5', 'pro_arm_joint_6'
        ]
    }

    moveit_controller_manager_gripper = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        
        # List of controller names
        'moveit_simple_controller_manager.controller_names': ['arm_trajectory_controller', 'gripper_action_controller'],
        
        # Parameters specific to each controller
        'moveit_simple_controller_manager.arm_trajectory_controller.action_ns': 'follow_joint_trajectory',
        'moveit_simple_controller_manager.arm_trajectory_controller.type': 'FollowJointTrajectory',
        'moveit_simple_controller_manager.arm_trajectory_controller.joints': [
            'pro_arm_joint_1', 'pro_arm_joint_2', 'pro_arm_joint_3',
            'pro_arm_joint_4', 'pro_arm_joint_5', 'pro_arm_joint_6'
        ],
        'moveit_simple_controller_manager.arm_trajectory_controller.default': True,
        
        'moveit_simple_controller_manager.gripper_action_controller.action_ns': 'gripper_cmd',
        'moveit_simple_controller_manager.gripper_action_controller.type': 'GripperCommand',
        'moveit_simple_controller_manager.gripper_action_controller.joints': ['joint_7', 'joint_8'],
        'moveit_simple_controller_manager.gripper_action_controller.default': True,
    }

    # Trajectory execution
    trajectory_execution = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # Controller parameters
    declared_arguments.append(
        DeclareLaunchArgument(
            "__controller_parameters_basename",
            default_value=["controllers_", dof, "dof.yaml"]
        )
    )

    controller_parameters = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_package),
            "config",
            LaunchConfiguration("__controller_parameters_basename"),
        ]
    )

    # List of nodes to be launched
    nodes = [
        # robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                {
                    "publish_frequency": 30.0,
                    "frame_prefix": "",
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                controller_parameters,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics,
                joint_limits,
                planning_pipeline,
                trajectory_execution,
                planning_scene_monitor_parameters,
                moveit_controller_manager_no_gripper,
                {"use_sim_time": use_sim_time},
            ],
            condition=(
                IfCondition(
                    PythonExpression(
                        ["'", gripper, "' == 'none'"]
                    )
                )
            ),
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics,
                joint_limits,
                planning_pipeline,
                trajectory_execution,
                planning_scene_monitor_parameters,
                moveit_controller_manager_gripper,
                {"use_sim_time": use_sim_time},
            ],
            condition=(
                IfCondition(
                    PythonExpression(
                        ["'", gripper, "' != 'none'"]
                    )
                )
            ),
        ),
        # rviz2
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=[
                "--display-config",
                rviz_config,
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics,
                planning_pipeline,
                joint_limits,
                {"use_sim_time": use_sim_time},
            ],
            condition=IfCondition(enable_rviz),
        ),
        # Add the emergency stop marker node
        Node(
            package="pro_arm_moveit",
            executable="emergency_stop_marker.py",
            name="emergency_stop_marker",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    # Add nodes for loading controllers
    for controller in ["joint_state_broadcaster", "arm_trajectory_controller"]:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                output="log",
                arguments=[controller, "--ros-args", "--log-level", log_level],
                parameters=[{"use_sim_time": use_sim_time}],
             ),
        )

    nodes.append(
        Node(
            package="controller_manager",
            executable="spawner",
            output="log",
            arguments=["gripper_action_controller", "--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(PythonExpression(["'", gripper, "' != 'none'"]))
        )
    )

    return LaunchDescription(declared_arguments + nodes)

def load_yaml(package_name: str, file_path: str):
    """
    Load yaml configuration based on package name and file path relative to its share.
    """

    package_path = get_package_share_directory(package_name)
    absolute_file_path = path.join(package_path, file_path)
    return parse_yaml(absolute_file_path)


def parse_yaml(absolute_file_path: str):
    """
    Parse yaml from file, given its absolute file path.
    """

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # Locations of robot resources
        DeclareLaunchArgument(
            "description_package",
            default_value="pro_arm_description",
            description="Custom package with robot description.",
        ),
        DeclareLaunchArgument(
            "description_filepath",
            default_value="urdf/pro_arm.urdf.xacro",
            description="Path to xacro or URDF description of the robot, relative to share of `description_package`.",
        ),
        # Naming of the robot
        DeclareLaunchArgument(
            "name",
            default_value="pro_arm",
            description="Name of the robot.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix for all robot entities. If modified, then joint names in the configuration of controllers must also be updated.",
        ),
        # Version
        DeclareLaunchArgument(
            "dof",
            default_value='6',
            choices=['5','6'],
            description="Parameter to select dof version."
        ),
        DeclareLaunchArgument(
            "size",
            default_value='550',
            choices=['550','900'],
            description="Parameter to select size version."
        ),
        DeclareLaunchArgument(
            "gripper",
            default_value='none',
            choices=['none','pge_5040','cge_1010'],
            description="Parameter to select gripper model."
        ),
        DeclareLaunchArgument(
            "finger",
            default_value='40',
            choices=['20','40','60','80'],
            description="Parameter to select finger separation model."
        ),
        # # Safety controller
        # DeclareLaunchArgument(
        #     "safety_limits",
        #     default_value="true",
        #     description="Flag to enable safety limits controllers on manipulator joints.",
        # ),
        # DeclareLaunchArgument(
        #     "safety_position_margin",
        #     default_value="0.15",
        #     description="Lower and upper margin for position limits of all safety controllers.",
        # ),
        # ROS 2 control
        DeclareLaunchArgument(
            "ros2_control",
            default_value="true",
            description="Flag to enable ros2 controllers for manipulator.",
        ),
        DeclareLaunchArgument(
            "ros2_control_plugin",
            default_value="fake",
            description="The ros2_control plugin that should be loaded for the manipulator ('fake', 'sim', 'real' or custom).",
        ),
        # # Gazebo
        # DeclareLaunchArgument(
        #     "gazebo_preserve_fixed_joint",
        #     default_value="false",
        #     description="Flag to preserve fixed joints and prevent lumping when generating SDF for Gazebo.",
        # ),
        # Miscellaneous
        DeclareLaunchArgument(
            "enable_rviz", default_value="true", description="Flag to enable RViz2."
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("pro_arm_moveit"),
                "rviz",
                "moveit.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]