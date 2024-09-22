import os
import yaml
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    gripper_max_velocity = LaunchConfiguration("gripper_max_velocity")
    gripper_max_force = LaunchConfiguration("gripper_max_force")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_internal_bus_gripper_comm = LaunchConfiguration("use_internal_bus_gripper_comm")

    launch_arguments = {
        # Real robot
        # "robot_ip": "192.168.100.20",
        # "use_fake_hardware": "false",
        # Fake robot
        # "robot_ip": "yyy.yyy.yyy.yyy",
        # "use_fake_hardware": "true",
        # Launch CLI defined arguments
        "robot_ip": robot_ip,
        "use_fake_hardware": use_fake_hardware,
        "gripper": "robotiq_2f_85",
        "gripper_joint_name": "robotiq_85_left_knuckle_joint",
        "dof": "7",
        "gripper_max_velocity": "100.0",
        "gripper_max_force": "0.0005",
        "use_internal_bus_gripper_comm": "true",
        "vision": "true",
    }
    
    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config")
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # .trajectory_execution(file_path="config/ros2_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    moveit_config.moveit_cpp.update({"use_sim_time": use_sim_time.perform(context) == "true"})

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("mtc_demo") + "/config/gen3_mtc.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("kinova_gen3_7dof_robotiq_2f_85_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    robot_pos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["twist_controller", "--inactive", "-c", "/controller_manager"],
    )

    robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
        # condition=IfCondition(use_internal_bus_gripper_comm),
    )

    fault_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fault_controller", "-c", "/controller_manager"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    nodes_to_start = [
        ros2_control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        robot_traj_controller_spawner,
        robot_pos_controller_spawner,
        robot_hand_controller_spawner,
        fault_controller_spawner,
        move_group_node,
        static_tf,
    ]

    return nodes_to_start


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="yyy.yyy.yyy.yyy",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_velocity",
            default_value="100.0",
            description="Max velocity for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_force",
            default_value="100.0",
            description="Max force for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_internal_bus_gripper_comm",
            default_value="true",
            description="Use arm's internal gripper connection",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_external_cable",
            default_value="false",
            description="Max force for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

