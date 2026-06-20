#!/usr/bin/env python3
"""
spawn_tools.launch.py
Spawns all four detachable tools for the rbvogui_plus tool-changer scheme:
  - rg6         (front_left  clamp, actuated gripper)
  - screwdriver (front_right clamp, passive)
  - suction_array (rear_left  clamp, passive)
  - custom_tool   (rear_right clamp, passive)

For each tool it:
  1. Starts a robot_state_publisher (own namespace / robot_description topic)
  2. Spawns the Gazebo model at its clamp world pose
  3. Bridges all attach/detach/status and pose→/tf topics

Tool spawn positions (x, y, z, Y) are arguments so they can be tuned to
match the actual TC clamp world coordinates after the robot spawns.

Usage (after the main robot is up):
  ros2 launch renee_rbvogui_plus_moveit_config spawn_tools.launch.py
"""

import os
import launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, ExecuteProcess
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def create_tool_rsp(namespace, xacro_path, use_sim_time):
    """Create a robot_state_publisher node for a standalone tool model."""
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", xacro_path]),
                value_type=str,
            ),
            "publish_robot_description": True,
            "use_sim_time": use_sim_time,
        }],
        remappings=[
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
        ],
    )


def create_tool_spawner(name, topic, x, y, z, yaw, pitch="0", roll="0",
                        use_sim_time=True):
    """Spawn a Gazebo tool model from its robot_description topic."""
    args = [
        "-name", name,
        "-topic", topic,
        "-x", str(x), "-y", str(y), "-z", str(z),
        "-Y", str(yaw),
    ]
    if pitch != "0":
        args += ["-P", str(pitch)]
    if roll != "0":
        args += ["-R", str(roll)]
    return Node(
        package="ros_gz_sim",
        executable="create",
        arguments=args,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )


def generate_launch_description():

    pkg_campetella = get_package_share_directory("campetella_sim")
    pkg_moveit = get_package_share_directory("renee_rbvogui_plus_moveit_config")

    # ── Arguments ──────────────────────────────────────────────────────────

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    robot_id_arg = DeclareLaunchArgument(
        "robot_id", default_value="robot"
    )
    spawn_delay_arg = DeclareLaunchArgument(
        "spawn_delay", default_value="5.0",
        description="Seconds to wait after RSPs before spawning Gazebo models"
    )

    # Tool spawn positions — tune these to match TC clamp world coordinates
    # after the robot is placed.  Values below are placeholders matching
    # a robot at x=2.5, y=2.5 (demo_spawn.sh default) with arm raised.
    rg6_x_arg          = DeclareLaunchArgument("rg6_x",          default_value="2.425")
    rg6_y_arg          = DeclareLaunchArgument("rg6_y",          default_value="2.89")
    rg6_z_arg          = DeclareLaunchArgument("rg6_z",          default_value="0.75")
    rg6_yaw_arg        = DeclareLaunchArgument("rg6_yaw",        default_value="1.57")

    screwdriver_x_arg  = DeclareLaunchArgument("screwdriver_x",  default_value="2.425")
    screwdriver_y_arg  = DeclareLaunchArgument("screwdriver_y",  default_value="2.11")
    screwdriver_z_arg  = DeclareLaunchArgument("screwdriver_z",  default_value="0.75")
    screwdriver_yaw_arg = DeclareLaunchArgument("screwdriver_yaw", default_value="1.57")

    suction_x_arg      = DeclareLaunchArgument("suction_array_x",   default_value="2.215")
    suction_y_arg      = DeclareLaunchArgument("suction_array_y",   default_value="2.89")
    suction_z_arg      = DeclareLaunchArgument("suction_array_z",   default_value="0.75")
    suction_yaw_arg    = DeclareLaunchArgument("suction_array_yaw", default_value="-1.57")

    custom_x_arg       = DeclareLaunchArgument("custom_tool_x",   default_value="2.215")
    custom_y_arg       = DeclareLaunchArgument("custom_tool_y",   default_value="2.11")
    custom_z_arg       = DeclareLaunchArgument("custom_tool_z",   default_value="0.75")
    custom_yaw_arg     = DeclareLaunchArgument("custom_tool_yaw", default_value="0.0")

    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_id     = LaunchConfiguration("robot_id")

    # ── Robot State Publishers ─────────────────────────────────────────────

    rg6_rsp = create_tool_rsp(
        namespace="rg6_tool",
        xacro_path=os.path.join(pkg_campetella, "models", "rg6_tool", "urdf", "rg6_tool.urdf.xacro"),
        use_sim_time=use_sim_time,
    )

    screwdriver_rsp = create_tool_rsp(
        namespace="sd_35_screwdriver",
        xacro_path=os.path.join(pkg_campetella, "models", "sd_35", "urdf", "sd_35.urdf.xacro"),
        use_sim_time=use_sim_time,
    )

    suction_rsp = create_tool_rsp(
        namespace="suction_array",
        xacro_path=os.path.join(pkg_campetella, "models", "suction_array", "urdf",
                                "suction_array.urdf.xacro"),
        use_sim_time=use_sim_time,
    )

    custom_rsp = create_tool_rsp(
        namespace="custom_tool",
        xacro_path=os.path.join(pkg_campetella, "models", "custom_tool", "urdf",
                                "custom_tool.urdf.xacro"),
        use_sim_time=use_sim_time,
    )

    # ── Gazebo Spawners ────────────────────────────────────────────────────

    rg6_spawner = Node(
        package="ros_gz_sim", executable="create",
        arguments=[
            "-name", "rg6_tool",
            "-topic", "/rg6_tool/robot_description",
            "-x", LaunchConfiguration("rg6_x"),
            "-y", LaunchConfiguration("rg6_y"),
            "-z", LaunchConfiguration("rg6_z"),
            "-Y", LaunchConfiguration("rg6_yaw"),
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    screwdriver_spawner = Node(
        package="ros_gz_sim", executable="create",
        arguments=[
            "-name", "sd_35_screwdriver",
            "-topic", "/sd_35_screwdriver/robot_description",
            "-x", LaunchConfiguration("screwdriver_x"),
            "-y", LaunchConfiguration("screwdriver_y"),
            "-z", LaunchConfiguration("screwdriver_z"),
            "-Y", LaunchConfiguration("screwdriver_yaw"),
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    suction_spawner = Node(
        package="ros_gz_sim", executable="create",
        arguments=[
            "-name", "suction_array",
            "-topic", "/suction_array/robot_description",
            "-x", LaunchConfiguration("suction_array_x"),
            "-y", LaunchConfiguration("suction_array_y"),
            "-z", LaunchConfiguration("suction_array_z"),
            "-Y", LaunchConfiguration("suction_array_yaw"),
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    custom_spawner = Node(
        package="ros_gz_sim", executable="create",
        arguments=[
            "-name", "custom_tool",
            "-topic", "/custom_tool/robot_description",
            "-x", LaunchConfiguration("custom_tool_x"),
            "-y", LaunchConfiguration("custom_tool_y"),
            "-z", LaunchConfiguration("custom_tool_z"),
            "-Y", LaunchConfiguration("custom_tool_yaw"),
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── ros_gz_bridge ──────────────────────────────────────────────────────
    # Bridges detachable-joint status/attach/detach for all 4 tools
    # plus tool pose → /tf so MoveIt and TF tree see them.

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Pose → /tf for each tool
            "/model/rg6_tool/pose@geometry_msgs/msg/TFMessage[ignition.msgs.Pose",
            "/model/sd_35_screwdriver/pose@geometry_msgs/msg/TFMessage[ignition.msgs.Pose",
            "/model/suction_array/pose@geometry_msgs/msg/TFMessage[ignition.msgs.Pose",
            "/model/custom_tool/pose@geometry_msgs/msg/TFMessage[ignition.msgs.Pose",
            # Status (Bool) – Gazebo → ROS
            "/model/robot/detachable_joint/rg6_arm@std_msgs/msg/Bool@ignition.msgs.Boolean",
            "/model/robot/detachable_joint/rg6_storage@std_msgs/msg/Bool@ignition.msgs.Boolean",
            "/model/robot/detachable_joint/screwdriver_arm@std_msgs/msg/Bool@ignition.msgs.Boolean",
            "/model/robot/detachable_joint/screwdriver_storage@std_msgs/msg/Bool@ignition.msgs.Boolean",
            "/model/robot/detachable_joint/suction_array_arm@std_msgs/msg/Bool@ignition.msgs.Boolean",
            "/model/robot/detachable_joint/suction_array_storage@std_msgs/msg/Bool@ignition.msgs.Boolean",
            "/model/robot/detachable_joint/custom_tool_arm@std_msgs/msg/Bool@ignition.msgs.Boolean",
            "/model/robot/detachable_joint/custom_tool_storage@std_msgs/msg/Bool@ignition.msgs.Boolean",
            # Attach/detach (Empty) – ROS → Gazebo
            "/model/robot/detachable_joint/rg6_arm/attach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/model/robot/detachable_joint/rg6_arm/detach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/model/robot/detachable_joint/rg6_storage/attach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/model/robot/detachable_joint/rg6_storage/detach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/model/robot/detachable_joint/screwdriver_arm/attach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/model/robot/detachable_joint/screwdriver_arm/detach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/model/robot/detachable_joint/screwdriver_storage/attach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/model/robot/detachable_joint/screwdriver_storage/detach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/model/robot/detachable_joint/suction_array_arm/attach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/model/robot/detachable_joint/suction_array_arm/detach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/model/robot/detachable_joint/suction_array_storage/attach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/model/robot/detachable_joint/suction_array_storage/detach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/model/robot/detachable_joint/custom_tool_arm/attach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/model/robot/detachable_joint/custom_tool_arm/detach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/model/robot/detachable_joint/custom_tool_storage/attach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/model/robot/detachable_joint/custom_tool_storage/detach@std_msgs/msg/Empty]ignition.msgs.Empty",
        ],
        remappings=[
            ("/model/rg6_tool/pose",          "/tf"),
            ("/model/sd_35_screwdriver/pose", "/tf"),
            ("/model/suction_array/pose",     "/tf"),
            ("/model/custom_tool/pose",       "/tf"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # ── Initial attach/detach (ensure all tools start locked in their clamps) ──
    # DetachableJoint starts detached by default; we must explicitly attach the
    # storage joints to hold each tool in its clamp, then detach the arm joints
    # so no tool is mounted on the arm at startup.

    def make_pub(topic):
        return ExecuteProcess(
            cmd=["ros2", "topic", "pub", "-t", "1", topic,
                 "std_msgs/msg/Empty", "{}"],
            output="screen",
        )

    initial_storage_attach_actions = [
        make_pub("/model/robot/detachable_joint/rg6_storage/attach"),
        make_pub("/model/robot/detachable_joint/screwdriver_storage/attach"),
        make_pub("/model/robot/detachable_joint/suction_array_storage/attach"),
        make_pub("/model/robot/detachable_joint/custom_tool_storage/attach"),
    ]

    initial_detach_actions = [
        make_pub("/model/robot/detachable_joint/rg6_arm/detach"),
        make_pub("/model/robot/detachable_joint/screwdriver_arm/detach"),
        make_pub("/model/robot/detachable_joint/suction_array_arm/detach"),
        make_pub("/model/robot/detachable_joint/custom_tool_arm/detach"),
    ]

    # ── rg6 gripper controller spawner ────────────────────────────────────
    # Runs against /rg6_tool/controller_manager (started by gz_ros2_control
    # inside the rg6_tool Gazebo model).

    rg6_jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/rg6_tool/controller_manager",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # gz_ros2_control drops onrobot_rg_gripper_controller.type from the CM params
    # when the controller fails its auto-start (mimic joint unsupported).
    # We re-set it at t=25s — well after the model-spawn DDS burst at t=5-12s
    # clears (spawner/detach-pub nodes exit), so ros2 param set has a free slot.
    # The actual spawner fires at t=28s to let the param settle.
    rg6_gripper_type_setter = ExecuteProcess(
        cmd=[
            "ros2", "param", "set",
            "/rg6_tool/controller_manager",
            "onrobot_rg_gripper_controller.type",
            "position_controllers/GripperActionController",
        ],
        output="screen",
    )

    rg6_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "onrobot_rg_gripper_controller",
            "--controller-manager", "/rg6_tool/controller_manager",
            "-p", os.path.join(pkg_campetella, "models", "rg6_tool", "config", "rg6_controllers.yaml"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # ── tool_manager ──────────────────────────────────────────────────────

    tool_manager_node = Node(
        package="campetella_sim",
        executable="tool_manager",
        name="tool_manager",
        parameters=[
            os.path.join(pkg_moveit, "config", "tool_manager_tools.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    return LaunchDescription([
        # ── Arguments ──
        use_sim_time_arg, robot_id_arg, spawn_delay_arg,
        rg6_x_arg, rg6_y_arg, rg6_z_arg, rg6_yaw_arg,
        screwdriver_x_arg, screwdriver_y_arg, screwdriver_z_arg, screwdriver_yaw_arg,
        suction_x_arg, suction_y_arg, suction_z_arg, suction_yaw_arg,
        custom_x_arg, custom_y_arg, custom_z_arg, custom_yaw_arg,

        # ── RSPs start immediately ──
        rg6_rsp,
        screwdriver_rsp,
        suction_rsp,
        custom_rsp,
        bridge,

        # ── Gazebo spawns after RSPs are up ──
        TimerAction(period=LaunchConfiguration("spawn_delay"), actions=[
            rg6_spawner,
            screwdriver_spawner,
            suction_spawner,
            custom_spawner,
        ]),

        # ── Attach storage joints so tools are held in their clamps ──
        TimerAction(period=7.0, actions=initial_storage_attach_actions),

        # ── Detach arm joints (no tool on arm at startup) ──
        TimerAction(period=10.0, actions=initial_detach_actions),

        # ── rg6 controllers (after Gazebo model is up) ──
        TimerAction(period=12.0, actions=[rg6_jsb_spawner]),
        # Set type after DDS burst from model-spawn phase settles (~25s)
        TimerAction(period=25.0, actions=[rg6_gripper_type_setter]),
        # Spawn 3s later so the CM has ingested the type param
        TimerAction(period=28.0, actions=[rg6_gripper_spawner]),

        # ── tool_manager ──
        TimerAction(period=8.0, actions=[tool_manager_node]),
    ])
