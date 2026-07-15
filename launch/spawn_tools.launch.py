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
    DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
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
        "spawn_delay", default_value="30.0",
        description="Max seconds to wait for each tool's robot_description "
                     "topic before giving up on spawning its Gazebo model "
                     "(readiness timeout, not a fixed delay)"
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
            # Status – Gazebo → ROS. The DetachableJoint system (gz-sim 8 /
            # Harmonic) publishes gz.msgs.StringMsg ("attached"/"detached") on
            # its output topic, not gz.msgs.Boolean — verified against the
            # actual plugin (libgz-sim-detachable-joint-system.so) in a
            # headless test world. The previous std_msgs/msg/Bool@Boolean
            # declaration never matched the publisher's type, so gz-transport
            # silently dropped every status message; ros_gz_bridge would log
            # a type mismatch and no message ever reached tool_manager.cpp's
            # status subscription.
            "/model/robot/detachable_joint/rg6_arm@std_msgs/msg/String@ignition.msgs.StringMsg",
            "/model/robot/detachable_joint/rg6_storage@std_msgs/msg/String@ignition.msgs.StringMsg",
            "/model/robot/detachable_joint/screwdriver_arm@std_msgs/msg/String@ignition.msgs.StringMsg",
            "/model/robot/detachable_joint/screwdriver_storage@std_msgs/msg/String@ignition.msgs.StringMsg",
            "/model/robot/detachable_joint/suction_array_arm@std_msgs/msg/String@ignition.msgs.StringMsg",
            "/model/robot/detachable_joint/suction_array_storage@std_msgs/msg/String@ignition.msgs.StringMsg",
            "/model/robot/detachable_joint/custom_tool_arm@std_msgs/msg/String@ignition.msgs.StringMsg",
            "/model/robot/detachable_joint/custom_tool_storage@std_msgs/msg/String@ignition.msgs.StringMsg",
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

    # Initial attach/detach (ensure all tools start locked in their clamps)
    # DetachableJoint starts detached by default; we must explicitly attach the
    # storage joints to hold each tool in its clamp, then detach the arm joints
    # so no tool is mounted on the arm at startup.
    #
    # RSP publishes robot_description -> spawn Gazebo model (create exits
    # once the entity exists) -> attach storage joint (`ros2 topic pub -t 5`
    # exits once its 5 publishes are sent) -> detach arm joint -> (rg6 only)
    # bring up its gripper controllers.

    def make_pub(topic):
        return ExecuteProcess(
            cmd=["ros2", "topic", "pub", "-t", "5", topic,
                 "std_msgs/msg/Empty", "{}"],
            output="screen",
        )

    def wait_for_rsp_topic(namespace):
        return ExecuteProcess(
            cmd=["wait_for_ros", "--timeout", LaunchConfiguration("spawn_delay"),
                 "topic", f"/{namespace}/robot_description", "--msg"],
            name=f"wait_for_{namespace}_robot_description",
            output="screen",
        )

    # ── rg6 gripper controller spawner ────────────────────────────────────
    # Runs against /rg6_tool/controller_manager (started by gz_ros2_control
    # inside the rg6_tool Gazebo model). `--controller-manager-timeout` lets
    # the spawner itself wait out the controller_manager service coming up,
    # instead of a fixed pre-delay.

    rg6_jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/rg6_tool/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # gz_ros2_control drops onrobot_rg_gripper_controller.type from the CM
    # params when the controller fails its auto-start (mimic joint
    # unsupported). We must re-set it once the controller_manager is up and
    # has attempted (and failed) that auto-start — retry instead of guessing
    # a fixed settle time, since the auto-start failure isn't independently
    # observable over ROS.
    rg6_gripper_type_setter = ExecuteProcess(
        cmd=["bash", "-c",
             "for i in $(seq 1 30); do "
             "ros2 param set /rg6_tool/controller_manager "
             "onrobot_rg_gripper_controller.type "
             "position_controllers/GripperActionController "
             "&& exit 0; sleep 0.5; done; "
             "echo 'rg6_gripper_type_setter: giving up after 15s' >&2; exit 1"],
        name="rg6_gripper_type_setter",
        output="screen",
    )

    rg6_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "onrobot_rg_gripper_controller",
            "--controller-manager", "/rg6_tool/controller_manager",
            "--controller-manager-timeout", "60",
            "-p", os.path.join(pkg_campetella, "models", "rg6_tool", "config", "rg6_controllers.yaml"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # ── tool_manager ──────────────────────────────────────────────────────
    # Starts immediately: service/topic setup doesn't require Gazebo to be up
    # yet. Verified attach/detach behavior (waiting for bridge subscribers,
    # confirming status instead of assuming success) lives in tool_manager.cpp
    # itself.

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

    # ── Per-tool event chains ───────────────────────────────────────────────

    wait_rg6_rsp = wait_for_rsp_topic("rg6_tool")
    rg6_storage_attach = make_pub("/model/robot/detachable_joint/rg6_storage/attach")
    rg6_arm_detach = make_pub("/model/robot/detachable_joint/rg6_arm/detach")

    wait_screwdriver_rsp = wait_for_rsp_topic("sd_35_screwdriver")
    screwdriver_storage_attach = make_pub("/model/robot/detachable_joint/screwdriver_storage/attach")
    screwdriver_arm_detach = make_pub("/model/robot/detachable_joint/screwdriver_arm/detach")

    wait_suction_rsp = wait_for_rsp_topic("suction_array")
    suction_storage_attach = make_pub("/model/robot/detachable_joint/suction_array_storage/attach")
    suction_arm_detach = make_pub("/model/robot/detachable_joint/suction_array_arm/detach")

    wait_custom_rsp = wait_for_rsp_topic("custom_tool")
    custom_storage_attach = make_pub("/model/robot/detachable_joint/custom_tool_storage/attach")
    custom_arm_detach = make_pub("/model/robot/detachable_joint/custom_tool_arm/detach")

    # Tool chains run strictly one after another (rg6 -> screwdriver ->
    # suction_array -> custom_tool) instead of all four in parallel. Spawning
    # all four at once was slamming CycloneDDS with a burst of concurrent
    # short-lived participants (create, ros2 topic pub, wait_for_ros for all
    # 4 tools nearly simultaneously), which randomly exhausted CycloneDDS's
    # participant-index pool ("Failed to find a free participant index for
    # domain 0") and aborted whichever tool's `create` process lost the race
    # — observed as suction_array/screwdriver silently failing to spawn while
    # rg6/custom_tool happened to get a slot. Serializing keeps at most one
    # tool's worth of transient nodes alive at a time.
    tool_chain_events = [
        RegisterEventHandler(OnProcessExit(target_action=wait_rg6_rsp, on_exit=[rg6_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=rg6_spawner, on_exit=[rg6_storage_attach])),
        RegisterEventHandler(OnProcessExit(target_action=rg6_storage_attach, on_exit=[rg6_arm_detach])),
        RegisterEventHandler(OnProcessExit(target_action=rg6_arm_detach, on_exit=[rg6_jsb_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=rg6_jsb_spawner, on_exit=[rg6_gripper_type_setter])),
        RegisterEventHandler(OnProcessExit(target_action=rg6_gripper_type_setter, on_exit=[rg6_gripper_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=rg6_gripper_spawner, on_exit=[wait_screwdriver_rsp])),

        RegisterEventHandler(OnProcessExit(target_action=wait_screwdriver_rsp, on_exit=[screwdriver_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=screwdriver_spawner, on_exit=[screwdriver_storage_attach])),
        RegisterEventHandler(OnProcessExit(target_action=screwdriver_storage_attach, on_exit=[screwdriver_arm_detach])),
        RegisterEventHandler(OnProcessExit(target_action=screwdriver_arm_detach, on_exit=[wait_suction_rsp])),

        RegisterEventHandler(OnProcessExit(target_action=wait_suction_rsp, on_exit=[suction_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=suction_spawner, on_exit=[suction_storage_attach])),
        RegisterEventHandler(OnProcessExit(target_action=suction_storage_attach, on_exit=[suction_arm_detach])),
        RegisterEventHandler(OnProcessExit(target_action=suction_arm_detach, on_exit=[wait_custom_rsp])),

        RegisterEventHandler(OnProcessExit(target_action=wait_custom_rsp, on_exit=[custom_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=custom_spawner, on_exit=[custom_storage_attach])),
        RegisterEventHandler(OnProcessExit(target_action=custom_storage_attach, on_exit=[custom_arm_detach])),
    ]

    return LaunchDescription([
        # ── Arguments ──
        use_sim_time_arg, robot_id_arg, spawn_delay_arg,
        rg6_x_arg, rg6_y_arg, rg6_z_arg, rg6_yaw_arg,
        screwdriver_x_arg, screwdriver_y_arg, screwdriver_z_arg, screwdriver_yaw_arg,
        suction_x_arg, suction_y_arg, suction_z_arg, suction_yaw_arg,
        custom_x_arg, custom_y_arg, custom_z_arg, custom_yaw_arg,

        # ── RSPs, bridge, and tool_manager start immediately ──
        rg6_rsp,
        screwdriver_rsp,
        suction_rsp,
        custom_rsp,
        bridge,
        tool_manager_node,

        # ── Tool chains run one at a time; only the first waiter starts
        #    immediately, the rest are triggered at the end of the previous
        #    tool's chain (see tool_chain_events above) ──
        wait_rg6_rsp,

        *tool_chain_events,
    ])
