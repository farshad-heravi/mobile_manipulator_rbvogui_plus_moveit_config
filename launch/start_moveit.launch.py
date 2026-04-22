import os
from launch import LaunchDescription
from launch_param_builder import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Use rviz',
        ),
        DeclareLaunchArgument(
            'end_effector',
            default_value='rg6',
            choices=['rg6', '2f85', 'screwdriver', 'none'],
            description='End effector to use',
        ),
    ]

    def launch_setup(context, *args, **kwargs):
        use_sim_time = context.launch_configurations.get('use_sim_time', False)
        use_rviz = context.launch_configurations.get('use_rviz', True)
        use_sim_time_bool = use_sim_time=='true'
        end_effector = context.launch_configurations.get('end_effector', 'rg6')
        
        moveit_config_file = "config/moveit_controllers.yaml"
        joint_controllers_file = os.path.join( get_package_share_directory('renee_rbvogui_plus_moveit_config'), 'config', 'ros2_controllers.yaml' )
        log1 = LogInfo( msg=['Loading Controllers for Simulation'] )

        rviz_config = PathJoinSubstitution(
            [FindPackageShare("renee_rbvogui_plus_moveit_config"), "config", "moveit.rviz"] # TODO
        ) 

        # moveit config
        moveit_config = (
            MoveItConfigsBuilder("renee_rbvogui_plus", package_name="renee_rbvogui_plus_moveit_config")
            .robot_description(file_path="config/rbvogui_plus.urdf.xacro",
                mappings={
                    "use_sim_time": use_sim_time,
                    "use_rviz": use_rviz,
                    "end_effector": end_effector,
                    "prefix": "robot_",
                })
            .robot_description_semantic(file_path="config/rbvogui_plus.srdf")
            .trajectory_execution(file_path=moveit_config_file)
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .planning_scene_monitor(
                publish_robot_description= True, publish_robot_description_semantic=True, publish_planning_scene=True
            )
            .planning_pipelines(
                pipelines=["chomp", "ompl", "pilz_industrial_motion_planner"],
                default_planning_pipeline="pilz_industrial_motion_planner"
            )
            .to_moveit_configs()
        )

        # Robot State Publisher - NOT started here when using Gazebo,
        # because Gazebo already provides /robot/robot_state_publisher.
        # robot_state_publisher_node = Node(
        #     package="robot_state_publisher",
        #     executable="robot_state_publisher",
        #     name="robot_state_publisher",
        #     parameters=[moveit_config.robot_description, {'use_sim_time': use_sim_time_bool}],
        #     remappings=[
        #         ('joint_states', '/robot/joint_states'),
        #     ],
        #     output="screen"
        # )

        # include ur_robot_bringup.py
        # ur_robot_bringup_node = IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource(
        #                 os.path.join(get_package_share_directory('fnh_rbvogui_moveit_config'), 'launch', 'ur_robot_bringup.py')
        #             ),
        #             launch_arguments={
        #                 'ur_type': 'ur5e',
        #                 'robot_ip': "1.1.1.1",
        #                 'launch_rviz': 'false',
        #                 'robot_state_pub_node': 'false',
        #             }.items(),
        #             condition=IfCondition(real_robot),
        #         )

        # # Controller Manager - only for simulation (fake hardware)
        # controller_manager_node = Node(
        #     package="controller_manager",
        #     executable="ros2_control_node",
        #     parameters=[
        #         moveit_config.robot_description,
        #         joint_controllers_file,
        #         {'use_sim_time': use_sim_time_bool},
        #     ],
        #     remappings=[
        #         ('robot_description', '/robot/robot_description'),
        #         # ('/joint_states', '/ur_internal/joint_states'),
        #     ],
        #     output="screen",
        #     # condition=IfCondition(use_fake_hardware)
        # )
        # NOTE: When running with Gazebo, the controller_manager is provided by
        # gz_ros2_control inside Gazebo at /robot/controller_manager.
        # Do NOT run a standalone ros2_control_node — it cannot load GazeboSimSystem.

        # Spawner for real robot controller - starts scaled_joint_trajectory_controller from UR driver
        # The UR driver loads this controller but doesn't start it by default
        # scaled_joint_trajectory_controller_spawner = Node(
        #     package="controller_manager",
        #     executable="spawner",
        #     arguments=["scaled_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        #     output="screen",
        #     condition=IfCondition(real_robot)
        # )

        # # Spawners for simulation controllers - only when using fake hardware
        # ur5_arm_controller_spawner = Node(
        #     package="controller_manager",
        #     executable="spawner",
        #     arguments=["ur5_arm_controller", "--controller-manager", "/robot/controller_manager"],
        #     output="screen",
        #     # condition=IfCondition(use_fake_hardware)
        # )

        # # Spawner for joint state broadcaster - only for simulation (fake hardware)
        # joint_state_broadcaster_spawner = Node(
        #     package="controller_manager",
        #     executable="spawner",
        #     arguments=["joint_state_broadcaster", "--controller-manager", "/robot/controller_manager"],
        #     output="screen",
        #     # remappings=[('/joint_states', '/ur_internal/joint_states')],  # real robot only
        #     # condition=IfCondition(use_fake_hardware)
        # )
        # NOTE: Gazebo already starts `joint_state_broadcaster` and `arm_controller`
        # via gz_ros2_control at /robot/controller_manager. No spawners needed here.

        gripper_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "onrobot_rg_gripper_controller",
                "--controller-manager", "/robot/controller_manager",
                "--param-file", os.path.join(
                    get_package_share_directory('renee_rbvogui_plus_moveit_config'),
                    'config', 'gripper_controller_params.yaml'
                ),
                "--namespace", "robot",
            ],
            output="screen",
        )

        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            namespace="robot",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': use_sim_time_bool},
                {"trajectory_execution.allowed_start_tolerance": 0.05},
                {"trajectory_execution.allowed_goal_duration_margin": 0.5},
                {"log_level": "DEBUG"},
                # Warehouse settings for plan caching (SQLite - no server required)
                # {"warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection"},
                # {"warehouse_host": warehouse_db_path},
            ],
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            # rviz2 intentionally NOT namespaced: the MotionPlanning plugin builds
            # service names as "{move_group_namespace}/get_planning_scene" etc.
            # If rviz were in /robot, "robot/get_planning_scene" → /robot/robot/...
            # From root namespace → /robot/get_planning_scene ✓
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[
                moveit_config.to_dict(),
                # Warehouse settings for RViz Motion Planning plugin (SQLite)
                # {"warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection"},
                # {"warehouse_host": warehouse_db_path},
                {'use_sim_time': use_sim_time_bool},
            ],
        )

        timer_action = TimerAction(
            period=8.0,
            actions=[move_group_node, rviz_node]
        )

        return [
            log1,
            gripper_controller_spawner,
            timer_action,
        ]

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])