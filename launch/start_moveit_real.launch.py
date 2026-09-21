"""Start ur_robot_driver, MoveIt and RViz for the real RB-VOGUI+ UR5e."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    reverse_ip = LaunchConfiguration('reverse_ip')
    calibration_file = LaunchConfiguration('kinematics_params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    wrist_camera = LaunchConfiguration('wrist_camera')
    script_sender_port = LaunchConfiguration('script_sender_port')

    declared_arguments = [
        DeclareLaunchArgument(
            'script_sender_port',
            default_value='50002',
            description='Puerto configurado en el URCap External Control',
        ),
        DeclareLaunchArgument('robot_ip'),
        DeclareLaunchArgument('reverse_ip'),
        DeclareLaunchArgument('kinematics_params_file'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'wrist_camera',
            default_value='stereolabs_zed2i',
            choices=['stereolabs_zed2i', 'realsense_d435i', 'none'],
        ),
    ]

    ur_driver_share = FindPackageShare('ur_robot_driver')
    moveit_share = FindPackageShare('renee_rbvogui_plus_moveit_config')

    moveit_config = (
        MoveItConfigsBuilder(
            'renee_rbvogui_plus',
            package_name='renee_rbvogui_plus_moveit_config',
        )
        .robot_description(
            file_path='config/rbvogui_plus_real.urdf.xacro',
            mappings={
                'kinematics_params': calibration_file,
                'wrist_camera': wrist_camera,
            },
        )
        .robot_description_semantic(file_path='config/rbvogui_plus.srdf')
        .trajectory_execution(file_path='config/moveit_controllers_real.yaml')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .planning_pipelines(
            pipelines=['chomp', 'ompl', 'pilz_industrial_motion_planner'],
            default_planning_pipeline='pilz_industrial_motion_planner',
        )
        .to_moveit_configs()
    )

    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ur_driver_share, 'launch', 'ur_control.launch.py'])
        ),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': robot_ip,
            'reverse_ip': reverse_ip,
            'kinematics_params_file': calibration_file,
            'tf_prefix': 'robot_arm_',
            'script_sender_port': script_sender_port,
            'description_launchfile': PathJoinSubstitution([
                moveit_share, 'launch', 'real_rsp.launch.py',
            ]),
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
            'activate_joint_controller': 'true',
            'headless_mode': 'false',
            'launch_rviz': 'false',
            'use_mock_hardware': 'false',
        }.items(),
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace='robot',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False},
            {'trajectory_execution.allowed_start_tolerance': 0.05},
            {'trajectory_execution.allowed_execution_duration_scaling': 25.0},
            {'trajectory_execution.allowed_goal_duration_margin': 2.0},
            {'trajectory_execution.execution_duration_monitoring': True},
        ],
        remappings=[('joint_states', '/joint_states')],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=[
            '-d',
            PathJoinSubstitution([moveit_share, 'config', 'moveit.rviz']),
        ],
        parameters=[moveit_config.to_dict(), {'use_sim_time': False}],
    )

    return LaunchDescription([
        *declared_arguments,
        ur_driver,
        move_group,
        rviz,
    ])
