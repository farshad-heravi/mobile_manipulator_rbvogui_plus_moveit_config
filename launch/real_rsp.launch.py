"""Publish the fixed UR5e subtree used by the real RB-VOGUI+ driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    argument_names = [
        'robot_ip', 'joint_limit_params_file', 'kinematics_params_file',
        'physical_params_file', 'visual_params_file',
        'headless_mode', 'use_mock_hardware', 'mock_sensor_commands',
        'use_tool_communication', 'tool_voltage', 'tool_parity', 'tool_baud_rate',
        'tool_stop_bits', 'tool_rx_idle_chars', 'tool_tx_idle_chars',
        'tool_device_name', 'tool_tcp_port', 'reverse_ip', 'script_command_port',
        'reverse_port', 'script_sender_port', 'trajectory_port', 'wrist_camera',
    ]

    defaults = {
        'robot_ip': '',
        'joint_limit_params_file': PathJoinSubstitution([
            FindPackageShare('ur_description'), 'config', 'ur5e', 'joint_limits.yaml',
        ]),
        'kinematics_params_file': PathJoinSubstitution([
            FindPackageShare('ur_description'), 'config', 'ur5e', 'default_kinematics.yaml',
        ]),
        'physical_params_file': PathJoinSubstitution([
            FindPackageShare('ur_description'), 'config', 'ur5e', 'physical_parameters.yaml',
        ]),
        'visual_params_file': PathJoinSubstitution([
            FindPackageShare('ur_description'), 'config', 'ur5e', 'visual_parameters.yaml',
        ]),
        'headless_mode': 'false',
        'use_mock_hardware': 'false',
        'mock_sensor_commands': 'false',
        'use_tool_communication': 'false',
        'tool_voltage': '0',
        'tool_parity': '0',
        'tool_baud_rate': '115200',
        'tool_stop_bits': '1',
        'tool_rx_idle_chars': '1.5',
        'tool_tx_idle_chars': '3.5',
        'tool_device_name': '/tmp/ttyUR',
        'tool_tcp_port': '54321',
        'reverse_ip': '0.0.0.0',
        'script_command_port': '50004',
        'reverse_port': '50001',
        'script_sender_port': '50002',
        'trajectory_port': '50003',
        'wrist_camera': 'stereolabs_zed2i',
    }
    declared_arguments = [
        DeclareLaunchArgument(name, default_value=defaults[name])
        if name in defaults else DeclareLaunchArgument(name)
        for name in argument_names
    ]

    script_filename = PathJoinSubstitution([
        FindPackageShare('ur_client_library'), 'resources', 'external_control.urscript',
    ])
    input_recipe_filename = PathJoinSubstitution([
        FindPackageShare('ur_robot_driver'), 'resources', 'rtde_input_recipe.txt',
    ])
    output_recipe_filename = PathJoinSubstitution([
        FindPackageShare('ur_robot_driver'), 'resources', 'rtde_output_recipe.txt',
    ])
    description_file = PathJoinSubstitution([
        FindPackageShare('renee_rbvogui_plus_moveit_config'),
        'config', 'rbvogui_plus_real_arm.urdf.xacro',
    ])
    xacro_command = Command([
        FindExecutable(name='xacro'), ' ', description_file,
        ' robot_ip:=', LaunchConfiguration('robot_ip'),
        ' joint_limit_params:=', LaunchConfiguration('joint_limit_params_file'),
        ' kinematics_params:=', LaunchConfiguration('kinematics_params_file'),
        ' physical_params:=', LaunchConfiguration('physical_params_file'),
        ' visual_params:=', LaunchConfiguration('visual_params_file'),
        ' headless_mode:=', LaunchConfiguration('headless_mode'),
        ' script_filename:=', script_filename,
        ' input_recipe_filename:=', input_recipe_filename,
        ' output_recipe_filename:=', output_recipe_filename,
        ' use_mock_hardware:=', LaunchConfiguration('use_mock_hardware'),
        ' mock_sensor_commands:=', LaunchConfiguration('mock_sensor_commands'),
        ' use_tool_communication:=', LaunchConfiguration('use_tool_communication'),
        ' tool_voltage:=', LaunchConfiguration('tool_voltage'),
        ' tool_parity:=', LaunchConfiguration('tool_parity'),
        ' tool_baud_rate:=', LaunchConfiguration('tool_baud_rate'),
        ' tool_stop_bits:=', LaunchConfiguration('tool_stop_bits'),
        ' tool_rx_idle_chars:=', LaunchConfiguration('tool_rx_idle_chars'),
        ' tool_tx_idle_chars:=', LaunchConfiguration('tool_tx_idle_chars'),
        ' tool_device_name:=', LaunchConfiguration('tool_device_name'),
        ' tool_tcp_port:=', LaunchConfiguration('tool_tcp_port'),
        ' reverse_ip:=', LaunchConfiguration('reverse_ip'),
        ' script_command_port:=', LaunchConfiguration('script_command_port'),
        ' reverse_port:=', LaunchConfiguration('reverse_port'),
        ' script_sender_port:=', LaunchConfiguration('script_sender_port'),
        ' trajectory_port:=', LaunchConfiguration('trajectory_port'),
        ' wrist_camera:=', LaunchConfiguration('wrist_camera'),
    ])

    robot_description = {
        'robot_description': ParameterValue(xacro_command, value_type=str),
    }
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': False}],
    )

    return LaunchDescription(declared_arguments + [rsp])
