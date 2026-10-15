from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        ('motion_mode', 'circle', str, 'Target motion mode: circle, sinusoidal, random_walk'),
        ('max_range', '10.0', float, 'Boundary radius for non-circular motion in metres'),
        ('circle_radius', '5.0', float, 'Circle radius in local NED metres'),
        ('circle_angular_velocity', '0.5', float, 'Signed circle angular velocity in rad/s'),
        ('circle_initial_phase', '0.0', float, 'Initial circle phase in radians'),
        ('target_altitude', '-5.0', float, 'Local NED down position in metres'),
        ('defer_motion', 'false', bool, 'Hover until stable and externally scheduled'),
        ('takeoff_position_tolerance', '0.1', float, '3D hover position tolerance in metres'),
        ('takeoff_speed_tolerance', '0.2', float, '3D hover speed tolerance in m/s'),
        ('stability_duration', '2.0', float, 'Continuous stable hover duration in seconds'),
    ]
    declarations = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, _, description in arguments
    ]

    target_node = Node(
        package='uav_target_sim',
        executable='uav_target_sim',
        name='uav_target_sim',
        output='screen',
        parameters=[{
            name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
            for name, _, value_type, _ in arguments
        }],
    )

    return LaunchDescription(declarations + [target_node])
