from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description_wall_follower():
    # Common args
    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_topic = LaunchConfiguration("scan_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    # Behavior
    side = LaunchConfiguration("side")
    desired_distance = LaunchConfiguration("desired_distance")
    forward_speed = LaunchConfiguration("forward_speed")
    forward_angle_deg = LaunchConfiguration("forward_angle_deg")

    # Wall estimation
    theta_deg = LaunchConfiguration("theta_deg")
    lookahead = LaunchConfiguration("lookahead")

    # Lateral PID
    kp_lat = LaunchConfiguration("kp_lat")
    ki_lat = LaunchConfiguration("ki_lat")
    kd_lat = LaunchConfiguration("kd_lat")
    integral_limit = LaunchConfiguration("integral_limit")

    # Angle alignment
    k_angle = LaunchConfiguration("k_angle")

    # Limits
    max_vx = LaunchConfiguration("max_vx")
    max_vy = LaunchConfiguration("max_vy")
    max_wz = LaunchConfiguration("max_wz")

    # Safety
    min_front_dist = LaunchConfiguration("min_front_dist")
    front_window_deg = LaunchConfiguration("front_window_deg")

    # Scan robustness + loop
    sample_window = LaunchConfiguration("sample_window")
    control_rate_hz = LaunchConfiguration("control_rate_hz")

    wall_follower_node = Node(
        package="wall_follow",
        executable="wall_follower",
        name="wall_follower",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "scan_topic": scan_topic,
            "cmd_vel_topic": cmd_vel_topic,

            "side": side,
            "desired_distance": desired_distance,
            "forward_speed": forward_speed,
            "forward_angle_deg": forward_angle_deg,

            "theta_deg": theta_deg,
            "lookahead": lookahead,

            "kp_lat": kp_lat,
            "ki_lat": ki_lat,
            "kd_lat": kd_lat,
            "integral_limit": integral_limit,

            "k_angle": k_angle,

            "max_vx": max_vx,
            "max_vy": max_vy,
            "max_wz": max_wz,

            "min_front_dist": min_front_dist,
            "front_window_deg": front_window_deg,

            "sample_window": sample_window,
            "control_rate_hz": control_rate_hz,
        }],
        remappings=[
            ("/scan", scan_topic),
            ("/cmd_vel", cmd_vel_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),

        DeclareLaunchArgument("side", default_value="right"),              # right | left
        DeclareLaunchArgument("desired_distance", default_value="0.05"),   # [m]
        DeclareLaunchArgument("forward_speed", default_value="0.05"),      # [m/s]
        DeclareLaunchArgument("forward_angle_deg", default_value="0.0"),   # [deg] (180 pokud je lidar otočený)

        DeclareLaunchArgument("theta_deg", default_value="30.0"),          # [deg]
        DeclareLaunchArgument("lookahead", default_value="0.20"),          # [m]

        DeclareLaunchArgument("kp_lat", default_value="1.2"),
        DeclareLaunchArgument("ki_lat", default_value="0.0"),
        DeclareLaunchArgument("kd_lat", default_value="0.05"),
        DeclareLaunchArgument("integral_limit", default_value="0.30"),

        DeclareLaunchArgument("k_angle", default_value="1.5"),

        DeclareLaunchArgument("max_vx", default_value="0.20"),
        DeclareLaunchArgument("max_vy", default_value="0.20"),
        DeclareLaunchArgument("max_wz", default_value="1.2"),

        DeclareLaunchArgument("min_front_dist", default_value="0.25"),     # [m]
        DeclareLaunchArgument("front_window_deg", default_value="20.0"),   # +/- [deg]

        DeclareLaunchArgument("sample_window", default_value="2"),         # median over +/- indices
        DeclareLaunchArgument("control_rate_hz", default_value="20.0"),

        wall_follower_node,
    ])
