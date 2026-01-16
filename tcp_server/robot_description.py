
import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description_robot():
    

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # lidar = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','rplidar_c1_launch_l1.py'
    #             )])
    # )

    

    # lidar2 = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','rplidar_c1_launch_l2.py'
    #             )])
    # )

    lidars = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rplidar_c1_launch_all.py'
                )])
    )

    lidar_merger = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','merge_2_scan.launch.py'
                )])
    )

    # joystick = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','joystick.launch.py'
    #             )])
    # )
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
                        parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/mecanum_controller/reference_unstamped')]
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        #parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel_in','/mecanum_controller/reference_unstamped'),
                    ('/cmd_vel_out','/mecanum_controller/reference')]
    )



    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file],
                    remappings=[('/mecanum_controller/odometry','/odom'),
                    ('/mecanum_controller/tf_odometry','/tf')]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    mecanum_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        #arguments=["mecanum_controller", "--controller-manager", "/controller_manager"],
        arguments=["mecanum_controller"],
    )

    delayed_mecanum_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[mecanum_controller_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        #arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        arguments=["joint_state_broadcaster"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )


    slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','online_async_launch.py'
                )]), launch_arguments={'slam_params_file': './src/my_bot/config/mapper_params_online_async.yaml'}.items()
    )

    nav = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','navigation_launch.py'
                )]), launch_arguments={'params_file': './src/my_bot/config/nav2_params.yaml'}.items()
    )

    # tcp_server = Node(
    #     package="tcp_server",
    #     executable="talker",
    # )

    #joint_state_broadcaster_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner",
    #    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    #)


    #delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
    #    event_handler=OnProcessStart(
    #        target_action=controller_manager,
    #        on_start=[joint_state_broadcaster_spawner],
    #    )
    #)


    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner



    # Launch them all!
    return LaunchDescription([
        rsp,
        lidars,
        lidar_merger,
        # joystick,
        twist_mux,
        twist_stamper,
        delayed_controller_manager,
        delayed_mecanum_controller_spawner,
        delayed_joint_broad_spawner
        # slam
        # nav,
        # tcp_server
    ])
