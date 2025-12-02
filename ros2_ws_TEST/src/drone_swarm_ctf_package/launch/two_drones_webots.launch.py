from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    pkg_dir = get_package_share_directory('drone_swarm_ctf_package')

    world = PathJoinSubstitution([
        pkg_dir, 'worlds', 'mavic_2_pro.wbt'
    ])

    webots = WebotsLauncher(
        world=world,
        mode = 'realtime'
    )

    supervisor = Node(
        package='webots_ros2_driver',
        executable='ros2_supervisor.py',
        output='screen',
        namespace='Ros2Supervisor',
        parameters = [{'robot_name': 'Ros2Supervisor'},
                      {'use_sim_time': True}]
    )

    drone1 = Node(
        package='drone_swarm_ctf_package',
        executable='drone1_driver',
        output='screen',
        namespace='drone1',
        parameters=[{'robot_description': PathJoinSubstitution([pkg_dir, 'resource', 'drone2.urdf'])},
                    {'robot_name': 'drone1'},
                    {'use_sim_time': True}]
    )

    drone2 = Node(
        package='drone_swarm_ctf_package',
        executable='drone2_driver',
        output='screen',
        namespace='drone2',
        parameters=[{'robot_description': PathJoinSubstitution([pkg_dir, 'resource', 'drone2.urdf'])},
                    {'robot_name': 'drone2'},
                    {'use_sim_time': True}]
    )

    
    return LaunchDescription([
        webots,
        supervisor,
        drone1,
        drone2,
    ])