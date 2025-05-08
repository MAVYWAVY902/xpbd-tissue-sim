from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_filename',
        default_value=TextSubstitution(text='/workspace/config/demos/virtuoso_trachea/virtuoso_trachea.yaml'),
        description='Absolute path to the simulation config file.'
    )

    # the sim_bridge node
    sim_bridge_node = Node(
        package='sim_bridge',
        executable='sim_bridge_node',
        name='sim_bridge',
        output='screen',
        remappings=[
            ('/output/arm1_frames', '/sim/arm1_frames'),
            ('/output/arm2_frames', '/sim/arm2_frames'),
            ('/output/tissue_mesh', '/sim/tissue_mesh'),
            ('/output/tissue_mesh_vertices', '/sim/tissue_mesh_vertices')
        ],
        parameters=[
            {"publish_rate_hz": 30.0}
        ],
        arguments=[
            '--config-filename', LaunchConfiguration('config_filename')
        ]
    )

    # Include rosbridge_server launch file
    rosbridge_launch_file = PathJoinSubstitution([
        FindPackageShare('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml'
    ])
    
    # Create the include launch description action
    rosbridge_server = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rosbridge_launch_file)
    )

    ld = LaunchDescription([
        config_file_arg,
        sim_bridge_node,
        rosbridge_server
    ])

    return ld