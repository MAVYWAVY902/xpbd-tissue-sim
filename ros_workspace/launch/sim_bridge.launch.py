from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

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
            ('/input/arm1_joint_state', '/ves/left/joint/servo_jp'),
            ('/input/arm2_joint_state', '/ves/right/joint/servo_jp'),
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

    ld = LaunchDescription([
        config_file_arg,
        sim_bridge_node,
    ])

    return ld