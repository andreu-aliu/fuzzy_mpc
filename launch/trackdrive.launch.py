from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Read the workspace path
    pkg_share = get_package_share_directory('fuzzy_mpc')
    ws_path = os.path.abspath(
        os.path.join(pkg_share, '..', '..', '..', '..')
    )

    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    dyn_file    = os.path.join(pkg_share, 'config', 'dyn_trackdrive.yaml')

    return LaunchDescription([
        Node(
            package='fuzzy_mpc',
            executable='fuzzy_mpc',
            name='fuzzy_mpc',
            namespace='as/c',
            output='screen',
            # prefix = ['gnome-terminal -- gdb -x /home/andreu/ros_ws/src/as/control/fuzzy_mpc/debug/breakpoints.gdb --args'], # Debug with breakpoints
            prefix = ['gnome-terminal -- gdb --args'], # Debug only
            parameters=[    
                params_file,                        # Generic parameters of fuzzy_mpc
                dyn_file,                           # Specific event parameters
                {'ws_path': ws_path},               # Set workspace path
                {'use_sim_time': LaunchConfiguration('use_sim_time', default="false")}
            ]
        ),
    ])
