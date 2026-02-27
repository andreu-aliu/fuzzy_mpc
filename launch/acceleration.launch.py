from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('ltv_mpc')
    ws_root = os.path.abspath(os.path.join(pkg_share, "../../../../"))
    debug_path = os.path.join(ws_root,"src/as/control/ltv_mpc/test/data/")

    params_file = os.path.join(pkg_share, 'params', 'params.yaml')
    dyn_file    = os.path.join(pkg_share, 'params', 'dyn_trackdrive.yaml')

    return LaunchDescription([
        Node(
            package='ltv_mpc',
            executable='ltv_mpc',
            name='ltv_mpc',
            namespace='as/c',
            output='screen',
            parameters=[    
                params_file,                        # Generic parameters of ltv_mpc
                dyn_file,                           # Specific event parameters
                {'MPC.debug_path': debug_path},     # Set debug_path parameter
                {'use_sim_time': LaunchConfiguration('use_sim_time', default="false")}
            ]
        ),
    ])
