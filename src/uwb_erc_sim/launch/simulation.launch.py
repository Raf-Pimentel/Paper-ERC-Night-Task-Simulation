import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('uwb_erc_sim')
    world_file = os.path.join(pkg_share, 'worlds', 'standard.sdf')
    
    # THE CRITICAL JAZZY PATHS
    gz_vendor_lib = "/opt/ros/jazzy/opt/gz_sim_vendor/lib"
    gz_vendor_plugins = "/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins"

    return LaunchDescription([
        # 1. Fix the Plugin Loading Error
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', f"{gz_vendor_plugins}:{gz_vendor_lib}"),
        SetEnvironmentVariable('LD_LIBRARY_PATH', f"{gz_vendor_lib}:{os.environ.get('LD_LIBRARY_PATH', '')}"),
        
        # 2. Fix the Dell G15 / NVIDIA Display Error
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        SetEnvironmentVariable('OGRE_RTT_MODE', 'FBO'), # Forces Ogre2 to use Frame Buffer Objects

        ExecuteProcess(cmd=['gz', 'sim', '-r', world_file], output='screen'),

        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
             output='screen')
    ])
