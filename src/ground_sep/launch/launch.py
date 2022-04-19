import launch
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

import os

ground_sep_pkg_prefix = get_package_share_directory('ground_sep')
ground_sep_param_file = os.path.join(ground_sep_pkg_prefix, 'param/params.yaml')

euclidean_clustering_pkg_prefix = get_package_share_directory('euclidean_clustering')
euclidean_clustering_param_file = os.path.join(euclidean_clustering_pkg_prefix, 'param/params.yaml')




def generate_launch_description():
    ground_sep_node = Node(
        package='ground_sep',
        executable='ground_sep_node_exe',
        namespace='ground_sep',
        parameters=[ground_sep_param_file]
        # prefix=['valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=yes'],
        # output='screen'
    )

    # ros2 run tf2_ros static_transform_publisher 0 0 0 0 -0.174533 0 map map_c
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "-0.174533", "0", "velodyne_middle", "velodyne_middle_corrected"]
    )

    euclidean_clustering_node = Node(
        package='euclidean_clustering',
        executable='euclidean_clustering_node_exe',
        namespace='euclidean_clustering',
        parameters=[euclidean_clustering_param_file]
    )

    return launch.LaunchDescription([ground_sep_node, static_transform_publisher, euclidean_clustering_node])

