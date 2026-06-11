"""Single-drone Swarm Autonomy stack under /drone_0.

Brings up the per-drone nodes (controller, VIO bridge, mapping, planner) in a
namespace. PX4 SITL + Gazebo + the uXRCE-DDS agent are started separately by
``sim/launch_sim.sh``; this launch file owns only the ROS 2 application graph.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params = PathJoinSubstitution(
        [FindPackageShare("swarm_autonomy_bringup"), "config", "drone_params.yaml"])
    ns = LaunchConfiguration("namespace")

    common = {"namespace": ns, "parameters": [params], "output": "screen"}

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="drone_0"),
        Node(package="swarm_autonomy_control", executable="offboard_control", **common),
        Node(package="swarm_autonomy_perception", executable="vio_to_ekf2_bridge", **common),
        Node(package="swarm_autonomy_perception", executable="target_detector", **common),
        Node(package="swarm_autonomy_mapping", executable="map_merge_node", **common),
        Node(package="swarm_autonomy_planning", executable="planner_node", **common),
        Node(package="swarm_autonomy_coordination", executable="coordination_node", **common),
    ])
