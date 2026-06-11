"""Multi-drone Swarm Autonomy swarm.

Replicates the per-drone stack into N namespaces (/drone_0 .. /drone_{N-1}) and
starts the single comms middleware that brokers all inter-drone traffic with
range/rate/dropout gating + bandwidth logging. Use ``num_drones:=5`` to scale.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _spawn(context, *args, **kwargs):
    n = int(LaunchConfiguration("num_drones").perform(context))
    params = PathJoinSubstitution(
        [FindPackageShare("swarm_autonomy_bringup"), "config", "drone_params.yaml"])
    profile = LaunchConfiguration("profile").perform(context)

    actions = []
    for i in range(n):
        ns = f"drone_{i}"
        common = dict(namespace=ns, parameters=[params, {"drone_id": i, "num_drones": n}],
                      output="screen")
        actions += [
            Node(package="swarm_autonomy_control", executable="offboard_control", **common),
            Node(package="swarm_autonomy_perception", executable="vio_to_ekf2_bridge", **common),
            Node(package="swarm_autonomy_perception", executable="target_detector", **common),
            Node(package="swarm_autonomy_mapping", executable="map_merge_node", **common),
            Node(package="swarm_autonomy_planning", executable="planner_node",
                 namespace=ns, parameters=[params, {"drone_id": i, "profile": profile}],
                 output="screen"),
            Node(package="swarm_autonomy_coordination", executable="coordination_node", **common),
        ]

    # Single broker for all inter-drone traffic.
    actions.append(Node(
        package="swarm_autonomy_comms", executable="comms_middleware",
        parameters=[params, {"num_drones": n}], output="screen"))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("num_drones", default_value="3"),
        DeclareLaunchArgument("profile", default_value="coop_explore"),
        OpaqueFunction(function=_spawn),
    ])
