"""Run the Humble planner server with synthetic maps, without chassis nodes."""
import os
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml


def test_global_planner_detours_and_accepts_start_inside_zone(tmp_path, monkeypatch):
    width, height = 80, 40
    domain = 190 + os.getpid() % 20
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(domain_id=domain)
    node = rclpy.create_node('planning_zone_planner_test')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    base = node.create_publisher(OccupancyGrid, '/test/map', qos)
    mask = node.create_publisher(OccupancyGrid, '/planning_zones/mask', qos)
    raw_plans = []
    node.create_subscription(PathMsg, '/unsmoothed_plan', raw_plans.append, 1)
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = 0.1
    grid.info.width, grid.info.height = width, height
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * (width * height)
    base.publish(grid)

    def publish_zone(left, right, bottom=10, top=30):
        grid.data = [100 if left <= x < right and bottom <= y < top else 0
                     for y in range(height) for x in range(width)]
        mask.publish(grid)

    publish_zone(15, 65)
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'base_link'
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)

    navigation = yaml.safe_load((Path(__file__).resolve().parents[1] /
                                 'config' / 'nav2_omni.yaml').read_text())
    global_costmap = navigation['global_costmap']['global_costmap']['ros__parameters']
    params = {
        'planner_server': navigation['planner_server'],
        'global_costmap': {'global_costmap': {'ros__parameters': {
            'global_frame': 'map', 'robot_base_frame': 'base_link', 'update_frequency': 10.0,
            'width': 8, 'height': 4, 'resolution': 0.1, 'plugins': ['static_layer'],
            'filters': ['planning_zone_layer'],
            'static_layer': {'plugin': 'nav2_costmap_2d::StaticLayer', 'map_topic': '/test/map',
                             'map_subscribe_transient_local': True},
            'planning_zone_layer': global_costmap['planning_zone_layer'],
        }}},
    }
    path = tmp_path / 'planner.yaml'
    path.write_text(yaml.safe_dump(params))
    log = tmp_path / 'planner.log'
    executable = Path(get_package_prefix('nav2_planner')) / 'lib/nav2_planner/planner_server'
    process = None

    def wait(predicate):
        until = time.monotonic() + 8
        while not predicate() and time.monotonic() < until and process.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.03)
        assert predicate(), log.read_text()

    def result(future):
        wait(future.done)
        return future.result()

    def pose(x, y):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.w = 1.0
        return p

    with log.open('w') as output:
        try:
            process = subprocess.Popen(
                [str(executable), '--ros-args', '--params-file', str(path)],
                env=dict(os.environ, ROS_DOMAIN_ID=str(domain)), stdout=output,
                stderr=subprocess.STDOUT,
            )
            lifecycle = node.create_client(ChangeState, '/planner_server/change_state')
            wait(lifecycle.service_is_ready)
            for transition in (1, 3):
                req = ChangeState.Request()
                req.transition.id = transition
                assert result(lifecycle.call_async(req)).success, log.read_text()
            costmap = node.create_client(GetCostmap, '/global_costmap/get_costmap')
            wait(costmap.service_is_ready)

            def wait_for_cost(expected):
                until = time.monotonic() + 4
                while True:
                    actual = result(costmap.call_async(GetCostmap.Request())).map.data
                    if len(actual) == width * height and actual[20 * width + 40] == expected:
                        return
                    assert time.monotonic() < until, log.read_text()

            wait_for_cost(252)
            wait(lambda: node.count_publishers('/unsmoothed_plan') > 0)

            planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
            wait(planner.server_is_ready)

            def plan(start):
                raw_plans.clear()
                goal = ComputePathToPose.Goal(start=pose(*start), goal=pose(7.65, 2.05),
                                             planner_id='GridBased', use_start=True)
                handle = result(planner.send_goal_async(goal))
                assert handle.accepted, log.read_text()
                planned = result(handle.get_result_async()).result.path.poses
                assert planned, log.read_text()
                wait(lambda: bool(raw_plans))
                return ([p.pose.position for p in planned],
                        [p.pose.position for p in raw_plans[0].poses])

            inside, _ = plan((6.05, 2.05))
            assert 1.5 <= inside[0].x < 6.5 and inside[-1].x > 6.5
            outside, raw_outside = plan((0.35, 2.05))
            assert not any(1.5 <= p.x < 6.5 and 1.0 <= p.y < 3.0 for p in raw_outside)
            # The production smoother may skim the first row of a traversable zone.
            # At its midpoint the returned route still avoids the interior core.
            middle = min(outside, key=lambda p: abs(p.x - 4.0))
            assert not 1.1 <= middle.y <= 2.9

            publish_zone(0, 0)
            wait_for_cost(0)
            cleared, _ = plan((0.35, 2.05))
            assert all(abs(p.y - 2.05) < 1e-5 for p in cleared)

            # A zone spanning the map cannot be bypassed, but is still traversable.
            publish_zone(35, 45, 0, height)
            wait_for_cost(252)
            traversed, _ = plan((0.35, 2.05))
            assert traversed[-1].x > 4.5
        finally:
            if process:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()
