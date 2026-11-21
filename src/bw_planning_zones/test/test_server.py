import os
import time

import pytest
from PIL import Image

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('bw_interface.srv._create_planning_zone')

from bw_interface.srv import CreatePlanningZone, DeletePlanningZone
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray

from bw_planning_zones.server import PlanningZoneServer, SNAPSHOT_QOS


def test_create_delete_and_late_subscriber(tmp_path):
    # Use localhost-only DDS; this test must never edit a live robot's zones.
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    Image.new('L', (8, 6), 254).save(tmp_path / 'map.pgm')
    path = tmp_path / 'map.yaml'
    path.write_text('image: map.pgm\nresolution: 0.5\norigin: [0, 0, 0]\n')
    rclpy.init(domain_id=184)
    server = None
    client = None
    try:
        server = PlanningZoneServer(map_yaml=str(path))
        client = rclpy.create_node('planning_zone_test')
        create = client.create_client(CreatePlanningZone, '/planning_zones/create')
        delete = client.create_client(DeletePlanningZone, '/planning_zones/delete')

        def wait(predicate):
            until = time.monotonic() + 5
            while not predicate() and time.monotonic() < until:
                rclpy.spin_once(server, timeout_sec=0.02)
                rclpy.spin_once(client, timeout_sec=0.02)
            assert predicate()

        wait(create.service_is_ready)
        request = CreatePlanningZone.Request(map_id=server.store.spec.id)
        request.zone.enabled = True
        request.zone.vertices = [Point(x=x, y=y) for x, y in [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]]
        result = create.call_async(request)
        wait(result.done)
        saved = result.result()
        assert saved.success and saved.published and saved.id and saved.revision == 1
        assert server.store.path.is_file()

        masks, markers = [], []
        client.create_subscription(OccupancyGrid, '/planning_zones/mask', masks.append, SNAPSHOT_QOS)
        client.create_subscription(MarkerArray, '/planning_zones/markers', markers.append, SNAPSHOT_QOS)
        wait(lambda: masks and markers)
        assert 100 in masks[-1].data
        assert markers[-1].markers[1].type == 11
        request = DeletePlanningZone.Request(id=saved.id, map_id=saved.map_id, expected_revision=1)
        result = delete.call_async(request)
        wait(result.done)
        assert result.result().success
        wait(lambda: masks and markers and not any(masks[-1].data) and len(markers[-1].markers) == 1)
        assert markers[-1].markers[0].action == 3
    finally:
        if server:
            server.destroy_node()
        if client:
            client.destroy_node()
        rclpy.shutdown()
