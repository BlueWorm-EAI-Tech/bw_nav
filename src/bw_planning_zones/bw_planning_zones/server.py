from array import array
from functools import partial

from bw_interface.msg import PlanningZone, PlanningZoneState
from bw_interface.srv import (
    CreatePlanningZone, DeletePlanningZone, ListPlanningZones, ReloadPlanningZones,
    ReplacePlanningZones, SetPlanningZoneEnabled, UpdatePlanningZone,
)
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from .geometry import MapSpec, Zone
from .store import Store, ZoneError

SNAPSHOT_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE)


def zone_from_msg(msg):
    if any(point.z != 0.0 for point in msg.vertices):
        raise ZoneError('INVALID_ARGUMENT', 'Vertex z must be zero')
    return Zone(msg.id, msg.name, msg.enabled, tuple((p.x, p.y) for p in msg.vertices))


def zone_msg(zone):
    return PlanningZone(id=zone.id, name=zone.name, enabled=zone.enabled,
                        vertices=[Point(x=x, y=y) for x, y in zone.vertices])


class PlanningZoneServer(Node):
    def __init__(self, *, map_yaml='', zones_file=''):
        super().__init__('planning_zone_server')
        try:
            spec = MapSpec.load(self.declare_parameter('map_yaml', map_yaml).value)
            self.store = Store(spec, self.declare_parameter('zones_file', zones_file).value)
        except Exception:
            self.destroy_node()
            raise
        self.mask = self.create_publisher(OccupancyGrid, 'planning_zones/mask', SNAPSHOT_QOS)
        self.markers = self.create_publisher(MarkerArray, 'planning_zones/markers', SNAPSHOT_QOS)
        self.state = self.create_publisher(PlanningZoneState, 'planning_zones/state', SNAPSHOT_QOS)
        for name, typ in (
            ('create', CreatePlanningZone), ('delete', DeletePlanningZone),
            ('list', ListPlanningZones), ('replace_all', ReplacePlanningZones),
            ('update', UpdatePlanningZone), ('set_enabled', SetPlanningZoneEnabled),
            ('reload', ReloadPlanningZones),
        ):
            self.create_service(typ, 'planning_zones/' + name, partial(self.handle_request, name))
        self.pending = self.messages(self.store.snapshot)
        # A steady clock also retries when a simulation clock is paused.
        self.create_timer(1.0, self.publish_pending,
                          clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
        self.publish_pending()

    def messages(self, snapshot):
        spec = self.store.spec
        stamp = self.get_clock().now().to_msg()
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = stamp
        grid.info.map_load_time = stamp
        grid.info.width, grid.info.height = spec.width, spec.height
        grid.info.resolution = spec.resolution
        grid.info.origin.position.x, grid.info.origin.position.y = spec.origin[:2]
        grid.info.origin.orientation.w = 1.0
        grid.data = array('b', snapshot.mask)
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        markers.markers[0].header.frame_id = 'map'
        for i, (zone, triangles) in enumerate(zip(snapshot.zones, snapshot.fills)):
            if not zone.enabled:
                continue
            outline = (*zone.vertices, zone.vertices[0])
            for kind, ns, vertices, height, alpha in (
                (Marker.TRIANGLE_LIST, 'fill', [p for tri in triangles for p in tri], 0.02, 0.35),
                (Marker.LINE_STRIP, 'outline', outline, 0.023, 0.95),
            ):
                marker = Marker(id=i, ns=ns, type=kind, action=Marker.ADD)
                marker.header.frame_id = 'map'
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.02 if kind == Marker.LINE_STRIP else 1.0
                marker.scale.y = marker.scale.z = 1.0
                marker.color.r, marker.color.a = 1.0, alpha
                marker.points = [Point(x=x, y=y, z=height) for x, y in vertices]
                markers.markers.append(marker)
        state = PlanningZoneState(map_id=spec.id, revision=snapshot.revision,
                                  zones=[zone_msg(zone) for zone in snapshot.zones])
        return grid, markers, state

    def publish_pending(self):
        if self.pending is not None:
            try:
                for pub, msg in zip((self.mask, self.markers, self.state), self.pending):
                    pub.publish(msg)
            except Exception as error:
                self.get_logger().error(f'Zone snapshot committed; will retry publication: {error}')
                return False
            self.pending = None
        return True

    def handle_request(self, operation, req, response):
        response.map_id = self.store.spec.id
        prepared = []
        try:
            if operation == 'list':
                if req.map_id and req.map_id != response.map_id:
                    raise ZoneError('MAP_MISMATCH', 'Selected map does not match server map')
                response.zones = [zone_msg(zone) for zone in self.store.snapshot.zones]
            else:
                zone = zone_from_msg(req.zone) if hasattr(req, 'zone') else None
                zones = [zone_from_msg(z) for z in req.zones] if hasattr(req, 'zones') else ()
                created = self.store.mutate(
                    operation, req.map_id, req.expected_revision, zone=zone, zones=zones,
                    zone_id=getattr(req, 'id', ''), enabled=getattr(req, 'enabled', False),
                    before_commit=lambda snapshot: prepared.append(self.messages(snapshot)),
                )
                if created:
                    response.id = created.id
                if prepared:
                    self.pending = prepared[0]
                response.published = self.publish_pending()
                if not response.published:
                    response.message = 'Saved; publication will be retried'
            response.success = True
        except ZoneError as error:
            response.error_code, response.message = error.code, str(error)
        except OSError as error:
            response.error_code, response.message = 'IO_ERROR', str(error)
        except (ValueError, yaml.YAMLError) as error:
            response.error_code, response.message = 'INVALID_ARGUMENT', str(error)
        response.revision = self.store.snapshot.revision
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PlanningZoneServer()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
