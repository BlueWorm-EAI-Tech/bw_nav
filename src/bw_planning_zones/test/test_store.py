import math
from dataclasses import replace

import pytest
import yaml
from PIL import Image
from shapely.geometry import Polygon

from bw_planning_zones.geometry import MapSpec, Zone, rasterize, triangles
from bw_planning_zones.store import Store, ZoneError


@pytest.fixture
def map_spec(tmp_path):
    Image.new('L', (8, 6), 254).save(tmp_path / 'map.pgm')
    path = tmp_path / 'map.yaml'
    path.write_text(yaml.safe_dump({'image': 'map.pgm', 'resolution': 0.5,
                                    'origin': [-1.0, -2.0, 0.0]}))
    return MapSpec.load(path)


def test_concave_and_edge_contact(map_spec):
    vertices = ((-1, -2), (0.5, -2), (0.5, -1.5), (-0.5, -1.5),
                (-0.5, 0), (-1, 0))
    zone = Zone('a', 'Concave', True, vertices)
    mask = rasterize(map_spec, [zone, zone])
    assert len(mask) == 48
    assert {i for i, v in enumerate(mask) if v == 100} == {0, 1, 2, 8, 16, 24}
    parts = triangles(vertices)
    polygon = Polygon(vertices)
    assert all(polygon.covers(Polygon(part)) for part in parts)
    assert math.isclose(sum(Polygon(part).area for part in parts), polygon.area)


def test_thin_polygon_and_map_transform(map_spec):
    xy = map_spec.pixel_to_world(1.5, 2.5)
    assert map_spec.world_to_pixel(*xy) == (1.5, 2.5)
    zone = Zone('a', '', True, ((-0.99, -1.99), (0.24, -1.99), (0.24, -1.98)))
    assert rasterize(map_spec, [zone]).count(100) == 3


def test_decimal_grid_edges_do_not_mark_touching_neighbors(map_spec):
    spec = replace(map_spec, origin=(-10., -8., 0.), resolution=0.05)
    zone = Zone('a', '', True, tuple(spec.pixel_to_world(x, spec.height - y)
                                   for x, y in [(3, 3), (4, 3), (4, 4), (3, 4)]))
    assert [i for i, value in enumerate(rasterize(spec, [zone])) if value] == [3 * spec.width + 3]


def test_revision_and_delete(map_spec):
    store = Store(map_spec)
    zone = Zone('', 'Zone', True, ((-1, -2), (0, -2), (0, -1)))
    saved = store.mutate('create', map_spec.id, 0, zone=zone)
    assert saved.id
    with pytest.raises(ZoneError, match='revision'):
        store.mutate('delete', map_spec.id, 0, zone_id=saved.id)
    assert Store(map_spec).snapshot.zones == (saved,)
    store.mutate('delete', map_spec.id, 1, zone_id=saved.id)
    assert store.snapshot.revision == 2
    assert not any(store.snapshot.mask)


def test_bad_geometry_and_wrong_map_do_not_write(map_spec):
    store = Store(map_spec)
    bad = Zone('bad', '', True, ((-1, -2), (0, -1), (0, -2), (-1, -1)))
    with pytest.raises(ZoneError):
        store.mutate('create', map_spec.id, 0, zone=bad)
    with pytest.raises(ZoneError):
        store.mutate('create', 'other', 0, zone=bad)
    assert not map_spec.config_path.exists()
    assert store.snapshot.revision == 0
