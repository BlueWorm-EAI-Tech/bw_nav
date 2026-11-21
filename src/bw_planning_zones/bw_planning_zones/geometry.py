from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

from PIL import Image
from shapely.geometry import Point, Polygon, box
import yaml


def number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('Expected a finite number')
    return float(value)


@dataclass(frozen=True)
class MapSpec:
    path: Path
    image: Path
    width: int
    height: int
    resolution: float
    origin: tuple
    digest: str

    @classmethod
    def load(cls, path):
        path = Path(path).expanduser().resolve()
        data = yaml.safe_load(path.read_bytes())
        if not isinstance(data, dict) or not isinstance(data.get('image'), str):
            raise ValueError('Map YAML must specify an image')
        image = (path.parent / data['image']).resolve()
        resolution = number(data.get('resolution'))
        origin = data.get('origin')
        if not isinstance(origin, list) or len(origin) != 3:
            raise ValueError('Expected origin [x, y, yaw]')
        origin = tuple(map(number, origin))
        if resolution <= 0 or origin[2] != 0:
            raise ValueError('Expected positive resolution and zero origin yaw')
        with Image.open(image) as bitmap:
            width, height = bitmap.size
        with image.open('rb') as source:
            digest = hashlib.sha256()
            for chunk in iter(lambda: source.read(1024 * 1024), b''):
                digest.update(chunk)
        return cls(path, image, width, height, resolution, origin, digest.hexdigest())

    @property
    def metadata(self):
        return dict(image_sha256=self.digest, width=self.width, height=self.height,
                    resolution=self.resolution, origin=list(self.origin))

    @property
    def id(self):
        return hashlib.sha256(json.dumps(self.metadata, sort_keys=True).encode()).hexdigest()

    @property
    def config_path(self):
        return self.path.with_suffix('.planning_zones.yaml')

    def pixel_to_world(self, u, v):
        return (self.origin[0] + u * self.resolution,
                self.origin[1] + (self.height - v) * self.resolution)

    def world_to_pixel(self, x, y):
        return ((x - self.origin[0]) / self.resolution,
                self.height - (y - self.origin[1]) / self.resolution)


@dataclass(frozen=True)
class Zone:
    id: str
    name: str
    enabled: bool
    vertices: tuple

    def normalized(self, spec):
        if (not isinstance(self.id, str) or not self.id.strip() or
                not isinstance(self.name, str) or type(self.enabled) is not bool):
            raise ValueError('Expected a nonempty id, a string name and a boolean enabled flag')
        vertices = []
        for point in self.vertices:
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise ValueError('Each vertex must be [x, y]')
            x, y = map(number, point)
            left, bottom = spec.origin[:2]
            right = left + spec.width * spec.resolution
            top = bottom + spec.height * spec.resolution
            if not (left <= x <= right and bottom <= y <= top):
                raise ValueError('Vertex outside map limits')
            if not vertices or vertices[-1] != (x, y):
                vertices.append((x, y))
        if len(vertices) > 1 and vertices[0] == vertices[-1]:
            vertices.pop()
        if len(vertices) < 3 or not (p := Polygon(vertices)).is_valid or not p.area:
            raise ValueError('Expected a simple polygon with nonzero area')
        return Zone(self.id, self.name, self.enabled, tuple(vertices))


def triangles(vertices):
    """Ear clipping keeps every triangle inside the validated simple polygon."""
    vertices = list(vertices)
    if not Polygon(vertices).exterior.is_ccw:
        vertices.reverse()
    result = []
    while len(vertices) > 3:
        for i, b in enumerate(vertices):
            a, c = vertices[i - 1], vertices[(i + 1) % len(vertices)]
            cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
            if cross == 0:
                vertices.pop(i)
                break
            if cross < 0:
                continue
            ear = Polygon((a, b, c))
            neighbors = {(i - 1) % len(vertices), i, (i + 1) % len(vertices)}
            if not any(ear.covers(Point(v)) for j, v in enumerate(vertices) if j not in neighbors):
                result.append((a, b, c))
                vertices.pop(i)
                break
        else:
            raise ValueError('Could not triangulate polygon')
    if Polygon(vertices).area:
        result.append(tuple(vertices))
    return result


def _areas(geometry):
    if geometry.geom_type == 'Polygon' and geometry.area > 0:
        yield geometry
    elif hasattr(geometry, 'geoms'):
        for child in geometry.geoms:
            yield from _areas(child)


def _snap_grid_edge(value):
    # Decimal map resolutions can put an exact cell boundary a few ULPs off the grid.
    nearest = round(value)
    return nearest if abs(value - nearest) <= 1e-10 else value


def rasterize(spec, zones):
    mask = bytearray(spec.width * spec.height)
    ox, oy = spec.origin[:2]
    for zone in zones:
        if not zone.enabled:
            continue
        vertices = [((x - ox) / spec.resolution, (y - oy) / spec.resolution)
                    for x, y in zone.vertices]
        polygon = Polygon([(_snap_grid_edge(x), _snap_grid_edge(y)) for x, y in vertices])
        if not polygon.is_valid or not polygon.area:
            polygon = Polygon(vertices)
        _, bottom, _, top = polygon.bounds
        for row in range(max(0, math.floor(bottom)), min(spec.height, math.ceil(top))):
            strip = polygon.intersection(box(0, row, spec.width, row + 1))
            # Project each connected piece of the row strip separately, preserving notches.
            for piece in _areas(strip):
                left, _, right, _ = piece.bounds
                a = max(0, math.floor(left))
                b = min(spec.width, math.ceil(right))
                offset = row * spec.width
                mask[offset + a:offset + b] = b'\x64' * (b - a)
    return bytes(mask)
