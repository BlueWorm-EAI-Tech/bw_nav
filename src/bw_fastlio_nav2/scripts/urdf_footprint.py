#!/usr/bin/env python3
"""Derive a Nav2 costmap `footprint` string from a URDF.

Nav2 cannot read a URDF: `nav2_costmap_2d` only accepts a flat list of 2D
vertices (`footprint`) or a single radius (`robot_radius`).  This tool closes
the gap offline: it walks the kinematic tree, projects every collision (or
visual) mesh onto the ground plane, takes the convex hull and optionally grows
it by a fixed margin.

Usage:
    python3 urdf_footprint.py MODEL.urdf
    python3 urdf_footprint.py MODEL.urdf --padding 0.08 --ascii

The outline is cached under ``$XDG_CACHE_HOME/bw_fastlio_nav2/urdf_footprint``
(override the location with ``BW_URDF_FOOTPRINT_CACHE``) and keyed on the URDF
plus every mesh it references, so rebuilding a mesh invalidates it by itself.
``--no-cache`` forces a fresh run; a missing or unreadable cache silently falls
back to the full computation.

Exit codes: 0 ok, 1 bad input, 2 geometry check failed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

try:  # scipy is optional; the monotone-chain fallback below is enough
    from scipy.spatial import ConvexHull
except ImportError:  # pragma: no cover - depends on the host image
    ConvexHull = None


# Integer packing grid for the duplicate filter: +-1048 m at a 1 mm step.  The
# span stays below 2**31 so ``ix * _GRID_SPAN + iy`` fits comfortably in int64.
_GRID_BIAS = 1 << 20
_GRID_SPAN = 1 << 21


# --------------------------------------------------------------------------- #
# URDF parsing
# --------------------------------------------------------------------------- #
def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis roll/pitch/yaw (rotation about X, then Y, then Z)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def _origin_transform(element: ET.Element | None) -> np.ndarray:
    transform = np.eye(4)
    if element is None:
        return transform
    xyz = [float(value) for value in element.get("xyz", "0 0 0").split()]
    rpy = [float(value) for value in element.get("rpy", "0 0 0").split()]
    transform[:3, :3] = _rpy_to_matrix(*rpy)
    transform[:3, 3] = xyz
    return transform


def _resolve_mesh(uri: str, urdf_dir: Path) -> Path | None:
    """Resolve package://, file:// or a plain relative path to a real file."""
    if uri.startswith("package://"):
        relative = uri[len("package://"):]
        try:  # works once the workspace is sourced
            from ament_index_python.packages import get_package_share_directory

            package, _, rest = relative.partition("/")
            candidate = Path(get_package_share_directory(package)) / rest
            if candidate.is_file():
                return candidate
        except Exception:
            pass
        # Fall back to walking up from the URDF until the package name matches.
        for parent in [urdf_dir, *urdf_dir.parents]:
            candidate = parent / relative
            if candidate.is_file():
                return candidate
        return None
    if uri.startswith("file://"):
        candidate = Path(uri[len("file://"):])
        return candidate if candidate.is_file() else None
    candidate = (urdf_dir / uri).resolve()
    return candidate if candidate.is_file() else None


def _stl_vertices(path: Path) -> np.ndarray:
    """Read the vertices of a binary or ASCII STL."""
    data = path.read_bytes()
    if len(data) >= 84:
        triangles = int(np.frombuffer(data[80:84], dtype=np.uint32)[0])
        if len(data) == 84 + triangles * 50:  # exact binary layout
            record = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)),
                               ("attr", "<u2")])
            faces = np.frombuffer(data[84:], dtype=record, count=triangles)
            return faces["vertices"].reshape(-1, 3).astype(float)
    vertices = []
    for line in data.decode("ascii", errors="ignore").splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == "vertex":
            vertices.append([float(value) for value in parts[1:]])
    return np.array(vertices, dtype=float).reshape(-1, 3)


def _shape_vertices(geometry: ET.Element) -> np.ndarray:
    """Vertices for the primitive shapes URDF allows besides meshes."""
    box = geometry.find("box")
    if box is not None:
        sx, sy, sz = (float(value) for value in box.get("size").split())
        return np.array([[x, y, z]
                         for x in (-sx / 2, sx / 2)
                         for y in (-sy / 2, sy / 2)
                         for z in (-sz / 2, sz / 2)])
    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        radius = float(cylinder.get("radius"))
        length = float(cylinder.get("length"))
        return np.array([[radius * math.cos(a), radius * math.sin(a), z]
                         for a in np.linspace(0, 2 * math.pi, 64, endpoint=False)
                         for z in (-length / 2, length / 2)])
    sphere = geometry.find("sphere")
    if sphere is not None:
        radius = float(sphere.get("radius"))
        return np.array([[radius * math.cos(a), radius * math.sin(a), 0.0]
                         for a in np.linspace(0, 2 * math.pi, 64, endpoint=False)])
    return np.empty((0, 3))


def load_urdf(urdf_path: Path, prefer: str, log) -> tuple[np.ndarray, list[dict]]:
    """Return every mesh vertex in the root-link frame plus per-link records."""
    root = ET.parse(urdf_path).getroot()
    urdf_dir = urdf_path.parent

    joints = {}
    child_links = set()
    for joint in root.findall("joint"):
        name = joint.get("name")
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        joints[child] = {
            "name": name,
            "parent": parent,
            "type": joint.get("type"),
            "transform": _origin_transform(joint.find("origin")),
        }
        child_links.add(child)
    links = [link.get("name") for link in root.findall("link")]
    roots = [name for name in links if name not in child_links]
    if len(roots) != 1:
        raise SystemExit(f"expected exactly one root link, found {roots}")

    link_world: dict[str, np.ndarray] = {roots[0]: np.eye(4)}

    def link_transform(name: str) -> np.ndarray:
        if name in link_world:
            return link_world[name]
        joint = joints[name]
        link_world[name] = link_transform(joint["parent"]) @ joint["transform"]
        return link_world[name]

    all_points = []
    records = []
    for link in root.findall("link"):
        name = link.get("name")
        elements = link.findall(prefer)
        source = prefer
        if not elements:
            elements = link.findall("visual" if prefer == "collision" else "collision")
            source = "visual" if source == "collision" else "collision"
        for element in elements:
            geometry = element.find("geometry")
            if geometry is None:
                continue
            mesh = geometry.find("mesh")
            if mesh is not None:
                path = _resolve_mesh(mesh.get("filename"), urdf_dir)
                if path is None:
                    log(f"[warn] {name}: cannot resolve {mesh.get('filename')}")
                    continue
                vertices = _stl_vertices(path)
                label = f"{path.parent.name}/{path.name}"
                triangles = len(vertices) // 3
                scale = mesh.get("scale")
                if scale:
                    vertices = vertices * np.array(
                        [float(value) for value in scale.split()])
            else:
                vertices = _shape_vertices(geometry)
                label = "primitive"
                triangles = 0
            if not len(vertices):
                continue
            transform = link_transform(name) @ _origin_transform(
                element.find("origin"))
            world = vertices @ transform[:3, :3].T + transform[:3, 3]
            all_points.append(world[:, :2])
            records.append({
                "link": name,
                "source": source,
                "mesh": label,
                "triangles": triangles,
                "xy": (world[:, 0].min(), world[:, 0].max(),
                       world[:, 1].min(), world[:, 1].max()),
            })
    return np.vstack(all_points) if all_points else np.empty((0, 2)), records


# --------------------------------------------------------------------------- #
# Geometry
# --------------------------------------------------------------------------- #
def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def monotone_chain(points: np.ndarray) -> np.ndarray:
    """Andrew's monotone chain; drops collinear vertices."""
    points = np.unique(points, axis=0)
    if len(points) < 3:
        return points

    def half(sequence):
        stack = []
        for point in sequence:
            while len(stack) >= 2 and _cross(stack[-2], stack[-1], point) <= 0:
                stack.pop()
            stack.append(point)
        return stack

    ordered = points[np.lexsort((points[:, 1], points[:, 0]))].tolist()
    lower = half(ordered)
    upper = half(ordered[::-1])
    return np.array(lower[:-1] + upper[:-1])


def convex_hull(points: np.ndarray) -> np.ndarray:
    if len(points) < 3:
        return points
    if ConvexHull is not None:
        hull = ConvexHull(points)
        return points[hull.vertices]
    return monotone_chain(points)


def signed_area(polygon: np.ndarray) -> float:
    x, y = polygon[:, 0], polygon[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def _line_intersection(p1, d1, p2, d2):
    matrix = np.array([[d1[0], -d2[0]], [d1[1], -d2[1]]])
    rhs = np.array([p2[0] - p1[0], p2[1] - p1[1]])
    t = np.linalg.solve(matrix, rhs)[0]
    return p1 + t * d1


def offset_convex(polygon: np.ndarray, distance: float) -> np.ndarray:
    """Minkowski sum with a disc, approximated by the outer miter polygon."""
    if distance <= 0.0:
        return polygon
    polygon = polygon if signed_area(polygon) > 0 else polygon[::-1]
    edges = []
    for index in range(len(polygon)):
        p = polygon[index]
        q = polygon[(index + 1) % len(polygon)]
        direction = (q - p) / np.linalg.norm(q - p)
        outward = np.array([direction[1], -direction[0]])
        edges.append((p + distance * outward, direction))
    return np.array([
        _line_intersection(*edges[index - 1], *edges[index])
        for index in range(len(edges))
    ])


def contains(polygon: np.ndarray, points: np.ndarray, eps: float = 1e-9) -> bool:
    if signed_area(polygon) < 0:
        polygon = polygon[::-1]
    for index in range(len(polygon)):
        p = polygon[index]
        q = polygon[(index + 1) % len(polygon)]
        if np.min((q[0] - p[0]) * (points[:, 1] - p[1])
                  - (q[1] - p[1]) * (points[:, 0] - p[0])) < -eps:
            return False
    return True


def inscribed_radius(polygon: np.ndarray) -> float:
    """Largest disc centred on base_footprint that fits inside *polygon*.

    This is the number Nav2 uses as the robot radius: a planner that calls
    ``setFootprint(hull, /*radius=*/true)`` only knows this width, and the map
    builder uses the very same value to clear returns the chassis must have
    driven over.  Deriving both from the same hull keeps the map and the
    costmap from disagreeing about how wide the robot is.

    Raises ``ValueError`` when the origin is not inside the polygon, because
    then no chassis-centred disc is guaranteed free.
    """
    polygon = np.asarray(polygon, dtype=float).reshape(-1, 2)
    if len(polygon) < 3:
        raise ValueError("footprint polygon needs at least 3 vertices")
    if signed_area(polygon) < 0:
        polygon = polygon[::-1]
    origin = np.zeros(2, dtype=float)
    if not contains(polygon, origin.reshape(1, 2), eps=0.0):
        raise ValueError("footprint does not contain base_footprint (0, 0)")
    best = float("inf")
    for index in range(len(polygon)):
        a = polygon[index]
        b = polygon[(index + 1) % len(polygon)]
        ab = b - a
        length_squared = float(ab @ ab)
        if length_squared <= 0.0:
            distance = float(np.linalg.norm(origin - a))
        else:
            t = float(np.clip((origin - a) @ ab / length_squared, 0.0, 1.0))
            distance = float(np.linalg.norm(origin - (a + t * ab)))
        best = min(best, distance)
    return best


def unique_2d(points: np.ndarray, quantize: float) -> np.ndarray:
    """Project onto a ``quantize`` grid and drop the duplicates.

    Packing both coordinates into a single int64 key replaces
    ``np.unique(..., axis=0)``, whose row-wise lexsort dominates the whole run
    (2.8 s of 3.4 s for the v6 chassis).  The packed variant needs 0.16 s for
    the same 4.5 M vertices and returns the same point set.
    """
    index = np.rint(points / quantize).astype(np.int64) + _GRID_BIAS
    if index.min() < 0 or index.max() >= _GRID_SPAN:
        # Model far outside the packed range: fall back to the generic path.
        rounded = np.round(points / quantize) * quantize
        return np.unique(rounded, axis=0)
    packed = np.unique(index[:, 0] * _GRID_SPAN + index[:, 1])
    ix, iy = np.divmod(packed, _GRID_SPAN)
    return (np.stack((ix, iy), axis=1) - _GRID_BIAS).astype(np.float64) * quantize


def drop_collinear(polygon: np.ndarray, scale: float = 1e3) -> np.ndarray:
    """Remove vertices that sit exactly on the segment between their neighbours.

    Uses integer arithmetic on a millimetre grid so the decision is exact and
    the polygon shape is preserved bit for bit -- unlike an area threshold,
    which would shave real corners and let points fall outside the hull.
    """
    grid = np.rint(polygon * scale).astype(np.int64)
    keep = []
    count = len(grid)
    for index in range(count):
        a, b, c = grid[index - 1], grid[index], grid[(index + 1) % count]
        cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        if cross != 0:
            keep.append(polygon[index])
    return np.array(keep) if len(keep) >= 3 else polygon


def simplify_convex(polygon: np.ndarray, tolerance: float) -> np.ndarray:
    """Drop the least significant vertices while the outline moves < tolerance.

    Each pass removes the vertex with the smallest distance to the chord of its
    neighbours, so the error never accumulates beyond `tolerance`.  The result
    is inscribed, hence the containment check in main() has to allow the same
    tolerance.
    """
    if tolerance <= 0.0 or len(polygon) < 4:
        return polygon
    points = [tuple(map(float, point)) for point in polygon]

    def deviation(index: int, vertices) -> float:
        count = len(vertices)
        a, b, c = vertices[index - 1], vertices[index], vertices[(index + 1) % count]
        length = math.hypot(c[0] - a[0], c[1] - a[1])
        return abs(_cross(a, b, c)) / length if length > 0.0 else math.inf

    while len(points) > 3:
        worst = min(((deviation(i, points), i) for i in range(len(points))))
        if worst[0] >= tolerance:
            break
        del points[worst[1]]
    return np.array(points)


def ascii_plot(polygon: np.ndarray, lidar: tuple[float, float] | None,
               width: int = 64) -> str:
    x_min, y_min = polygon.min(axis=0)
    x_max, y_max = polygon.max(axis=0)
    span_x = max(x_max - x_min, 1e-6)
    span_y = max(y_max - y_min, 1e-6)
    rows = max(int(width * span_y / span_x / 2.0), 8)
    canvas = [[" "] * width for _ in range(rows)]

    def to_cell(x, y):
        col = int((x - x_min) / span_x * (width - 1))
        row = rows - 1 - int((y - y_min) / span_y * (rows - 1))
        return max(0, min(rows - 1, row)), max(0, min(width - 1, col))

    for index in range(len(polygon)):
        p = polygon[index]
        q = polygon[(index + 1) % len(polygon)]
        for t in np.linspace(0, 1, max(int(np.hypot(*(q - p)) * 400), 2)):
            row, col = to_cell(*(p + t * (q - p)))
            canvas[row][col] = "#"
    if lidar is not None and x_min <= lidar[0] <= x_max and y_min <= lidar[1] <= y_max:
        row, col = to_cell(*lidar)
        canvas[row][col] = "L"
    row, col = to_cell(0.0, 0.0)
    canvas[row][col] = "O"
    return "\n".join("".join(line) for line in canvas)


# --------------------------------------------------------------------------- #
# Cache
# --------------------------------------------------------------------------- #
# Parsing the meshes costs ~0.3 s and the duplicate filter another ~0.1 s, but
# the answer only changes when a mesh does.  A hit therefore returns the stored
# outline in well under a millisecond; a missing or unreadable cache simply
# falls back to the full pipeline, so caching can never break a launch.
_CACHE_VERSION = 1
_CACHE_ENV = "BW_URDF_FOOTPRINT_CACHE"


def _cache_directory() -> Path | None:
    """Return a writable cache directory, or ``None`` when caching is impossible."""
    override = os.environ.get(_CACHE_ENV)
    if override:
        base = Path(override).expanduser()
    else:
        base = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
        base = base / "bw_fastlio_nav2" / "urdf_footprint"
    try:
        base.mkdir(parents=True, exist_ok=True)
    except OSError:
        return None
    return base if os.access(base, os.W_OK) else None


def _cache_inputs(urdf_path: Path) -> list[Path]:
    """Every file the geometry depends on: the URDF plus each referenced mesh."""
    files = [urdf_path]
    try:
        root = ET.parse(urdf_path).getroot()
    except ET.ParseError:
        return files
    for element in root.iter("mesh"):
        uri = element.get("filename") or ""
        resolved = _resolve_mesh(uri, urdf_path.parent) if uri else None
        if resolved is not None:
            files.append(resolved)
    return files


def _cache_key(urdf_path: Path, prefer: str, quantize: float, simplify: float,
               padding: float) -> str:
    """Hash the settings and the size/mtime of every input file."""
    parts = [
        f"v{_CACHE_VERSION}",
        str(urdf_path),
        prefer,
        f"{quantize:.6g}",
        f"{simplify:.6g}",
        f"{padding:.6g}",
    ]
    for path in _cache_inputs(urdf_path):
        try:
            stat = path.stat()
        except OSError:
            parts.append(f"{path}:missing")
        else:
            parts.append(f"{path}:{stat.st_mtime_ns}:{stat.st_size}")
    return hashlib.sha256("\n".join(parts).encode("utf-8")).hexdigest()[:32]


def _restore_records(records) -> list[dict]:
    """Rebuild the per-mesh records so a cache hit matches a fresh run exactly."""
    restored = []
    for record in records:
        record = dict(record)
        if "xy" in record:
            record["xy"] = tuple(float(value) for value in record["xy"])
        restored.append(record)
    return restored


def _load_cached(path: Path, urdf_path: Path, padding: float) -> dict | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        if int(payload["version"]) != _CACHE_VERSION:
            return None
        polygon = np.asarray(payload["polygon"], dtype=float).reshape(-1, 2)
        hull = np.asarray(payload["hull"], dtype=float).reshape(-1, 2)
        result = {
            "urdf": str(urdf_path),
            "points": int(payload["points"]),
            "unique_points": int(payload["unique_points"]),
            "exact_vertices": int(payload["exact_vertices"]),
            "padding": float(padding),
            "polygon": polygon,
            "hull": hull,
            "records": _restore_records(payload["records"]),
        }
    except (OSError, ValueError, KeyError, TypeError):
        return None
    if len(polygon) < 3 or len(hull) < 3:
        return None
    result["area"] = float(signed_area(polygon))
    result["radius"] = float(np.max(np.linalg.norm(polygon, axis=1)))
    result["cached"] = True
    return result


def _store_cached(path: Path, result: dict, log) -> None:
    payload = {
        "version": _CACHE_VERSION,
        "urdf": result["urdf"],
        "points": result["points"],
        "unique_points": result["unique_points"],
        "exact_vertices": result["exact_vertices"],
        "padding": result["padding"],
        "polygon": np.asarray(result["polygon"], dtype=float).round(6).tolist(),
        "hull": np.asarray(result["hull"], dtype=float).round(6).tolist(),
        "records": result["records"],
    }
    temporary = path.with_name(path.name + ".tmp")
    try:
        temporary.write_text(json.dumps(payload), encoding="utf-8")
        os.replace(temporary, path)
    except OSError as exc:  # a read-only home must not break the launch
        log(f"[cache] could not store {path.name}: {exc}")


# --------------------------------------------------------------------------- #
# Pipeline
# --------------------------------------------------------------------------- #
def compute_polygon(urdf_path: Path,
                    padding: float = 0.0,
                    quantize: float = 1e-3,
                    simplify: float = 2e-3,
                    prefer: str = "collision",
                    log=print,
                    max_points: int = 4_000_000,
                    use_cache: bool = True) -> dict:
    """Project ``urdf_path`` onto the ground plane and return its footprint.

    The result is a counter-clockwise convex polygon in the ``base_footprint``
    frame, grown by ``padding`` metres.  Raises ``ValueError`` when the model
    carries no usable geometry or the outline fails the sanity checks, so a
    caller (``main`` or the navigation launch file) can fall back cleanly.
    """
    urdf_path = Path(urdf_path).expanduser().resolve()
    if not urdf_path.is_file():
        raise ValueError(f"{urdf_path} not found")
    if padding < 0.0:
        raise ValueError("padding must be >= 0")

    cache_file = None
    if use_cache:
        directory = _cache_directory()
        if directory is not None:
            key = _cache_key(urdf_path, prefer, quantize, simplify, padding)
            cache_file = directory / f"{key}.json"
            cached = _load_cached(cache_file, urdf_path, padding)
            if cached is not None:
                log(f"[cache] hit {cache_file.name} ({len(cached['polygon'])} "
                    f"vertices, {cached['area']:.4f} m^2, padding {padding:.3f} m)")
                return cached

    points, records = load_urdf(urdf_path, prefer, log)
    if not len(points):
        raise ValueError(f"{urdf_path} references no usable geometry")

    unique = unique_2d(points, quantize)
    if len(unique) > max_points:
        raise ValueError(
            f"{len(unique)} unique vertices exceed max_points ({max_points:,})")
    exact = drop_collinear(convex_hull(unique))
    hull = simplify_convex(exact, simplify)
    polygon = offset_convex(hull, padding)

    if not contains(polygon, unique, eps=simplify + 1e-6):
        raise ValueError("hull does not contain every vertex (numerical blow-up)")
    area = signed_area(polygon)
    if not (0.01 < area < 25.0):
        raise ValueError(f"implausible footprint area {area:.4f} m^2")
    radius = float(np.max(np.linalg.norm(polygon, axis=1)))
    if not (0.05 < radius < 5.0):
        raise ValueError(f"implausible footprint radius {radius:.3f} m")

    result = {
        "urdf": str(urdf_path),
        "points": int(len(points)),
        "unique_points": int(len(unique)),
        "exact_vertices": int(len(exact)),
        "padding": float(padding),
        "polygon": polygon,
        "hull": hull,
        "area": float(area),
        "radius": radius,
        "records": records,
        "cached": False,
    }
    if cache_file is not None:
        _store_cached(cache_file, result, log)
    return result


def format_footprint(polygon: np.ndarray) -> str:
    """Render a polygon as the Nav2 ``footprint`` string."""
    return "[" + ", ".join(f"[{x:.4f}, {y:.4f}]" for x, y in polygon) + "]"


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def main() -> int:
    parser = argparse.ArgumentParser(
        description="Derive a Nav2 footprint from a URDF.")
    parser.add_argument("urdf", type=Path, help="path to the .urdf file")
    parser.add_argument("--padding", type=float, default=0.0,
                        help="grow the hull by this many metres (default 0)")
    parser.add_argument("--quantize", type=float, default=1e-3,
                        help="round projected points to this grid (default 1 mm)")
    parser.add_argument("--simplify", type=float, default=0.002,
                        help="drop hull vertices that move the outline less "
                             "than this (default 2 mm)")
    parser.add_argument("--max-points", type=int, default=4_000_000,
                        help="abort if the meshes carry more vertices than this")
    parser.add_argument("--lidar", default="",
                        help="optional 'x,y' marker for the ASCII plot")
    parser.add_argument("--ascii", action="store_true", help="draw the outline")
    parser.add_argument("--verbose", action="store_true", help="per-mesh detail")
    parser.add_argument("--json", action="store_true", help="machine readable output")
    parser.add_argument("--no-cache", action="store_true",
                        help="recompute even when a cached outline exists")
    args = parser.parse_args()

    urdf_path = args.urdf.expanduser().resolve()
    if not urdf_path.is_file():
        print(f"error: {urdf_path} not found", file=sys.stderr)
        return 1
    if args.padding < 0.0:
        print("error: --padding must be >= 0", file=sys.stderr)
        return 1

    log = (lambda *args, **kwargs: None) if args.json else print
    try:
        result = compute_polygon(urdf_path, args.padding, args.quantize,
                                 args.simplify, "collision", log,
                                 args.max_points, use_cache=not args.no_cache)
    except ValueError as exc:
        message = str(exc)
        print(f"error: {message}", file=sys.stderr)
        return 2 if message.startswith(("hull", "implausible")) else 1

    polygon = result["polygon"]
    area = result["area"]
    radius = result["radius"]
    footprint = format_footprint(polygon)

    if args.verbose:
        for record in result["records"]:
            x_min, x_max, y_min, y_max = record["xy"]
            print(f"[mesh] {record['link']:<12s} {record['source']:<9s} "
                  f"{record['mesh']:<28s} tris={record['triangles']:>8d} "
                  f"xy=[{x_min:+.3f}, {x_max:+.3f}] x [{y_min:+.3f}, {y_max:+.3f}]")

    if args.json:
        import json

        print(json.dumps({
            "urdf": str(urdf_path),
            "points": result["points"],
            "unique_points": result["unique_points"],
            "padding": args.padding,
            "vertices": [[round(float(x), 4), round(float(y), 4)] for x, y in polygon],
            "area": round(area, 4),
            "radius": round(radius, 4),
            "cached": bool(result["cached"]),
            "footprint": footprint,
        }, indent=2))
        return 0

    print(f"[input]  {urdf_path}")
    print(f"[points] {result['points']:,} vertices -> {result['unique_points']:,} "
          f"unique on a {args.quantize * 1000:g} mm grid")
    print(f"[hull]   {result['exact_vertices']} exact vertices -> "
          f"{len(result['hull'])} after {args.simplify * 1000:g} mm simplification, "
          f"padding {args.padding:+.3f} m -> {len(polygon)} vertices")
    print(f"[shape]  area {area:.4f} m^2, max radius {radius:.3f} m, "
          f"extent x [{polygon[:, 0].min():+.3f}, {polygon[:, 0].max():+.3f}] "
          f"y [{polygon[:, 1].min():+.3f}, {polygon[:, 1].max():+.3f}]")
    assert signed_area(polygon) > 0, "polygon must be counter-clockwise"
    for index, (x, y) in enumerate(polygon, start=1):
        print(f"  {index:2d}. x={x:+.4f}  y={y:+.4f}")
    print(f"\n[footprint]\n{footprint}")

    if args.ascii:
        marker = None
        if args.lidar:
            marker = tuple(float(value) for value in args.lidar.split(","))
        print("\n[outline]  '#' hull, 'O' base_footprint, 'L' lidar")
        print(ascii_plot(polygon, marker))
    return 0


if __name__ == "__main__":
    sys.exit(main())
