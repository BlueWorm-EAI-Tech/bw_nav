from dataclasses import asdict, dataclass, replace
import fcntl
import os
from pathlib import Path
import tempfile
import uuid

import yaml

from .geometry import Zone, rasterize, triangles


class ZoneError(ValueError):
    def __init__(self, code, message):
        self.code = code
        super().__init__(message)


@dataclass(frozen=True)
class Snapshot:
    revision: int
    zones: tuple
    mask: bytes
    fills: tuple


class Store:
    def __init__(self, spec, path=None):
        self.spec = spec
        self.path = Path(path).expanduser().resolve() if path else spec.config_path
        self.required = bool(path)
        self.snapshot = self.read()

    def prepare(self, revision, zones):
        if type(revision) is not int or not 0 <= revision < 2**64:
            raise ZoneError('INVALID_ARGUMENT', 'Invalid revision')
        try:
            zones = tuple(zone.normalized(self.spec) for zone in zones)
            if len({zone.id for zone in zones}) != len(zones):
                raise ZoneError('ALREADY_EXISTS', 'Duplicate zone id')
            fills = tuple(tuple(triangles(z.vertices)) if z.enabled else () for z in zones)
            return Snapshot(revision, zones, rasterize(self.spec, zones), fills)
        except ZoneError:
            raise
        except (ValueError, TypeError, OverflowError) as error:
            raise ZoneError('INVALID_ARGUMENT', str(error)) from error

    def read(self):
        try:
            data = yaml.safe_load(self.path.read_bytes())
        except FileNotFoundError:
            if not self.required:
                return self.prepare(0, ())
            raise
        if (not isinstance(data, dict) or data.get('version') != 1 or
                data.get('frame_id') != 'map' or data.get('cost') != 252):
            raise ZoneError('INVALID_ARGUMENT', 'Invalid planning zone file format')
        meta = data.get('map')
        if not isinstance(meta, dict) or any(meta.get(k) != v for k, v in self.spec.metadata.items()):
            raise ZoneError('MAP_MISMATCH', 'Zone configuration does not match this map')
        entries = data.get('zones')
        try:
            if not isinstance(entries, list):
                raise TypeError('zones must be a list')
            zones = [Zone(**entry) for entry in entries]
        except TypeError as error:
            raise ZoneError('INVALID_ARGUMENT', str(error)) from error
        return self.prepare(data.get('revision', 0), zones)

    def _write(self, snapshot):
        data = dict(version=1, revision=snapshot.revision, frame_id='map', cost=252,
                    map=dict(self.spec.metadata, yaml=self.spec.path.name),
                    zones=[asdict(z) for z in snapshot.zones])
        tmp = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=self.path.parent,
                                             prefix=f'.{self.path.name}.', delete=False) as f:
                tmp = Path(f.name)
                yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.path)
        finally:
            if tmp is not None:
                tmp.unlink(missing_ok=True)

    def mutate(self, operation, map_id, revision, *, zone=None, zone_id='', zones=(),
               enabled=False, before_commit=lambda _: None):
        if map_id != self.spec.id:
            raise ZoneError('MAP_MISMATCH', 'Selected map does not match server map')
        if revision != self.snapshot.revision:
            raise ZoneError('REVISION_CONFLICT', 'Zone revision changed; read the latest state')
        current = {entry.id: entry for entry in self.snapshot.zones}
        created = None
        with self.path.with_suffix(self.path.suffix + '.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            disk = self.read()
            if operation == 'reload':
                proposed = disk.zones
            else:
                if (disk.revision, disk.zones) != (revision, self.snapshot.zones):
                    raise ZoneError('REVISION_CONFLICT', 'Configuration changed on disk; reload first')
                if operation == 'create':
                    created = replace(zone, id=zone.id or uuid.uuid4().hex)
                    if created.id in current:
                        raise ZoneError('ALREADY_EXISTS', 'Zone id already exists')
                    current[created.id] = created
                elif operation in ('delete', 'update', 'set_enabled'):
                    key = zone.id if operation == 'update' else zone_id
                    if key not in current:
                        raise ZoneError('NOT_FOUND', 'Zone id not found')
                    if operation == 'delete':
                        del current[key]
                    elif operation == 'update':
                        current[key] = zone
                    else:
                        current[key] = replace(current[key], enabled=enabled)
                elif operation != 'replace_all':
                    raise ZoneError('INVALID_ARGUMENT', 'Unknown operation')
                proposed = zones if operation == 'replace_all' else current.values()
            candidate = self.prepare(max(revision, disk.revision) + 1, proposed)
            if (candidate.zones == self.snapshot.zones and
                    (disk.revision, disk.zones) == (revision, self.snapshot.zones)):
                return None
            before_commit(candidate)
            self._write(candidate)
            self.snapshot = candidate
            if created is not None:
                return next(z for z in candidate.zones if z.id == created.id)
            return None
