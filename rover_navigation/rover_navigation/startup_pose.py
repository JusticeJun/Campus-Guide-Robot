"""Map-specific initial guess; never substitutes for native registration evidence."""

import hashlib
import math
from pathlib import Path

import yaml


def map_identity(database, occupancy):
    """Cheap source identity: never hash or open the multi-GB live database."""
    database = Path(database).resolve()
    stat = database.stat()
    return {
        'database': str(database),
        'size': stat.st_size,
        'mtime_ns': stat.st_mtime_ns,
        'occupancy_sha256': hashlib.sha256(Path(occupancy).read_bytes()).hexdigest(),
    }


def load_startup_pose(filename, database, occupancy):
    """Return an RTAB initial_pose string and diagnostic; ignore invalid old hints."""
    filename = Path(filename)
    if not filename.is_file():
        return '', ''
    try:
        data = yaml.safe_load(filename.read_text())
        if not isinstance(data, dict):
            raise ValueError('expected a YAML mapping')
        if data.get('enabled') is False:
            return '', 'Startup pose disabled in ' + str(filename)
        if data.get('enabled') is not True:
            raise ValueError('enabled must be a boolean')
        if data.get('frame_id') != 'map' or data.get('base_frame_id') != 'base_link':
            raise ValueError('expected map -> base_link')
        if data.get('source') != map_identity(database, occupancy):
            raise ValueError('source map changed; capture a new startup pose')
        pose = data.get('pose')
        if not isinstance(pose, list) or len(pose) != 6 or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
            for v in pose
        ):
            raise ValueError('pose must contain finite x y z roll pitch yaw')
        return ' '.join(format(v, '.12g') for v in pose), 'Startup pose hint: ' + str(filename)
    except (OSError, ValueError, TypeError, yaml.YAMLError) as error:
        return '', f'Ignoring startup pose {filename}: {error}'
