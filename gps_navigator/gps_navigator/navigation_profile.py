"""Plain YAML selection and calibration binding for replaceable map artifacts."""

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

from gps_navigator.geo_utils import EARTH_RADIUS_M, gps_to_local_xy


def finite_vector(value, size, label):
    if (
        not isinstance(value, list)
        or len(value) != size
        or any(
            isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x)
            for x in value
        )
    ):
        raise ValueError(f'{label} requires {size} finite numbers')
    return value


def file_hash(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def profile_fingerprint(profile, params_file):
    """Bind one-time commissioning to data, graph, extrinsic and configuration."""
    data = copy.deepcopy(profile)
    for key in ('validated_fingerprint', '_fingerprint'):
        data.pop(key, None)
    # Content, not pathname: moving a complete version directory is harmless.
    for key in ('map', 'image', 'database', 'waypoints_file'):
        data[key] = file_hash(profile[key])
    data['nav2_parameters'] = file_hash(params_file)
    return hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()


def load_profile(
    filename,
    mode='planner',
    map_override='',
    params_file=None,
    route_required=True,
    profile_data=None,
):
    if mode not in ('planner', 'validation', 'drive'):
        raise ValueError('mode must be planner, validation or drive')
    path = Path(filename).expanduser().resolve()
    profile = (
        copy.deepcopy(profile_data)
        if profile_data is not None
        else yaml.safe_load(path.read_text()) if path.is_file() else {}
    )
    if not isinstance(profile, dict):
        raise ValueError('navigation profile must be a mapping')
    if map_override and mode != 'planner':
        raise ValueError('map override is planner-only; select a matching profile')

    def artifact(value):
        if not isinstance(value, str) or not value:
            raise ValueError('Missing navigation artifact path')
        target = Path(value).expanduser()
        if not target.is_absolute():
            target = path.parent / target
        if not target.is_file():
            raise ValueError(f'Missing navigation artifact: {target}')
        return str(target.resolve())

    selected = map_override or profile.get('map')
    if not selected:
        raise ValueError('Select profile:=... or supply map:=... in planner mode')
    profile['map'] = artifact(selected)
    metadata = yaml.safe_load(Path(profile['map']).read_text())
    if not isinstance(metadata, dict):
        raise ValueError('occupancy YAML must be a mapping')
    finite_vector(metadata.get('origin'), 3, 'occupancy origin')
    if metadata['origin'][2] != 0:
        raise ValueError('Nav2 StaticLayer requires an axis-aligned map origin')
    image = Path(metadata['image']).expanduser()
    if not image.is_absolute():
        image = Path(profile['map']).parent / image
    profile['image'] = artifact(str(image))
    if mode == 'planner':
        return profile
    profile['database'] = artifact(profile.get('database'))
    share = Path(get_package_share_directory('gps_navigator'))
    if not route_required:
        # Map-coordinate action clients do not depend on geographic route data.
        # Explicit drive selection connects the adapter; runtime guard still gates motion.
        finite_vector(profile.get('base_to_camera'), 6, 'base_to_camera xyz rpy')
        if profile.get('odometry_provider') != 'rgbd_odometry':
            raise ValueError('This version requires rgbd_odometry')
        for topic in ('rgb_topic', 'depth_topic', 'camera_info_topic'):
            if not isinstance(profile.get(topic), str) or not profile[topic].startswith('/'):
                raise ValueError(f'Missing absolute {topic}')
        return profile
    profile['waypoints_file'] = artifact(
        profile.get('waypoints_file', str(share / 'config/waypoints.yaml'))
    )
    alignment = profile.get('alignment') or {}
    finite_vector(alignment.get('datum'), 2, 'alignment.datum [lat, lon]')
    finite_vector(alignment.get('enu_to_map'), 3, 'enu_to_map [x, y, yaw]')
    lat, lon = alignment['datum']
    if not (-90 < lat < 90 and -180 <= lon <= 180):
        raise ValueError('invalid datum')
    if alignment.get('reviewed') is not True:
        raise ValueError('ENU/map alignment requires surveyed-anchor review')
    finite_vector(profile.get('base_to_camera'), 6, 'base_to_camera xyz rpy')
    width = profile.get('route_half_width_m')
    if not isinstance(width, (int, float)) or not math.isfinite(width) or width <= 0:
        raise ValueError('route_half_width_m must be positive')
    for topic in ('rgb_topic', 'depth_topic', 'camera_info_topic'):
        if not isinstance(profile.get(topic), str) or not profile[topic].startswith('/'):
            raise ValueError(f'Missing absolute {topic}')
    if profile.get('odometry_provider') != 'rgbd_odometry':
        raise ValueError('This version supports only rgbd_odometry as local TF owner')
    graph = yaml.safe_load(Path(profile['waypoints_file']).read_text())['waypoints']
    covered_graph(graph, profile)
    params_file = params_file or str(share / 'config/nav2_rtab.yaml')
    profile['_fingerprint'] = profile_fingerprint(profile, params_file)
    if mode == 'drive':
        if profile.get('odometry_qualified') is not True:
            raise ValueError('Drive blocked: RGB-D odometry has not been qualified')
        if profile.get('validated_fingerprint') != profile['_fingerprint']:
            raise ValueError('Drive blocked: map/calibration/config changed or not commissioned')
    return profile


def gps_to_map(latitude, longitude, alignment):
    east, north = gps_to_local_xy(latitude, longitude, *alignment['datum'])
    x, y, yaw = alignment['enu_to_map']
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * east - s * north, y + s * east + c * north


def map_to_gps(x, y, alignment):
    tx, ty, yaw = alignment['enu_to_map']
    c, s = math.cos(yaw), math.sin(yaw)
    east, north = c * (x - tx) + s * (y - ty), -s * (x - tx) + c * (y - ty)
    lat0, lon0 = alignment['datum']
    lat = lat0 + math.degrees(north / EARTH_RADIUS_M)
    lon = lon0 + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians((lat + lat0) / 2))))
    return lat, lon


def covered_graph(waypoints, profile):
    """Preserve IDs and graph semantics; optionally correct nodes in fixed ENU."""
    nodes = profile.get('covered_nodes') or list(waypoints)
    if any(node not in waypoints for node in nodes):
        raise ValueError('Unknown covered graph node')
    edges = profile.get('reviewed_edges', [])
    for edge in edges:
        if (
            not isinstance(edge, list)
            or len(edge) != 2
            or edge[0] not in nodes
            or edge[1] not in nodes
            or edge[1] not in waypoints[edge[0]]['neighbors']
        ):
            raise ValueError('Reviewed edge must exist in covered campus graph')
    result = {
        key: {**waypoints[key], 'neighbors': [target for start, target in edges if start == key]}
        for key in nodes
    }
    for key, offset in profile.get('node_offsets_enu', {}).items():
        if key not in result:
            raise ValueError('Node offset is outside covered graph')
        finite_vector(offset, 2, 'node_offsets_enu east/north offset')
        alignment = {**profile['alignment'], 'enu_to_map': [0.0, 0.0, 0.0]}
        node = result[key]
        x, y = gps_to_map(node['latitude'], node['longitude'], alignment)
        node['latitude'], node['longitude'] = map_to_gps(x + offset[0], y + offset[1], alignment)
    return result


def main():
    parser = argparse.ArgumentParser(
        description=('Print commissioning fingerprint; does not approve or change a profile.')
    )
    parser.add_argument('--profile', required=True)
    parser.add_argument('--params-file')
    args = parser.parse_args()
    profile = load_profile(args.profile, 'validation', params_file=args.params_file)
    print('validated_fingerprint: ' + profile['_fingerprint'])
