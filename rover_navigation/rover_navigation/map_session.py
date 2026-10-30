"""Resolve replaceable map artifacts without requiring a per-map profile."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

from rover_navigation.navigation_profile import load_profile


def read_mapping(filename):
    data = yaml.safe_load(Path(filename).read_text())
    if not isinstance(data, dict):
        raise ValueError(f'Expected YAML mapping: {filename}')
    return data


def select_session(
    mode,
    *,
    map_dir='',
    profile='',
    database='',
    occupancy='',
    params_file='',
    localization_file='',
    route_required=False,
):
    """CLI overrides profile/session paths; relative paths belong to that session."""
    share = Path(get_package_share_directory('rover_navigation'))
    defaults = read_mapping(share / 'config/rtab_runtime.yaml')
    if map_dir and profile:
        raise ValueError('Select map_dir or profile, not both')
    if map_dir:
        directory = Path(map_dir).expanduser().resolve()
        if not directory.is_dir():
            raise ValueError(f'Missing map directory: {directory}')
        filename = directory / 'profile.yaml'
    elif profile:
        filename = Path(profile).expanduser().resolve()
        if not filename.is_file():
            raise ValueError(f'Missing profile: {filename}')
        directory = filename.parent
    elif database or occupancy:
        directory = Path.cwd()
        filename = directory / '.explicit-map-selection.yaml'
    else:
        filename = Path('~/ros2_ws/maps/current.yaml').expanduser().resolve()
        directory = filename.parent

    selected = read_mapping(filename) if filename.is_file() else {}
    data = {**defaults['profile'], **selected}

    def path(value):
        target = Path(value).expanduser()
        return str((directory / target).resolve())

    def conventional(names):
        found = [directory / name for name in names if (directory / name).is_file()]
        if len(found) != 1:
            raise ValueError(
                f'Select an explicit artifact; expected one of {names} in {directory}'
            )
        return str(found[0])

    for key, override, names in (
        ('map', occupancy, ('rtabmap.yaml', 'map.yaml')),
        ('database', database, ('rtabmap.db', 'map.db')),
    ):
        if key == 'database' and mode == 'planner' and not override:
            continue
        choice = override or data.get(key)
        data[key] = path(choice) if choice else conventional(names)

    def config(override, key, local_name, default_name):
        choice = override or selected.get(key)
        result = (
            Path(path(choice))
            if choice
            else (
                directory / local_name
                if map_dir and (directory / local_name).is_file()
                else share / 'config' / default_name
            )
        )
        if not result.is_file():
            raise ValueError(f'Missing runtime configuration: {result}')
        return str(result)

    params = config(params_file, 'params_file', 'nav2.yaml', 'nav2_rtab.yaml')
    localization = config(
        localization_file, 'localization_file', 'localization.yaml', 'rtab_runtime.yaml'
    )
    resolved = load_profile(
        str(filename), mode, params_file=params, route_required=route_required, profile_data=data
    )
    return resolved, params, localization, str(filename)
