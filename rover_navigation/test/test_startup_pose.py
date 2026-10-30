"""A startup hint belongs only to the map and base frame that were verified."""

import pytest
import yaml

from rover_navigation.startup_pose import load_startup_pose, map_identity


@pytest.fixture
def saved(tmp_path):
    db, occupancy, hint = [tmp_path / name for name in ('map.db', 'map.yaml', 'startup_pose.yaml')]
    db.write_bytes(b'map identity fixture')
    occupancy.write_text('resolution: 0.05\norigin: [0, 0, 0]\n')
    data = {'enabled': True, 'frame_id': 'map', 'base_frame_id': 'base_link',
            'source': map_identity(db, occupancy), 'pose': [1, 2, 0, 0, 0, -0.8]}
    hint.write_text(yaml.safe_dump(data))
    return db, occupancy, hint, data


def test_matching_map_loads_six_dof_hint(saved):
    db, occupancy, hint, _ = saved
    pose, message = load_startup_pose(hint, db, occupancy)
    assert [float(v) for v in pose.split()] == [1, 2, 0, 0, 0, -0.8]
    assert 'hint' in message


@pytest.mark.parametrize('change', ['db', 'map', 'frame', 'nan', 'count', 'bool', 'disabled'])
def test_changed_map_or_invalid_hint_is_ignored(saved, change):
    db, occupancy, hint, data = saved
    if change == 'db':
        db.write_bytes(b'new map')
    elif change == 'map':
        occupancy.write_text('origin: [50, 0, 0]')
    elif change == 'frame':
        data['base_frame_id'] = 'camera_link'
    elif change == 'nan':
        data['pose'][0] = float('nan')
    elif change == 'count':
        data['pose'] = [1, 2, 3]
    elif change == 'bool':
        data['pose'][0] = True
    else:
        data['enabled'] = False
    hint.write_text(yaml.safe_dump(data))
    assert load_startup_pose(hint, db, occupancy)[0] == ''


def test_missing_or_malformed_hint_does_not_prevent_startup(saved):
    db, occupancy, hint, _ = saved
    hint.unlink()
    assert load_startup_pose(hint, db, occupancy) == ('', '')
    hint.write_text('pose: [broken')
    assert load_startup_pose(hint, db, occupancy)[0] == ''
