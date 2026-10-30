"""Offline transaction tests: preserve rollback data even if FCU acknowledgement fails."""

import importlib.util
import json
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location(
    'fcu_externalnav_tool', Path(__file__).resolve().parents[2] / 'tools/fcu_externalnav.py'
)
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)


class FakeFCU:
    """Simulate persistent FCU parameters, including a write with missing acknowledgement."""

    def __init__(self):
        self.identity = {'uid2': [1, 2], 'flight_sw_version': 67568127}
        self.values = {
            name: {'value': float(value), 'type': 9} for name, value in tool.FCU_PROFILE.items()
        }
        self.values['GPS1_TYPE']['value'] = 1.0
        self.values['VISO_TYPE']['value'] = 0.0
        self.values['SERVO3_MAX'] = {'value': 1700.0, 'type': 4}
        self.writes = []
        self.fail = False

    def snapshot(self):
        return self.values

    def get(self, key):
        return dict(self.values[key])

    def set(self, key, value, kind):
        self.values[key] = {'value': value, 'type': kind}
        self.writes.append(key)
        if self.fail:
            self.fail = False
            raise RuntimeError('lost acknowledgement after FCU stored the value')


def args(action, directory):
    from types import SimpleNamespace

    return SimpleNamespace(action=action, directory=str(directory))


def test_write_failure_has_durable_undo_and_rolls_back_only_touched_values(tmp_path):
    fcu = FakeFCU()
    tool.run(args('backup', tmp_path), fcu)
    fcu.fail = True
    with pytest.raises(RuntimeError, match='lost acknowledgement'):
        tool.run(args('apply', tmp_path), fcu)
    journal = json.loads((tmp_path / 'journal.json').read_text())
    assert journal['undo']['GPS1_TYPE']['value'] == 1.0
    assert journal['events'][-1]['status'] == 'intent'
    assert fcu.values['GPS1_TYPE']['value'] == 0.0
    tool.run(args('rollback', tmp_path), fcu)
    assert fcu.values['GPS1_TYPE']['value'] == 1.0
    assert 'SERVO3_MAX' not in fcu.writes


def test_enable_apply_rollback_restores_original_viso_and_preserves_gains(tmp_path):
    fcu = FakeFCU()
    original = dict(fcu.values['SERVO3_MAX'])
    tool.run(args('backup', tmp_path), fcu)
    tool.run(args('enable', tmp_path), fcu)
    tool.run(args('apply', tmp_path), fcu)
    tool.run(args('verify', tmp_path), fcu)
    tool.run(args('rollback', tmp_path), fcu)
    assert fcu.values['GPS1_TYPE']['value'] == 1.0
    assert fcu.values['VISO_TYPE']['value'] == 0.0
    assert fcu.values['SERVO3_MAX'] == original
    assert fcu.writes[-1] == 'VISO_TYPE'
    tool.run(args('verify-rollback', tmp_path), fcu)


@pytest.mark.parametrize('change', ['identity', 'backup', 'live'])
def test_rejects_wrong_device_corrupt_backup_or_changed_baseline(tmp_path, change):
    fcu = FakeFCU()
    tool.run(args('backup', tmp_path), fcu)
    if change == 'identity':
        fcu.identity = {'uid2': [99], 'flight_sw_version': 67568127}
    elif change == 'backup':
        with (tmp_path / 'original.json').open('a') as stream:
            stream.write(' ')
    else:
        fcu.values['GPS1_TYPE']['value'] = 2.0
    with pytest.raises(RuntimeError):
        tool.run(args('apply', tmp_path), fcu)
    assert not fcu.writes


def test_origin_is_journalled_restored_and_actuator_drift_is_detected(tmp_path):
    fcu = FakeFCU()
    for key, value in {
        'AHRS_OPTIONS': 3.0,
        'AHRS_ORIGIN_LAT': 0.0,
        'AHRS_ORIGIN_LON': 0.0,
        'AHRS_ORIGIN_ALT': 0.0,
    }.items():
        fcu.values[key] = {'value': value, 'type': 9}
    tool.run(args('backup', tmp_path), fcu)
    request = args('origin', tmp_path)
    request.origin_lat, request.origin_lon, request.origin_alt_m = 35.134843, 129.1038, 0.0
    tool.run(request, fcu)
    assert fcu.values['AHRS_OPTIONS']['value'] == 19.0
    tool.run(args('apply', tmp_path), fcu)
    tool.run(args('verify', tmp_path), fcu)
    fcu.values['SERVO3_MAX']['value'] = 1900.0
    with pytest.raises(RuntimeError, match='SERVO3_MAX'):
        tool.run(args('verify', tmp_path), fcu)
    tool.run(args('rollback', tmp_path), fcu)
    assert fcu.values['AHRS_OPTIONS']['value'] == 3.0
    assert fcu.values['AHRS_ORIGIN_LAT']['value'] == 0.0
    assert fcu.values['SERVO3_MAX']['value'] == 1900.0  # Unrelated edits are not overwritten.
