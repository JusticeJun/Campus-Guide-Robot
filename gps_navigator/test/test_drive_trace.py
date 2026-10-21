import json

import pytest

from gps_navigator.drive_trace_node import BoundedTrace


def test_trace_rotates_with_global_bound_across_sessions(tmp_path):
    for session in range(3):
        trace = BoundedTrace(tmp_path, max_bytes=256, backups=3)
        for sequence in range(40):
            trace.write({'session': session, 'sequence': sequence, 'value': 'x' * 40})
        trace.close()
    files = list(tmp_path.glob('trace.jsonl*'))
    assert len(files) == 4
    assert sum(p.stat().st_size for p in files) <= 4 * 256
    latest = [json.loads(line) for line in (tmp_path / 'trace.jsonl').read_text().splitlines()]
    assert latest[-1]['session'] == 2 and latest[-1]['sequence'] == 39
    for path in files:
        assert path.stat().st_size <= 256
        for line in path.read_text().splitlines():
            json.loads(line)


def test_trace_prevents_concurrent_writers_and_releases_lock(tmp_path):
    first = BoundedTrace(tmp_path)
    with pytest.raises(RuntimeError, match='already running'):
        BoundedTrace(tmp_path)
    first.close()
    second = BoundedTrace(tmp_path)
    second.close()


def test_trace_preserves_nonfinite_diagnostic_values_as_valid_json(tmp_path):
    trace = BoundedTrace(tmp_path, max_bytes=256)
    trace.write({'velocity': float('nan'), 'covariance': [float('inf')]})
    with pytest.raises(ValueError, match='limit'):
        trace.write({'huge': 'x' * 300})
    trace.close()
    record = json.loads((tmp_path / 'trace.jsonl').read_text())
    assert record == {'velocity': 'nan', 'covariance': ['inf']}
