import json

import pytest

from rover_navigation.drive_trace_node import BoundedTrace, resource_sample


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


def test_resource_sample_uses_available_ram_and_reports_pressure_without_controls(tmp_path):
    (tmp_path / 'meminfo').write_text(
        'MemTotal: 8000000 kB\nMemFree: 1000 kB\n'
        'MemAvailable: 1500000 kB\nSwapFree: 2000000 kB\n')
    (tmp_path / 'loadavg').write_text('3.20 2.00 1.00 1/2 10\n')
    (tmp_path / 'pressure').mkdir()
    (tmp_path / 'pressure/memory').write_text(
        'some avg10=1.50 avg60=0.00 avg300=0.00 total=100\n')
    result = resource_sample(tmp_path)
    assert result['MemAvailable_kib'] == 1500000
    assert result['memory_some_avg10_pct'] == 1.5
    assert result['load_1m'] == 3.2
