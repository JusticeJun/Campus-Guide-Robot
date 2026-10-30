"""RTAB checkpoint selection must preserve map payload and recover atomically."""

import sqlite3
import struct
import shutil
import sys
import zlib

import pytest

from rover_navigation.localization_memory import (
    main, optimized_positions, restore_reverse_links, seed_working_memory,
)


def matrix(values, scalar, kind):
    return (zlib.compress(struct.pack('<' + scalar * len(values), *values))
            + struct.pack('<iii', 1, len(values), kind))


def fixture_db(path):
    ids = list(range(1, 81))
    poses = []
    for node in ids:
        x = node / 10 if node <= 40 else 100 + node
        poses.extend([1, 0, 0, x, 0, 1, 0, 0, 0, 0, 1, 0])
    with sqlite3.connect(path) as db:
        db.executescript('''
            CREATE TABLE Admin(version TEXT,opt_ids BLOB,opt_poses BLOB,dictionary_index BLOB);
            CREATE TABLE Node(id INTEGER PRIMARY KEY, weight INTEGER, pose BLOB,time_enter TEXT);
            CREATE TABLE Word(id INTEGER PRIMARY KEY,descriptor BLOB,time_enter TEXT);
            CREATE TABLE Feature(node_id INTEGER,word_id INTEGER,descriptor BLOB);
            CREATE TABLE Info(dictionary_size INTEGER,time_enter TEXT);
            CREATE TABLE Link(from_id INTEGER,to_id INTEGER,type INTEGER,
                              transform BLOB,information_matrix BLOB,user_data BLOB);
            CREATE TABLE Data(id INTEGER,image BLOB);
            INSERT INTO Info VALUES(80,'2026-10-22 10:00:00');
        ''')
        db.execute('INSERT INTO Admin VALUES(?,?,?,?)',
                   ('0.23.7', matrix(ids, 'i', 4), matrix(poses, 'f', 5), b'old index'))
        for node in ids:
            # Raw odometry pose is deliberately different from optimized map pose.
            db.execute('INSERT INTO Node VALUES(?,0,?,?)',
                       (node, b'wrong session-local coordinates', '2026-10-22 10:00:00'))
            db.execute('INSERT INTO Word VALUES(?,?,?)',
                       (node, b'descriptor', '2026-10-22 10:00:00'))
            db.execute('INSERT INTO Feature VALUES(?,?,?)', (node, node, b'local descriptor'))
            transform = struct.pack('<12f', 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0)
            information = struct.pack('<36d', *[float(i % 7 == 0) for i in range(36)])
            for a, b in ((node, node + 1), (node + 1, node)):
                db.execute('INSERT INTO Link VALUES(?,?,0,?,?,NULL)',
                           (a, b, transform, information))
            db.execute('INSERT INTO Data VALUES(?,?)', (node, b'image depth calibration'))


def payload(path):
    with sqlite3.connect(path) as db:
        return [db.execute(sql).fetchall() for sql in (
            'SELECT id,weight,pose FROM Node', 'SELECT id,descriptor FROM Word',
            'SELECT * FROM Feature', 'SELECT * FROM Link WHERE type!=7', 'SELECT * FROM Data')]


def test_local_checkpoint_retains_entire_map_and_can_be_reseeded(tmp_path):
    path = tmp_path / 'private.db'
    fixture_db(path)
    before = payload(path)
    with sqlite3.connect(path) as db:
        graph = db.execute('SELECT opt_ids,opt_poses FROM Admin').fetchone()
    report = seed_working_memory(path, [0] * 6, graph=graph)
    assert report['seeded'] and report['nodes'] == 40 and report['words'] == 40
    assert report['reference_priors'] == 80
    assert payload(path) == before
    with sqlite3.connect(path) as db:
        selected = db.execute('SELECT id FROM Node WHERE time_enter>= '
                              '(SELECT max(time_enter) FROM Info)').fetchall()
        assert selected == [(i,) for i in range(1, 41)]
        assert db.execute('SELECT dictionary_index FROM Admin').fetchone() == (None,)
        poses = optimized_positions(*db.execute('SELECT opt_ids,opt_poses FROM Admin').fetchone())
        assert len(poses) == 40 and poses[40] == (4, 0, 0)
        reference = db.execute('SELECT transform FROM Link WHERE type=7 AND from_id=80').fetchone()
        assert struct.unpack('<12f', reference[0])[3] == 180
    assert seed_working_memory(path, [0] * 6, graph=graph) == report
    assert payload(path) == before


@pytest.mark.parametrize('damage', [
    'UPDATE Admin SET opt_ids=NULL,opt_poses=NULL',
    'DELETE FROM Word WHERE id=1',
])
def test_unusable_checkpoint_falls_back_without_partial_changes(tmp_path, damage):
    path = tmp_path / 'private.db'
    fixture_db(path)
    with sqlite3.connect(path) as db:
        db.execute(damage)
    before = path.read_bytes()
    assert not seed_working_memory(path, [0] * 6)['seeded']
    assert path.read_bytes() == before


def test_failure_rolls_back_all_membership_updates(tmp_path):
    path = tmp_path / 'private.db'
    fixture_db(path)
    with sqlite3.connect(path) as db:
        db.execute("CREATE TRIGGER fail_update BEFORE UPDATE ON Word "
                   "BEGIN SELECT RAISE(ABORT, 'test failure'); END")
    before = path.read_bytes()
    with pytest.raises(sqlite3.IntegrityError, match='test failure'):
        seed_working_memory(path, [0] * 6)
    assert path.read_bytes() == before


def test_pose_decoder_rejects_bad_matrix():
    with pytest.raises(ValueError):
        optimized_positions(matrix([1, 1], 'i', 4), matrix([0] * 24, 'f', 5))
    with pytest.raises(ValueError):
        optimized_positions(matrix([1], 'i', 4), matrix([float('nan')] * 12, 'f', 5))


def test_incomplete_global_checkpoint_falls_back_before_seeding(tmp_path):
    path = tmp_path / 'private.db'
    fixture_db(path)
    with sqlite3.connect(path) as db:
        ids, poses = db.execute('SELECT opt_ids,opt_poses FROM Admin').fetchone()
        # A far node is absent from the optimized checkpoint, even though the
        # start neighborhood is complete. It must not fail only upon retrieval.
        ids_raw, poses_raw = zlib.decompress(ids)[:-4], zlib.decompress(poses)[:-48]
        db.execute('UPDATE Admin SET opt_ids=?,opt_poses=?', (
            zlib.compress(ids_raw) + struct.pack('<iii', 1, 79, 4),
            zlib.compress(poses_raw) + struct.pack('<iii', 1, 79 * 12, 5),
        ))
    before = path.read_bytes()
    result = seed_working_memory(path, [0] * 6)
    assert not result['seeded'] and result['missing_optimized_nodes'] == 1
    assert path.read_bytes() == before


def test_restart_and_full_rollback_retain_complete_global_reference(tmp_path, monkeypatch):
    source, private = tmp_path / 'source.db', tmp_path / 'localization.db'
    fixture_db(source)
    # Real map1 has one-way constraints; full-mode rollback must restore them,
    # not leave retrieval-only reverse edges in the original graph topology.
    with sqlite3.connect(source) as db:
        db.execute('DELETE FROM Link WHERE from_id=1 AND to_id=2')
    shutil.copyfile(source, private)
    before = source.read_bytes()
    argv = ['prepare', '--source', str(source), '--database', str(private),
            '--pose', '0 0 0 0 0 0', '--report', str(tmp_path / 'report.json')]
    monkeypatch.setattr(sys, 'argv', argv)
    main()
    reference = private.with_suffix('.global_graph.json').read_bytes()
    # A normal native shutdown can leave a different optimized local subset.
    with sqlite3.connect(private) as db:
        db.execute('UPDATE Admin SET opt_ids=NULL,opt_poses=NULL')
    main()
    assert private.with_suffix('.global_graph.json').read_bytes() == reference
    with sqlite3.connect(private) as db:
        assert len(optimized_positions(*db.execute(
            'SELECT opt_ids,opt_poses FROM Admin').fetchone())) == 40
    monkeypatch.setattr(sys, 'argv', argv + ['--mode', 'full'])
    main()
    with sqlite3.connect(private) as db:
        assert len(optimized_positions(*db.execute(
            'SELECT opt_ids,opt_poses FROM Admin').fetchone())) == 80
        assert db.execute('SELECT count(*) FROM Link WHERE type=7').fetchone()[0] == 0
        assert db.execute(
            'SELECT count(*) FROM Link WHERE from_id=1 AND to_id=2').fetchone()[0] == 0
        assert db.execute(
            'SELECT count(*) FROM Link WHERE from_id=2 AND to_id=1').fetchone()[0] == 1
        assert not db.execute(
            "SELECT 1 FROM sqlite_master WHERE name='NavigationMemoryReverse'").fetchone()
    assert source.read_bytes() == before


def test_cli_refuses_source_database(tmp_path, monkeypatch):
    source = tmp_path / 'source.db'
    fixture_db(source)
    before = source.read_bytes()
    monkeypatch.setattr(sys, 'argv', [
        'prepare', '--source', str(source), '--database', str(source),
        '--pose', '0 0 0 0 0 0', '--report', str(tmp_path / 'report.json')])
    with pytest.raises(ValueError, match='source map'):
        main()
    assert source.read_bytes() == before


def test_full_mode_preserves_native_checkpoint_despite_stale_sidecar(tmp_path, monkeypatch):
    source, private = tmp_path / 'source.db', tmp_path / 'localization.db'
    fixture_db(source)
    shutil.copyfile(source, private)
    # A completed native session owns the latest graph/index. A sidecar left
    # from an old experiment must not rewind it or invalidate the saved index.
    private.with_suffix('.global_graph.json').write_text('{"obsolete": true}')
    before = private.read_bytes()
    monkeypatch.setattr(sys, 'argv', [
        'prepare', '--source', str(source), '--database', str(private),
        '--pose', '', '--mode', 'full', '--report', str(tmp_path / 'report.json')])
    main()
    assert private.read_bytes() == before


def test_forward_retrieval_links_restored_with_inverse_geometry(tmp_path):
    path = tmp_path / 'map.db'
    fixture_db(path)
    with sqlite3.connect(path) as db:
        db.execute('DELETE FROM Link WHERE from_id=1 AND to_id=2')
        t = struct.pack('<12f', 0, -1, 0, 2, 1, 0, 0, 3, 0, 0, 1, 4)
        db.execute('UPDATE Link SET transform=? WHERE from_id=2 AND to_id=1', (t,))
        assert restore_reverse_links(db) == 1
        inverse = db.execute(
            'SELECT transform FROM Link WHERE from_id=1 AND to_id=2').fetchone()[0]
        assert struct.unpack('<12f', inverse) == (0, 1, 0, -3, -1, 0, 0, 2, 0, 0, 1, -4)
        original = db.execute('SELECT transform FROM Link WHERE from_id=2 AND to_id=1').fetchone()
        assert original[0] == t
        assert restore_reverse_links(db) == 0


def test_null_link_inverse_keeps_null_transform_and_identity_information(tmp_path):
    path = tmp_path / 'map.db'
    fixture_db(path)
    with sqlite3.connect(path) as db:
        db.execute('DELETE FROM Link WHERE from_id=1 AND to_id=2')
        db.execute('UPDATE Link SET transform=? WHERE from_id=2 AND to_id=1', (bytes(48),))
        assert restore_reverse_links(db) == 1
        transform, information = db.execute(
            'SELECT transform,information_matrix FROM Link WHERE from_id=1 AND to_id=2'
        ).fetchone()
        assert transform == bytes(48)
        assert struct.unpack('<36d', information) == tuple(float(i % 7 == 0) for i in range(36))


def test_absolute_sensor_priors_are_not_silently_enabled_or_overwritten(tmp_path):
    path = tmp_path / 'private.db'
    fixture_db(path)
    with sqlite3.connect(path) as db:
        db.execute('INSERT INTO Link SELECT 1,1,7,transform,information_matrix,NULL '
                   'FROM Link LIMIT 1')
    before = path.read_bytes()
    with pytest.raises(ValueError, match='absolute sensor priors'):
        seed_working_memory(path, [0] * 6)
    assert path.read_bytes() == before
