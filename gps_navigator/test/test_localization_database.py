"""Private DB reuse, revision invalidation and exclusive-writer regressions."""

import shutil
import sqlite3

import pytest

from gps_navigator.localization_database import LocalizationDatabase, full_working_memory_saved


def test_reuse_preserves_original_and_refreshes_after_source_revision(tmp_path):
    source = tmp_path / 'source.db'
    source.write_bytes(b'original map')
    runtime = {'rtabmap': {'args': '--Vis/MinInliers 20'}}
    lease = LocalizationDatabase(source, runtime, tmp_path / 'cache')
    try:
        assert not lease.ready()
        shutil.copyfile(source, lease.pending)
        lease.finish_copy()
        assert lease.ready()
        # RTAB can write only this private working database.
        lease.path.write_bytes(b'private localization session state')
        assert source.read_bytes() == b'original map'
        private_path = lease.path
    finally:
        lease.close()
    reopened = LocalizationDatabase(source, runtime, tmp_path / 'cache')
    try:
        assert reopened.ready() and reopened.path == private_path
    finally:
        reopened.close()
    source.write_bytes(b'new map revision')
    refreshed = LocalizationDatabase(source, runtime, tmp_path / 'cache')
    try:
        assert refreshed.path == private_path and not refreshed.ready()
        shutil.copyfile(source, refreshed.pending)
        refreshed.finish_copy()
        assert refreshed.ready() and refreshed.path.read_bytes() == source.read_bytes()
    finally:
        refreshed.close()


def test_lock_prevents_concurrent_rtab_writers_and_runtime_change_isolated(tmp_path):
    source = tmp_path / 'source.db'
    source.write_bytes(b'original')
    lease = LocalizationDatabase(source, {'rtabmap': {}}, tmp_path / 'cache')
    try:
        with pytest.raises(ValueError, match='already in use'):
            LocalizationDatabase(source, {'rtabmap': {}}, tmp_path / 'cache')
        other = LocalizationDatabase(
            source, {'rtabmap': {'args': 'different'}}, tmp_path / 'cache'
        )
        try:
            assert other.path != lease.path and not other.ready()
        finally:
            other.close()
    finally:
        lease.close()


def test_partial_copy_and_source_mutation_cannot_be_marked_ready(tmp_path):
    source = tmp_path / 'source.db'
    source.write_bytes(b'original')
    lease = LocalizationDatabase(source, {}, tmp_path / 'cache')
    try:
        lease.pending.write_bytes(b'short')
        with pytest.raises(ValueError, match='Incomplete'):
            lease.finish_copy()
        assert not lease.ready()
        shutil.copyfile(source, lease.pending)
        source.write_bytes(b'revised')
        with pytest.raises(ValueError, match='changed during copy'):
            lease.finish_copy()
        assert not lease.ready()
    finally:
        lease.close()


def test_index_restore_requires_full_map_and_dictionary_checkpoint(tmp_path):
    database = tmp_path / 'memory.db'
    with sqlite3.connect(database) as connection:
        connection.executescript(
            '''
            CREATE TABLE Node(id INTEGER, weight INTEGER, time_enter TEXT);
            CREATE TABLE Info(dictionary_size INTEGER, time_enter TEXT);
            CREATE TABLE Word(id INTEGER, time_enter TEXT);
            INSERT INTO Node VALUES (1, 0, '2026-10-01'), (2, 0, '2026-10-02'),
                                    (3, -9, '2026-10-01');
            INSERT INTO Info VALUES (2, '2026-10-02');
            INSERT INTO Word VALUES (1, '2026-10-02'), (2, '2026-10-02');
        '''
        )
    assert not full_working_memory_saved(database)  # Node 1 absent from last session.
    with sqlite3.connect(database) as connection:
        connection.execute("UPDATE Node SET time_enter='2026-10-02' WHERE id=1")
    assert full_working_memory_saved(database)
    with sqlite3.connect(database) as connection:
        connection.execute('DELETE FROM Word WHERE id=2')
    assert not full_working_memory_saved(database)  # Interrupted dictionary save.
    assert not full_working_memory_saved(tmp_path / 'missing.db')
