"""Reuse a private localization DB while preserving the selected map source."""

import fcntl
from contextlib import closing
import hashlib
import json
import os
from pathlib import Path
import sqlite3


def source_identity(source):
    stat = source.stat()
    return {
        'path': str(source.resolve()),
        'size': stat.st_size,
        'mtime_ns': stat.st_mtime_ns,
        'ctime_ns': stat.st_ctime_ns,
        'device': stat.st_dev,
        'inode': stat.st_ino,
    }


def full_working_memory_saved(database):
    """
    Restore the previous WM only when it still covers the complete map.

    RTAB 0.23 uses Node/Word.time_enter >= the last Info timestamp to restore
    session memory. Otherwise keep InitWMWithAllNodes=true. In particular, an
    interrupted first dictionary repair must not reduce relocalization coverage.
    """
    try:
        with closing(
            sqlite3.connect(Path(database).resolve().as_uri() + '?mode=ro', uri=True, timeout=0.2)
        ) as connection:
            nodes, saved = connection.execute(
                'SELECT count(*), sum(time_enter >= (SELECT max(time_enter) FROM Info)) '
                'FROM Node WHERE weight > -9'
            ).fetchone()
            if not nodes or saved != nodes:
                return False
            info = connection.execute(
                'SELECT dictionary_size FROM Info ORDER BY time_enter DESC LIMIT 1'
            ).fetchone()
            words = connection.execute(
                'SELECT count(*) FROM Word '
                'WHERE time_enter >= (SELECT max(time_enter) FROM Info)'
            ).fetchone()[0]
            return bool(info and info[0] > 0 and words >= info[0])
    except (sqlite3.Error, OSError):
        return False


class LocalizationDatabase:
    """Hold one writer lease for the launch lifetime, including RTAB shutdown."""

    def __init__(self, source, runtime, cache_root):
        self.source = Path(source).resolve()
        self.identity = source_identity(self.source)
        # Configuration changes must not inherit parameters saved by a different
        # localization run. Source revisions replace this cache, not accumulate it.
        key = hashlib.sha256(
            json.dumps({'source': str(self.source), 'runtime': runtime}, sort_keys=True).encode()
        ).hexdigest()[:24]
        directory = Path(cache_root).expanduser() / key
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / 'localization.db'
        self.pending = directory / 'localization.db.partial'
        self.metadata = directory / 'source.json'
        self.lock = (directory / 'writer.lock').open('a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self.lock.close()
            raise ValueError('Localization database already in use: ' + str(self.path)) from exc

    def ready(self):
        if not self.path.is_file() or self.path.stat().st_size == 0:
            return False
        try:
            return json.loads(self.metadata.read_text()) == self.identity
        except (OSError, ValueError):
            return False

    def finish_copy(self):
        if source_identity(self.source) != self.identity:
            raise ValueError('Source map database changed during copy; restart navigation')
        if not self.pending.is_file() or self.pending.stat().st_size != self.identity['size']:
            raise ValueError('Incomplete localization database copy')
        os.replace(self.pending, self.path)
        temporary = self.metadata.with_suffix('.json.partial')
        temporary.write_text(json.dumps(self.identity, indent=2))
        os.replace(temporary, self.metadata)

    def close(self):
        self.lock.close()
