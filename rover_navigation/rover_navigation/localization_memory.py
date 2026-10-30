"""
Prepare RTAB's startup working set in a locked, private localization DB.

Nodes, links, images and descriptors stay in the DB for native retrieval.
The full optimized pose checkpoint is retained in a small sidecar; the DB's
startup pose checkpoint contains the selected nodes in the SAME map coordinates.
This adapter is specific to RTAB-Map's 0.23 SQLite checkpoint format.
"""

import argparse
import base64
from contextlib import closing
import json
import math
from pathlib import Path
import sqlite3
import struct
import zlib

from rover_navigation.localization_database import source_identity


def remove_reference_priors(db):
    """Remove only constraints created by this adapter, never mapping priors."""
    if db.execute("SELECT 1 FROM sqlite_master WHERE name='NavigationMemoryPrior'").fetchone():
        db.execute('DELETE FROM Link WHERE from_id=to_id AND type=7 '
                   'AND from_id IN (SELECT id FROM NavigationMemoryPrior)')
        db.execute('DROP TABLE NavigationMemoryPrior')


def remove_reverse_links(db):
    """Undo only links added for partial-memory retrieval before full loading."""
    if db.execute("SELECT 1 FROM sqlite_master WHERE name='NavigationMemoryReverse'").fetchone():
        db.execute('DELETE FROM Link WHERE EXISTS '
                   '(SELECT 1 FROM NavigationMemoryReverse r WHERE '
                   'r.from_id=Link.from_id AND r.to_id=Link.to_id AND r.type=Link.type)')
        db.execute('DROP TABLE NavigationMemoryReverse')


def install_reference_priors(db, ids_blob, poses_blob):
    """
    Keep the saved static map fixed when native retrieval optimizes a subset.

    These are map reference constraints, not fresh localization measurements.
    Use native RGBD/LocalizationPriorError's default 1 mm / 0.001 rad error.
    Existing absolute sensor priors must not become enabled as a side effect.
    """
    remove_reference_priors(db)
    if db.execute('SELECT 1 FROM Link WHERE type=7 LIMIT 1').fetchone():
        raise ValueError('map contains absolute sensor priors; use localization_memory:=full')
    ids_raw, poses_raw = zlib.decompress(ids_blob), zlib.decompress(poses_blob)
    ids = struct.unpack('<' + 'i' * (len(ids_raw) // 4), ids_raw)
    valid = {row[0] for row in db.execute('SELECT id FROM Node')}
    information = struct.pack('<36d', *[1e6 if i % 7 == 0 else 0.0 for i in range(36)])
    rows = [(node, node, poses_raw[i * 48:(i + 1) * 48], information)
            for i, node in enumerate(ids) if node > 0 and node in valid]
    db.execute('CREATE TABLE NavigationMemoryPrior(id INTEGER PRIMARY KEY)')
    db.executemany('INSERT INTO NavigationMemoryPrior VALUES(?)', ((row[0],) for row in rows))
    db.executemany('INSERT INTO Link(from_id,to_id,type,transform,information_matrix,user_data) '
                   'VALUES(?,?,7,?,?,NULL)', rows)
    return len(rows)


def restore_reverse_links(db):
    """
    Complete equivalent reverse constraints, following native Link::inverse().

    RTAB's per-node SQL loader reads only outgoing edges. A DB with one
    direction only cannot prefetch the forward trajectory.
    No new geometric measurement is invented; existing directions are untouched.
    """
    missing = db.execute(
        'SELECT a.from_id,a.to_id,a.type,a.transform,a.information_matrix FROM Link a '
        'JOIN Node n1 ON n1.id=a.from_id JOIN Node n2 ON n2.id=a.to_id '
        'WHERE a.from_id>0 AND a.to_id>0 AND a.from_id!=a.to_id AND a.type BETWEEN 0 AND 6 '
        'AND NOT EXISTS (SELECT 1 FROM Link b WHERE b.from_id=a.to_id '
        'AND b.to_id=a.from_id AND b.type=a.type)'
    ).fetchall()
    db.execute('CREATE TABLE IF NOT EXISTS NavigationMemoryReverse('
               'from_id INTEGER,to_id INTEGER,type INTEGER,PRIMARY KEY(from_id,to_id,type))')
    for from_id, to_id, kind, transform, information in missing:
        if not transform or len(transform) != 48 or not information or len(information) != 288:
            raise ValueError('unsupported link format; cannot restore reverse constraints')
        values = struct.unpack('<12f', transform)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('invalid transform in map link')
        db.execute('INSERT INTO NavigationMemoryReverse VALUES(?,?,?)', (to_id, from_id, kind))
        if not any(values):
            # Native null links carry no geometric transform. Its inverse is
            # also null, with identity information (Link::inverse()).
            identity = struct.pack('<36d', *[float(i % 7 == 0) for i in range(36)])
            db.execute('INSERT INTO Link('
                       'from_id,to_id,type,transform,information_matrix,user_data) '
                       'VALUES(?,?,?,?,?,NULL)', (to_id, from_id, kind, transform, identity))
            continue
        rotation = [[values[i * 4 + j] for j in range(3)] for i in range(3)]
        translation = [values[i * 4 + 3] for i in range(3)]
        for i in range(3):
            for j in range(3):
                dot = sum(rotation[k][i] * rotation[k][j] for k in range(3))
                if abs(dot - int(i == j)) > 0.001:
                    raise ValueError('nonrigid transform in map link')
        inverse = []
        for i in range(3):
            inverse.extend(rotation[j][i] for j in range(3))
            inverse.append(-sum(rotation[j][i] * translation[j] for j in range(3)))
        # Native Link::inverse keeps the information matrix and drops user_data.
        db.execute('INSERT INTO Link(from_id,to_id,type,transform,information_matrix,user_data) '
                   'VALUES(?,?,?,?,?,NULL)',
                   (to_id, from_id, kind, struct.pack('<12f', *inverse), information))
    return len(missing)


def optimized_positions(ids_blob, poses_blob):
    """Decode OpenCV matrices stored by RTAB, never session-local Node.pose."""
    def unpack(blob, cv_type, scalar):
        if not blob or len(blob) < 13:
            raise ValueError('optimized map poses are not saved')
        rows, cols, kind = struct.unpack('<iii', blob[-12:])
        if rows != 1 or kind != cv_type or not 0 < cols <= 12000000:
            raise ValueError('unsupported optimized pose matrix')
        raw = zlib.decompress(blob[:-12])
        if len(raw) != cols * 4:
            raise ValueError('invalid optimized pose matrix size')
        return struct.unpack('<' + scalar * cols, raw)

    ids = unpack(ids_blob, 4, 'i')
    poses = unpack(poses_blob, 5, 'f')
    if len(poses) != 12 * len(ids) or len(set(ids)) != len(ids):
        raise ValueError('inconsistent optimized pose IDs')
    if not all(math.isfinite(v) for v in poses):
        raise ValueError('nonfinite optimized map pose')
    return {node: (poses[i * 12 + 3], poses[i * 12 + 7], poses[i * 12 + 11])
            for i, node in enumerate(ids) if node > 0}


def seed_working_memory(database, pose, radius=15.0, max_nodes=400, graph=None):
    """
    Return a report; missing optimized map falls back to full initialization.

    The caller MUST hold LocalizationDatabase's writer lease and ensure no RTAB
    process has opened this DB. Failure rolls back the transaction; it never marks
    an incomplete seed as usable. Do not call on a mapping/source database.
    """
    pose = [float(v) for v in pose]
    if len(pose) != 6 or not all(math.isfinite(v) for v in pose):
        raise ValueError('expected finite startup x y z roll pitch yaw')
    if not math.isfinite(radius) or radius <= 0 or max_nodes < 30:
        raise ValueError('invalid local memory radius or node budget')
    with closing(sqlite3.connect(str(database), timeout=1.0)) as db:
        with db:
            db.execute('BEGIN IMMEDIATE')
            admin = db.execute('SELECT version,opt_ids,opt_poses FROM Admin').fetchone()
            if not admin or not admin[0].startswith('0.23.'):
                raise ValueError('local memory requires RTAB SQLite 0.23.x')
            ids_blob, poses_blob = graph if graph else admin[1:]
            if not ids_blob or not poses_blob:
                return {'seeded': False, 'reason': 'no saved optimized map; load full map once'}
            positions = optimized_positions(ids_blob, poses_blob)
            valid_ids = {row[0] for row in db.execute('SELECT id FROM Node WHERE weight>=0')}
            missing = valid_ids.difference(positions)
            if missing:
                # A saved localization checkpoint may cover only one part of a
                # continued map. Retrieving an unanchored node can then change
                # the partial graph's coordinates. Never seed that checkpoint.
                return {'seeded': False, 'reason': 'optimized map checkpoint is incomplete',
                        'missing_optimized_nodes': len(missing), 'map_nodes': len(valid_ids)}
            ranked = sorted(
                (sum((v - p) ** 2 for v, p in zip(xyz, pose[:3])), node)
                for node, xyz in positions.items() if node in valid_ids
            )
            selected = [node for distance, node in ranked if distance <= radius ** 2][:max_nodes]
            if len(selected) < 30:
                return {'seeded': False, 'reason': 'too few mapped nodes near startup pose'}
            threshold = db.execute('SELECT max(time_enter) FROM Info').fetchone()[0]
            if not threshold:
                raise ValueError('missing RTAB session checkpoint')
            older = db.execute("SELECT datetime(?, '-1 second')", (threshold,)).fetchone()[0]
            if not older:
                raise ValueError('invalid RTAB checkpoint timestamp')
            db.execute('CREATE TEMP TABLE selected_nodes(id INTEGER PRIMARY KEY)')
            db.executemany('INSERT INTO selected_nodes VALUES (?)', ((n,) for n in selected))
            db.execute('CREATE TEMP TABLE selected_words(id INTEGER PRIMARY KEY)')
            db.execute('INSERT INTO selected_words SELECT DISTINCT word_id FROM Feature '
                       'WHERE node_id IN (SELECT id FROM selected_nodes) AND word_id>0')
            word_count = db.execute('SELECT count(*) FROM selected_words').fetchone()[0]
            present = db.execute('SELECT count(*) FROM Word WHERE id IN '
                                 '(SELECT id FROM selected_words)').fetchone()[0]
            # An interrupted mapping DB may lack most Word records. Let native
            # full initialization repair it first, rather than seed a broken index.
            if not word_count or present != word_count:
                return {'seeded': False, 'reason': 'dictionary incomplete; full repair required'}
            reverse_links = restore_reverse_links(db)
            reference_priors = install_reference_priors(db, ids_blob, poses_blob)
            db.execute('UPDATE Node SET time_enter=? WHERE time_enter>=?', (older, threshold))
            db.execute('UPDATE Word SET time_enter=? WHERE time_enter>=?', (older, threshold))
            db.execute('UPDATE Node SET time_enter=? WHERE id IN '
                       '(SELECT id FROM selected_nodes)', (threshold,))
            db.execute('UPDATE Word SET time_enter=? WHERE id IN '
                       '(SELECT id FROM selected_words)', (threshold,))
            db.execute('UPDATE Info SET dictionary_size=? WHERE time_enter=?',
                       (word_count, threshold))
            # A serialized index for a different word set must never be restored.
            # Native 0.23 rejects optimized poses whose IDs are absent from WM.
            # Preserve the selected nodes' GLOBAL optimized transforms instead
            # of forcing a different partial-graph optimization at startup.
            # The caller retains the full checkpoint separately for restart/full mode.
            ids_raw, poses_raw = zlib.decompress(ids_blob), zlib.decompress(poses_blob)
            ids = struct.unpack('<' + 'i' * (len(ids_raw) // 4), ids_raw)
            selected_set = set(selected)
            indexes = [i for i, node in enumerate(ids) if node in selected_set]
            subset_ids = b''.join(ids_raw[i * 4:(i + 1) * 4] for i in indexes)
            subset_poses = b''.join(poses_raw[i * 48:(i + 1) * 48] for i in indexes)
            db.execute('UPDATE Admin SET dictionary_index=NULL,opt_ids=?,opt_poses=?', (
                zlib.compress(subset_ids) + struct.pack('<iii', 1, len(indexes), 4),
                zlib.compress(subset_poses) + struct.pack('<iii', 1, len(indexes) * 12, 5),
            ))
            farthest = max(d for d, node in ranked if node in selected_set)
            return {'seeded': True, 'nodes': len(selected), 'words': word_count,
                    'map_nodes': len(valid_ids), 'radius_m': radius,
                    'farthest_m': math.sqrt(farthest), 'reverse_links_added': reverse_links,
                    'reference_priors': reference_priors}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--database', required=True)
    parser.add_argument('--source', required=True)
    parser.add_argument('--pose', required=True)
    parser.add_argument('--report', required=True)
    parser.add_argument('--mode', choices=['auto', 'full'], default='auto')
    parser.add_argument('--radius', type=float, default=15.0)
    parser.add_argument('--nodes', type=int, default=400)
    args = parser.parse_args()
    database, source = Path(args.database).resolve(), Path(args.source).resolve()
    if database == source or database.samefile(source):
        raise ValueError('memory preparation must never modify the source map')
    if args.mode == 'full':
        with closing(sqlite3.connect(database.as_uri() + '?mode=ro', uri=True)) as db:
            adapted = db.execute(
                "SELECT 1 FROM sqlite_master WHERE name IN "
                "('NavigationMemoryPrior', 'NavigationMemoryReverse') LIMIT 1"
            ).fetchone()
        if not adapted:
            # Native full localization may have saved a newer graph and index.
            # A sidecar from an old experiment must not overwrite that checkpoint.
            report = {'seeded': False, 'reason': 'native full checkpoint preserved'}
            Path(args.report).write_text(json.dumps(report, indent=2) + '\n')
            print('Localization memory: ' + json.dumps(report), flush=True)
            return
    identity = source_identity(source)
    reference = database.with_suffix('.global_graph.json')
    graph = None
    if reference.exists():
        data = json.loads(reference.read_text())
        if data['source'] == identity:
            graph = tuple(base64.b64decode(data[key], validate=True)
                          for key in ('ids', 'poses'))
            optimized_positions(*graph)
    if graph is None:
        with closing(sqlite3.connect(database.as_uri() + '?mode=ro', uri=True)) as db:
            graph = db.execute('SELECT opt_ids,opt_poses FROM Admin').fetchone()
        if graph and all(graph):
            optimized_positions(*graph)
            data = {'source': identity, **{
                key: base64.b64encode(blob).decode('ascii')
                for key, blob in zip(('ids', 'poses'), graph)}}
            temporary = reference.with_suffix('.partial')
            temporary.write_text(json.dumps(data))
            temporary.replace(reference)
        else:
            graph = None
    if args.mode == 'full' or not args.pose:
        if graph:
            with closing(sqlite3.connect(database)) as db:
                with db:
                    remove_reference_priors(db)
                    remove_reverse_links(db)
                    db.execute('UPDATE Admin SET opt_ids=?,opt_poses=?', graph)
        report = {'seeded': False, 'reason': 'full-map initialization requested'}
    else:
        report = seed_working_memory(database, args.pose.split(), args.radius, args.nodes, graph)
        if not report['seeded'] and graph:
            with closing(sqlite3.connect(database)) as db:
                with db:
                    remove_reference_priors(db)
                    remove_reverse_links(db)
                    db.execute('UPDATE Admin SET opt_ids=?,opt_poses=?', graph)
    Path(args.report).write_text(json.dumps(report, indent=2) + '\n')
    print('Localization memory: ' + json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
