#!/usr/bin/env python3
"""Back up, selectively apply and roll back ExternalNav on a disarmed Pixhawk.

Never arms, sends motion, changes firmware or restores unrelated calibration.
Stop MAVROS/QGC serial access before using this standalone maintenance tool.
"""

import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'rover_navigation'))
from rover_navigation.external_navigation import FCU_PROFILE  # noqa: E402

PORT = '/dev/serial/by-id/usb-Holybro_Pixhawk6C_2B003C001151333230323637-if00'


def save(path, data):
    """Durably replace a journal before transmitting a parameter write."""
    path = Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    with tmp.open('w') as stream:
        json.dump(data, stream, indent=2, sort_keys=True)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    tmp.replace(path)
    fd = os.open(str(path.parent), os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def close_value(a, b):
    return math.isfinite(a) and abs(a - b) <= max(1e-6, abs(b) * 1e-6)


class FCU:
    def __init__(self, port):
        self.lock = open('/tmp/campus_fcu_maintenance.lock', 'a+')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        owners = subprocess.run(['fuser', os.path.realpath(port)], capture_output=True, text=True)
        if owners.returncode == 0:
            raise RuntimeError('Serial port already owned: ' + owners.stdout.strip())
        self.link = mavutil.mavlink_connection(
            port, baud=115200, source_system=255, source_component=190
        )
        # pyserial exclusive lock also excludes another maintenance tool using flock.
        self.link.port.exclusive = True
        self.state = None
        self.state_at = 0.0
        heartbeat = self.receive('HEARTBEAT', 8.0)
        if heartbeat is None or heartbeat.type != 10 or heartbeat.autopilot != 3:
            raise RuntimeError('Expected ArduRover heartbeat from system/component 1/1')
        self.safe_state()
        self.link.mav.command_long_send(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0)
        version = self.receive('AUTOPILOT_VERSION', 5.0)
        if version is None:
            raise RuntimeError('Cannot identify FCU firmware; no writes allowed')
        self.identity = {
            'uid2': list(version.uid2),
            'flight_sw_version': version.flight_sw_version,
            'flight_custom_version': list(version.flight_custom_version),
        }
        self.version = version.flight_sw_version
        if self.version >> 8 != (4 << 16 | 7 << 8 | 1):
            raise RuntimeError('This profile is reviewed only for ArduRover 4.7.1')

    def receive(self, kind, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            msg = self.link.recv_match(blocking=True, timeout=min(0.2, end - time.monotonic()))
            if msg is None or msg.get_srcSystem() != 1 or msg.get_srcComponent() != 1:
                continue
            if msg.get_type() == 'HEARTBEAT':
                self.state, self.state_at = msg, time.monotonic()
                if msg.base_mode & 128:
                    raise RuntimeError('FCU is armed; stopping maintenance without further writes')
            if msg.get_type() == kind:
                return msg
        return None

    def safe_state(self):
        self.receive('HEARTBEAT', 2.0)
        if (
            self.state is None
            or time.monotonic() - self.state_at > 2.0
            or self.state.base_mode & 128
            or mavutil.mode_string_v10(self.state) != 'HOLD'
        ):
            raise RuntimeError('Need fresh disarmed HOLD telemetry')

    def get(self, name):
        for _ in range(3):
            self.link.mav.param_request_read_send(1, 1, name.encode(), -1)
            end = time.monotonic() + 1.5
            while time.monotonic() < end:
                msg = self.receive('PARAM_VALUE', end - time.monotonic())
                if msg and msg.param_id == name:
                    return {'value': msg.param_value, 'type': msg.param_type}
        raise RuntimeError('Parameter unavailable: ' + name)

    def set(self, name, target, kind):
        self.safe_state()
        for _ in range(3):
            self.link.mav.param_set_send(1, 1, name.encode(), float(target), kind)
            actual = self.get(name)
            if close_value(actual['value'], target):
                print(f'{name}: verified {actual["value"]}', flush=True)
                return
            self.safe_state()
        raise RuntimeError(f'Write/readback failed: {name} expected {target}, got {actual}')

    def snapshot(self):
        self.link.mav.param_request_list_send(1, 1)
        params, indices, count = {}, set(), None
        end, retry, report = time.monotonic() + 150, time.monotonic() + 5, 0.0
        while time.monotonic() < end:
            msg = self.receive('PARAM_VALUE', 0.3)
            if msg and msg.param_index < msg.param_count:
                if count is not None and count != msg.param_count:
                    raise RuntimeError('Parameter list changed while backing up; retry while idle')
                count = msg.param_count
                params[msg.param_id] = {'value': msg.param_value, 'type': msg.param_type}
                indices.add(msg.param_index)
                if len(indices) == count:
                    self.safe_state()
                    return params
            now = time.monotonic()
            if now >= report:
                print(f'Backup {len(indices)}/{count}', flush=True)
                report = now + 5
            if now >= retry:
                if count is None:
                    self.link.mav.param_request_list_send(1, 1)
                else:
                    for index in sorted(set(range(count)) - indices)[:32]:
                        self.link.mav.param_request_read_send(1, 1, b'', index)
                retry = now + 2
        raise RuntimeError(f'Incomplete backup ({len(indices)}/{count}); no profile applied')

    def reboot(self):
        self.safe_state()
        self.link.mav.command_long_send(1, 1, 246, 0, 1, 0, 0, 0, 0, 0, 0)
        print('Normal autopilot reboot requested (not firmware/bootloader mode)', flush=True)

    def close(self):
        if getattr(self, 'link', None) is not None:
            self.link.close()
        if getattr(self, 'lock', None) is not None:
            self.lock.close()


def run(args, fcu):
    directory = Path(args.directory).expanduser()
    original = directory / 'original.json'
    journal_path = directory / 'journal.json'
    if args.action == 'backup':
        directory.mkdir(parents=True, exist_ok=True)
        if original.exists():
            raise RuntimeError('Refusing to overwrite original backup')
        params = fcu.snapshot()
        data = {
            'identity': fcu.identity,
            'utc': datetime.now(timezone.utc).isoformat(),
            'parameters': params,
            'complete': True,
        }
        save(original, data)
        lines = ['# Original FCU backup; MAV ID\tCOMPONENT ID\tNAME\tVALUE\tTYPE\n']
        lines += [f'1\t1\t{k}\t{v["value"]:.9g}\t{v["type"]}\n' for k, v in sorted(params.items())]
        (directory / 'original.params').write_text(''.join(lines))
        (directory / 'original.sha256').write_text(
            hashlib.sha256(original.read_bytes()).hexdigest() + '\n'
        )
        save(journal_path, {'identity': fcu.identity, 'undo': {}, 'events': []})
        print(f'Complete backup: {len(params)} parameters in {directory}', flush=True)
        return
    data = json.loads(original.read_text())
    expected_hash = (directory / 'original.sha256').read_text().strip()
    if hashlib.sha256(original.read_bytes()).hexdigest() != expected_hash:
        raise RuntimeError('Original backup checksum mismatch')
    if data['identity'] != fcu.identity or not data['complete']:
        raise RuntimeError('Backup identity/firmware mismatch')
    journal = json.loads(journal_path.read_text())
    if journal['identity'] != fcu.identity:
        raise RuntimeError('Journal identity mismatch')
    if args.action == 'reboot':
        fcu.reboot()
        return
    targets = {'VISO_TYPE': 3} if args.action == 'enable' else dict(FCU_PROFILE)
    if args.action == 'origin':
        lat, lon, alt = args.origin_lat, args.origin_lon, args.origin_alt_m
        if (
            any(v is None or not math.isfinite(v) for v in (lat, lon, alt))
            or not -90 <= lat <= 90
            or not -180 <= lon <= 180
            or not -200 <= alt <= 5000
            or (lat == 0 and lon == 0)
        ):
            raise ValueError('Supply a reviewed geographic origin and altitude datum')
        targets = {
            'AHRS_ORIGIN_LAT': lat,
            'AHRS_ORIGIN_LON': lon,
            'AHRS_ORIGIN_ALT': alt,
            'AHRS_OPTIONS': int(data['parameters']['AHRS_OPTIONS']['value']) | 16,
        }
    if args.action == 'verify':
        origin_file = directory / 'origin.json'
        if origin_file.exists():
            targets.update(json.loads(origin_file.read_text())['verified'])
        # Independently confirm that actuator tuning and arming checks stayed intact.
        for name, value in data['parameters'].items():
            if name.startswith(('SERVO', 'MOT_', 'ATC_', 'ARMING_')):
                targets[name] = value['value']
    if args.action in ('rollback', 'verify-rollback'):
        targets = {k: v['value'] for k, v in journal['undo'].items()}
        # Keep VISO enabled until hidden sensor parameters have been restored.
        targets = dict(sorted(targets.items(), key=lambda item: item[0] == 'VISO_TYPE'))
    if args.action == 'verify-rollback':
        # Hidden VISO parameters were restored before disabling the backend.
        targets = {k: v for k, v in targets.items() if k in data['parameters']}
    if args.action in ('verify', 'verify-rollback'):
        differences = {}
        for name, target in targets.items():
            live = fcu.get(name)
            if not close_value(live['value'], target):
                differences[name] = {'actual': live['value'], 'expected': target}
        save(
            directory / (args.action + '-result.json'),
            {'identity': fcu.identity, 'differences': differences},
        )
        if differences:
            raise RuntimeError('Verification mismatches: ' + json.dumps(differences))
        print(args.action + ': readback matches all expected values', flush=True)
        return
    # Read the whole transaction before writing any of it. Hidden VISO parameters
    # must first be exposed with enable/reboot; their old values are journalled too.
    current = {name: fcu.get(name) for name in targets}
    if args.action != 'rollback':
        for name, value in current.items():
            if name in data['parameters'] and name not in journal['undo']:
                if not close_value(value['value'], data['parameters'][name]['value']):
                    raise RuntimeError('FCU changed since backup: ' + name)
    for name, target in targets.items():
        old = current[name]
        if close_value(old['value'], target):
            continue
        if args.action != 'rollback':
            journal['undo'].setdefault(name, old)
        journal['events'].append(
            {
                'action': args.action,
                'name': name,
                'from': old['value'],
                'to': target,
                'status': 'intent',
            }
        )
        save(journal_path, journal)
        fcu.set(name, target, old['type'])
        journal['events'][-1]['status'] = 'verified'
        save(journal_path, journal)
    save(directory / (args.action + '.json'), {'identity': fcu.identity, 'verified': targets})
    print(args.action + ' completed; reboot then verify persistence', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'action',
        choices=[
            'backup',
            'enable',
            'apply',
            'origin',
            'verify',
            'rollback',
            'verify-rollback',
            'reboot',
        ],
    )
    parser.add_argument('--directory', required=True)
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--origin-lat', type=float)
    parser.add_argument('--origin-lon', type=float)
    parser.add_argument(
        '--origin-alt-m', type=float, help='AMSL datum in metres, not ellipsoid height'
    )
    args = parser.parse_args()
    fcu = FCU.__new__(FCU)
    try:
        fcu.__init__(args.port)
        run(args, fcu)
    finally:
        fcu.close()


if __name__ == '__main__':
    main()
