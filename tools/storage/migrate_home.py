#!/usr/bin/env python3
"""One-time, offline SD-home -> existing NVMe migration; never formats a disk.

install schedules the migration for the next boot, before user services/login.
apply is run only by the installed root service. The old SD home is removed only
AFTER copy verification, mounting the new home and rebuilding ROS successfully.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import pwd
import shutil
import subprocess
import sys
import tempfile

STATE = Path('/var/lib/campus-home-migration')
PROGRAM = Path('/usr/local/libexec/campus-home-migration.py')
UNIT = 'campus-home-migration.service'
EXCLUDES = ('/ssd/', '/ros2_ws', '/.ros', '/.cache/pip')


def run(*args, capture=False, **kwargs):
    return subprocess.run([str(a) for a in args], check=True, text=True,
                          stdout=subprocess.PIPE if capture else None, **kwargs)


def output(*args):
    return run(*args, capture=True).stdout.strip()


def fstab_for_home(text, uuid, old_mount, home):
    lines = text.splitlines()
    matches = [i for i, line in enumerate(lines)
               if not line.lstrip().startswith('#') and len(line.split()) >= 2
               and line.split()[0] == 'UUID='+uuid and line.split()[1] == str(old_mount)]
    if len(matches) != 1:
        raise RuntimeError('Expected exactly one matching SSD entry in fstab')
    if any(not l.lstrip().startswith('#') and len(l.split()) >= 2
           and l.split()[1] == str(home) for l in lines):
        raise RuntimeError('Home already has a separate fstab entry')
    # Do not fall back silently to an empty SD home if the SSD is unavailable.
    lines[matches[0]] = f'UUID={uuid} {home} ext4 defaults,noatime,x-systemd.device-timeout=30s 0 2'
    return '\n'.join(lines)+'\n'


def remap_path(value, home):
    for old, new in ((str(home/'ssd/user-data/ros'), str(home/'.ros')),
                     (str(home/'ssd/user-data/pip-cache'), str(home/'.cache/pip')),
                     (str(home/'ssd/ros2_ws'), str(home/'ros2_ws'))):
        value = value.replace(old, new)
    return value


def atomic(path, text, mode=0o600):
    temporary = path.with_name(path.name+'.migration-tmp')
    temporary.write_text(text)
    temporary.chmod(mode)
    os.replace(temporary, path)


def install(user):
    account = pwd.getpwnam(user)
    home = Path(account.pw_dir)
    old = home/'ssd'
    if home == Path('/') or not old.is_mount():
        raise RuntimeError('Expected the existing SSD mounted at HOME/ssd')
    uuid = output('findmnt', '-n', '-o', 'UUID', '-T', old)
    device = output('findmnt', '-n', '-o', 'SOURCE', '-T', old)
    if not device.startswith('/dev/nvme') or not uuid:
        raise RuntimeError('Expected a mounted NVMe partition with a UUID')
    if output('findmnt', '-n', '-o', 'FSTYPE', '-T', old) != 'ext4':
        raise RuntimeError('Expected ext4 SSD')
    if (home/'ros2_ws').resolve() != old/'ros2_ws':
        raise RuntimeError('Unexpected workspace location')
    for command in ('rsync', 'runuser', 'mount', 'umount', 'colcon'):
        if not shutil.which(command):
            raise RuntimeError('Missing command: '+command)
    if not (old/'ros2_ws/src/rover_navigation/package.xml').is_file():
        raise RuntimeError('Finish the rover_navigation rename/build first')
    text = Path('/etc/fstab').read_text()
    planned = fstab_for_home(text, uuid, old, home)
    STATE.mkdir(mode=0o700, parents=True, exist_ok=True)
    if (STATE/'complete').exists():
        raise RuntimeError('Migration is already complete')
    atomic(STATE/'original.fstab', text)
    atomic(STATE/'planned.fstab', planned)
    atomic(STATE/'state.json', json.dumps(dict(user=user, uid=account.pw_uid,
           gid=account.pw_gid, home=str(home), uuid=uuid), indent=2))
    PROGRAM.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(Path(__file__).resolve(), PROGRAM)
    PROGRAM.chmod(0o700)
    unit = f'''[Unit]
Description=Move campus user home from SD to NVMe before login
After=local-fs.target
Before=systemd-user-sessions.service user@{account.pw_uid}.service ssh.service display-manager.service
ConditionPathExists=!{STATE}/complete

[Service]
Type=oneshot
ExecStart=/usr/bin/python3 {PROGRAM} apply
TimeoutStartSec=infinity
WorkingDirectory=/
StandardOutput=journal+console
StandardError=journal+console

[Install]
WantedBy=multi-user.target
'''
    atomic(Path('/etc/systemd/system')/UNIT, unit, 0o644)
    run('systemd-analyze', 'verify', str(Path('/etc/systemd/system')/UNIT))
    run('systemctl', 'daemon-reload')
    run('systemctl', 'enable', UNIT)
    print('Scheduled for next boot. No files moved yet; reboot when ready.')


def repair_metadata(home):
    """Preserve map identity/start pose and reuse the existing localization DB."""
    import yaml
    workspace = home/'ros2_ws'
    cache = workspace/'.cache/localization'
    for metadata in cache.glob('*/source.json'):
        identity = json.loads(metadata.read_text())
        old_source = identity['path']
        new_source = remap_path(old_source, home)
        if new_source == old_source:
            continue
        source = Path(new_source)
        stat = source.stat()
        # Workspace data was not copied: same NVMe inode/device/contents.
        if any(identity.get(k) != v for k, v in dict(size=stat.st_size,
                mtime_ns=stat.st_mtime_ns, ctime_ns=stat.st_ctime_ns,
                device=stat.st_dev, inode=stat.st_ino).items()):
            raise RuntimeError('Map identity changed; refusing to relabel cache: '+new_source)
        runtime_file = source.parent/'localization.yaml'
        if not runtime_file.is_file():
            raise RuntimeError('Missing pinned runtime for cache migration')
        runtime = yaml.safe_load(runtime_file.read_text())
        def key(path):
            return hashlib.sha256(json.dumps({'source': path, 'runtime': runtime},
                                            sort_keys=True).encode()).hexdigest()[:24]
        if metadata.parent.name != key(old_source):
            raise RuntimeError('Unrecognized localization cache configuration')
        target = cache/key(new_source)
        if target.exists():
            raise RuntimeError('Cache target already exists')
        identity['path'] = new_source
        metadata.write_text(json.dumps(identity, indent=2))
        metadata.parent.rename(target)
    for p in (workspace/'maps').rglob('startup_pose.yaml'):
        text = p.read_text()
        p.write_text(remap_path(text, home))
    # Known runtime config/scripts only: do not rewrite user histories or keys.
    roots = [home/'.local/bin', home/'.local/share/campus-storage', home/'.config/systemd/user']
    files = [home/'.bashrc', home/'.profile']
    for root in roots:
        if root.exists():
            files.extend(p for p in root.rglob('*') if p.is_file() and not p.is_symlink()
                         and p.stat().st_size < 1024*1024)
    for p in files:
        if not p.is_file():continue
        try: text = p.read_text()
        except UnicodeError:continue
        new = remap_path(text, home)
        if text != new:p.write_text(new)
    # Absolute symbolic links are metadata, never copies of user data.
    for root, dirs, files in os.walk(home, followlinks=False):
        dirs[:] = [d for d in dirs if d not in ('build', 'install', '.git', 'node_modules')]
        for name in dirs+files:
            p = Path(root)/name
            if p.is_symlink():
                old = os.readlink(p); new = remap_path(old, home)
                if old != new:p.unlink();p.symlink_to(new)


def rebuild(home, user):
    workspace = home/'ros2_ws'
    # Keep the old generated files only until the new home build succeeds.
    for name in ('build', 'install'):
        p = workspace/name; previous = workspace/(name+'.home-transition')
        if previous.exists():raise RuntimeError('Unfinished previous build transition')
        if p.exists():p.rename(previous)
    command = '''set -e
source /opt/ros/humble/setup.bash
cd "$HOME/ros2_ws"
colcon build --symlink-install --executor sequential --packages-select ackermann_rpp localization_recovery_bt rover_navigation
source install/setup.bash
ros2 pkg prefix rover_navigation
ros2 launch rover_navigation navigation.launch.py --show-args >/dev/null
bash src/tools/check_storage.sh
test -f src/apps/campus_mission/web/dist/index.html
'''
    run('runuser', '-u', user, '--', 'env', '-i', 'HOME='+str(home), 'USER='+user,
        'PATH=/usr/local/bin:/usr/bin:/bin', 'SHELL=/bin/bash',
        'bash', '--noprofile', '--norc', '-c', command)


def apply():
    if (STATE/'complete').exists():return
    state = json.loads((STATE/'state.json').read_text())
    home = Path(state['home']); old = home/'ssd'
    if output('findmnt', '-n', '-o', 'UUID', '-T', old) != state['uuid']:
        raise RuntimeError('SSD UUID mismatch')
    if subprocess.run(['pgrep', '-u', str(state['uid'])], stdout=subprocess.DEVNULL).returncode == 0:
        raise RuntimeError('User processes still running: migration must run before login')
    if Path('/etc/fstab').read_text() != (STATE/'original.fstab').read_text():
        raise RuntimeError('fstab changed since scheduling; refusing to overwrite it')
    sd = Path('/run/campus-home-sd');sd.mkdir(exist_ok=True)
    run('mount', '--bind', home, sd)  # NOT recursive: excludes SSD submount.
    # A shared bind receives the subsequent SSD overmount too. Keep the SD
    # view private so the device check and cleanup still see the old home.
    run('mount', '--make-private', sd)
    switched = False
    old_unmounted = False
    flattened = []
    try:
        free = shutil.disk_usage(old).free
        used = int(output('du', '-sx', '--block-size=1', home).split()[0])
        if free < used+1024**3:raise RuntimeError('Insufficient SSD free space')
        options = ['-aHAXx', '--omit-dir-times'] + ['--exclude='+x for x in EXCLUDES]
        run('rsync', *options, str(sd)+'/', str(old)+'/')
        differences = output('rsync', *options, '--checksum', '--dry-run', '--itemize-changes', str(sd)+'/', str(old)+'/')
        if differences:raise RuntimeError('Home copy verification failed: '+differences[:300])
        for source, target in ((old/'user-data/ros', old/'.ros'),
                               (old/'user-data/pip-cache', old/'.cache/pip')):
            if target.exists() or target.is_symlink():raise RuntimeError('Destination conflict: '+str(target))
            target.parent.mkdir(parents=True, exist_ok=True)
            source.rename(target);flattened.append((source,target))
        os.chown(old,state['uid'],state['gid']);old.chmod(sd.stat().st_mode & 0o777)
        run('sync')
        run('umount', old)
        old_unmounted = True
        run('mount', '-t', 'ext4', '-o', 'noatime', 'UUID='+state['uuid'], home)
        switched = True
        # Save small metadata for rollback before any path changes.
        repair_files = list((home/'ros2_ws/.cache/localization').glob('*/source.json')) + list((home/'ros2_ws/maps').rglob('startup_pose.yaml'))
        saved = {str(p.relative_to(home)):p.read_bytes().hex() for p in repair_files}
        atomic(STATE/'metadata.json',json.dumps(saved))
        repair_metadata(home)
        rebuild(home,state['user'])
        atomic(Path('/etc/fstab'),(STATE/'planned.fstab').read_text(),0o644)
        run('systemctl','daemon-reload')
        # Migration has succeeded. There is one permanent home; no SD copy.
        atomic(STATE/'complete','Home mounted on SSD; ROS rebuild verified.\n')
        for name in ('build.home-transition','install.home-transition'):
            p=home/'ros2_ws'/name
            if p.exists():shutil.rmtree(p)
        userdata=home/'user-data'
        if userdata.exists() and not any(userdata.iterdir()):userdata.rmdir()
        if sd.stat().st_dev != Path('/').stat().st_dev or sd.stat().st_dev == home.stat().st_dev:
            raise RuntimeError('Unexpected source device; refusing SD cleanup')
        for p in sd.iterdir():
            if p.is_symlink() or not p.is_dir():p.unlink()
            else:shutil.rmtree(p)
        run('sync')
        atomic(STATE/'cleanup-complete', 'Verified SD home contents removed.\n')
        print('Home migration complete. SD home contents removed; OS remains on SD.')
    except Exception:
        if (STATE/'complete').exists():
            # Do not roll back a successfully mounted home after partial cleanup.
            raise
        if switched:
            workspace=home/'ros2_ws'
            for name in ('build','install'):
                previous=workspace/(name+'.home-transition');current=workspace/name
                if previous.exists():
                    if current.exists():shutil.rmtree(current)
                    previous.rename(current)
            saved=json.loads((STATE/'metadata.json').read_text()) if (STATE/'metadata.json').exists() else {}
            cache=workspace/'.cache/localization'
            for relative, encoded in saved.items():
                p=home/relative
                if p.name=='source.json' and not p.parent.exists():
                    original=json.loads(bytes.fromhex(encoded))
                    found=[x for x in cache.glob('*/source.json') if json.loads(x.read_text()).get('path')==remap_path(original['path'],home)]
                    if len(found)==1:found[0].parent.rename(p.parent)
                p.write_bytes(bytes.fromhex(encoded))
            run('umount',home)
            run('mount','-t','ext4','-o','noatime','UUID='+state['uuid'],old)
        elif old_unmounted:
            run('mount','-t','ext4','-o','noatime','UUID='+state['uuid'],old)
        for source,target in reversed(flattened):
            if target.exists():target.rename(source)
        atomic(Path('/etc/fstab'),(STATE/'original.fstab').read_text(),0o644)
        run('systemctl','daemon-reload')
        run('systemctl','disable',UNIT)
        raise
    finally:
        run('umount',sd)


def verify_cleanup_source(source, home, root):
    """Refuse cleanup if the hidden home resolves to the live SSD or a link."""
    if source.is_symlink() or not source.is_dir():
        raise RuntimeError('Expected a real SD home directory')
    if source.resolve() != root.resolve() / home.relative_to('/'):
        raise RuntimeError('Unexpected hidden home path')
    if source.stat().st_dev != root.stat().st_dev:
        raise RuntimeError('Hidden home is not on the SD root filesystem')
    if source.stat().st_dev == home.stat().st_dev:
        raise RuntimeError('Refusing to remove the active home filesystem')
    mounts = output('findmnt', '-rn', '-o', 'TARGET').splitlines()
    if any(Path(p) == source or source in Path(p).parents for p in mounts):
        raise RuntimeError('Unexpected mount below hidden SD home')


def cleanup(private=False):
    """Finish only SD cleanup after a verified migration, without restarting ROS."""
    if not private:
        run('unshare', '--mount', '--propagation', 'private', sys.executable,
            Path(__file__).resolve(), 'cleanup-private')
        run('systemctl', 'disable', UNIT)
        run('systemctl', 'reset-failed', UNIT)
        return
    if not (STATE/'complete').is_file():
        raise RuntimeError('Migration and ROS rebuild have not completed')
    state = json.loads((STATE/'state.json').read_text())
    home = Path(state['home'])
    if home != Path(pwd.getpwuid(state['uid']).pw_dir) or home == Path('/'):
        raise RuntimeError('Unexpected home account')
    if not home.is_mount() or output('findmnt', '-n', '-o', 'UUID', '-T', home) != state['uuid']:
        raise RuntimeError('The expected SSD is not mounted at home')
    if Path('/etc/fstab').read_text() != (STATE/'planned.fstab').read_text():
        raise RuntimeError('Unexpected fstab: refusing cleanup')
    run('runuser', '-u', state['user'], '--', 'env', '-i', 'HOME='+str(home),
        'PATH=/usr/local/bin:/usr/bin:/bin', 'bash', '--noprofile', '--norc', '-c',
        'set -e; source /opt/ros/humble/setup.bash; '
        'source "$HOME/ros2_ws/install/setup.bash"; '
        'ros2 pkg prefix rover_navigation; bash "$HOME/ros2_ws/src/tools/check_storage.sh"')
    root = Path(tempfile.mkdtemp(prefix='campus-sd-cleanup-', dir='/run'))
    mounted = False
    try:
        # Nonrecursive root view exposes the SD directory beneath the SSD mount.
        # The private namespace prevents mount propagation to running services.
        run('mount', '--bind', '/', root)
        mounted = True
        source = root / home.relative_to('/')
        verify_cleanup_source(source, home, root)
        for p in source.iterdir():
            if p.is_symlink() or not p.is_dir():
                p.unlink()
            else:
                shutil.rmtree(p)
        run('sync')
        atomic(STATE/'cleanup-complete', 'Verified SD home contents removed.\n')
        print('SD duplicate removed. Active SSD home and running services were preserved.')
    finally:
        if mounted:
            run('umount', root)
        root.rmdir()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('install','apply','cleanup','cleanup-private'))
    parser.add_argument('--user',default='ykk')
    args=parser.parse_args()
    if os.geteuid()!=0:parser.error('Use sudo; no password is stored by this tool')
    if args.action=='install':install(args.user)
    elif args.action=='apply':apply()
    else:cleanup(private=args.action=='cleanup-private')


if __name__=='__main__':main()
