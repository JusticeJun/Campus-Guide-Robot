import hashlib
import importlib.util
import json
import os
from pathlib import Path
import pytest
import yaml

spec=importlib.util.spec_from_file_location('migration',Path(__file__).with_name('migrate_home.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def test_fstab_only_replaces_verified_ssd_entry_and_requires_home():
    old='# test\n/dev/root / ext4 defaults 0 1\nUUID=abc /home/ykk/ssd ext4 nofail 0 2\n'
    new=m.fstab_for_home(old,'abc',Path('/home/ykk/ssd'),Path('/home/ykk'))
    assert '/dev/root / ext4 defaults 0 1' in new
    assert 'UUID=abc /home/ykk ext4 defaults,noatime' in new
    assert 'nofail' not in new and '/home/ykk/ssd' not in new
    with pytest.raises(RuntimeError):m.fstab_for_home(old,'different',Path('/home/ykk/ssd'),Path('/home/ykk'))
    with pytest.raises(RuntimeError):m.fstab_for_home(new,'abc',Path('/home/ykk/ssd'),Path('/home/ykk'))


def fixture(home):
    directory=home/'ros2_ws/maps/map1';directory.mkdir(parents=True)
    db=directory/'rtabmap.db';db.write_bytes(b'unchanged map')
    runtime={'camera':{'fps':15},'rtabmap':{'args':'unchanged'}}
    (directory/'localization.yaml').write_text(yaml.safe_dump(runtime))
    old=str(home/'ssd/ros2_ws/maps/map1/rtabmap.db')
    stat=db.stat()
    identity=dict(path=old,size=stat.st_size,mtime_ns=stat.st_mtime_ns,ctime_ns=stat.st_ctime_ns,device=stat.st_dev,inode=stat.st_ino)
    key=hashlib.sha256(json.dumps({'source':old,'runtime':runtime},sort_keys=True).encode()).hexdigest()[:24]
    cache=home/'ros2_ws/.cache/localization'/key;cache.mkdir(parents=True)
    (cache/'source.json').write_text(json.dumps(identity));(cache/'localization.db').write_bytes(b'working map')
    (directory/'startup_pose.yaml').write_text(yaml.safe_dump({'source':{'database':old},'pose':[1,2,0,0,0,.5]}))
    return db,cache,identity


def test_relocation_preserves_db_inode_and_pose_and_rekeys_cache(tmp_path):
    db,cache,_=fixture(tmp_path)
    inode=(cache/'localization.db').stat().st_ino
    (tmp_path/'example').symlink_to(str(tmp_path/'ssd/ros2_ws'))
    m.repair_metadata(tmp_path)
    entries=list((tmp_path/'ros2_ws/.cache/localization').iterdir())
    assert len(entries)==1 and entries[0]!=cache
    assert (entries[0]/'localization.db').stat().st_ino==inode
    assert json.loads((entries[0]/'source.json').read_text())['path']==str(db)
    pose=yaml.safe_load((db.parent/'startup_pose.yaml').read_text())
    assert pose['pose']==[1,2,0,0,0,.5] and pose['source']['database']==str(db)
    assert (tmp_path/'example').readlink()==tmp_path/'ros2_ws'
    assert db.read_bytes()==b'unchanged map'


def test_changed_map_is_not_relabelled_as_valid(tmp_path):
    db,cache,_=fixture(tmp_path);db.write_bytes(b'different source revision')
    with pytest.raises(RuntimeError,match='identity changed'):m.repair_metadata(tmp_path)
    assert cache.exists()


def test_cleanup_rejects_active_home_and_symlink(tmp_path, monkeypatch):
    root = tmp_path/'root'
    home = tmp_path/'live'
    home.mkdir()
    source = root/home.relative_to('/')
    source.mkdir(parents=True)
    monkeypatch.setattr(m, 'output', lambda *args: '')
    with pytest.raises(RuntimeError, match='active home'):
        m.verify_cleanup_source(source, home, root)
    source.rmdir()
    source.symlink_to(home)
    with pytest.raises(RuntimeError, match='real SD home'):
        m.verify_cleanup_source(source, home, root)


def test_cleanup_checks_device_and_nested_mounts(tmp_path, monkeypatch):
    root = tmp_path/'root'
    home = tmp_path/'live'
    home.mkdir()
    source = root/home.relative_to('/')
    source.mkdir(parents=True)
    original_stat = Path.stat

    def stat(path, *args, **kwargs):
        result = original_stat(path, *args, **kwargs)
        if path == home:
            values = list(result)
            values[2] += 1  # Model the live SSD as a different filesystem.
            return os.stat_result(values)
        return result

    monkeypatch.setattr(Path, 'stat', stat)
    monkeypatch.setattr(m, 'output', lambda *args: str(root))
    m.verify_cleanup_source(source, home, root)
    monkeypatch.setattr(m, 'output', lambda *args: str(source/'unexpected_mount'))
    with pytest.raises(RuntimeError, match='Unexpected mount'):
        m.verify_cleanup_source(source, home, root)


def test_cleanup_uses_private_namespace_before_clearing_failed_status(monkeypatch):
    calls = []
    monkeypatch.setattr(m, 'run', lambda *args, **kwargs: calls.append(args))
    m.cleanup()
    assert calls[0][:5] == ('unshare', '--mount', '--propagation', 'private', m.sys.executable)
    assert calls[0][-1] == 'cleanup-private'
    assert calls[1] == ('systemctl', 'disable', m.UNIT)
    assert calls[2] == ('systemctl', 'reset-failed', m.UNIT)


def test_cleanup_failure_keeps_failed_status(monkeypatch):
    calls = []

    def fail(*args):
        calls.append(args)
        raise RuntimeError('cleanup failed')

    monkeypatch.setattr(m, 'run', fail)
    with pytest.raises(RuntimeError, match='cleanup failed'):
        m.cleanup()
    assert len(calls) == 1
