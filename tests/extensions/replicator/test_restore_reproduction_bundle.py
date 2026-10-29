"""Small hostile archives and real HTTP Range exercise the standard-library restore."""
from contextlib import contextmanager
from copy import deepcopy
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import os
from pathlib import Path
import tarfile
import threading
from types import SimpleNamespace

import pytest
from scripts.extensions.replicator import restore_reproduction_bundle as restore


def sha(data):
    return hashlib.sha256(data).hexdigest()


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True))
    return dict(bytes=path.stat().st_size, sha256=restore.digest(path), url='https://example.invalid/' + path.name)


@pytest.fixture
def bundle(tmp_path):
    root = tmp_path / 'checkout'; root.mkdir()
    cache = tmp_path / 'cache'; cache.mkdir()
    lock_path = tmp_path / 'lock.json'
    lock = dict(schema_version=1, kind='robolaw_reproduction_bundle', bundle_id='fixture-v1',
                allowed_roots=['reports', 'exports'], default_profile='rerun', parts=[],
                profiles={'rerun':['core'], 'evidence':['core','native'], 'media':['media'], 'all':['core','native','media']})
    payloads = {'core':{'reports/result.json':b'{"frozen": true}'},
                'native':{'exports/physics.bin':b'original-state'}, 'media':{'exports/video.mp4':b'video'}}
    def part(part_id, files=None, members=None):
        files = payloads[part_id] if files is None else files
        rows = [dict(path=name, bytes=len(data), sha256=sha(data)) for name,data in files.items()]
        manifest = dict(kind='robolaw_reproduction_part', bundle_id=lock['bundle_id'], part_id=part_id, files=rows)
        ref = write_json(cache/(part_id+'.manifest.json'), manifest)
        archive = cache/(part_id+'.tar.gz')
        with tarfile.open(archive, 'w:gz') as tar:
            if members is None:
                for name,data in files.items():
                    info=tarfile.TarInfo(name);info.size=len(data);tar.addfile(info,io.BytesIO(data))
            else:
                for info,data in members:
                    tar.addfile(info,io.BytesIO(data) if data is not None else None)
        spec=dict(id=part_id,group='fixture',url='https://example.invalid/'+archive.name,
                  bytes=archive.stat().st_size,sha256=restore.digest(archive),manifest=ref)
        lock['parts']=[p for p in lock['parts'] if p['id']!=part_id]+[spec]
        write_json(lock_path,lock)
        return manifest, spec
    for key in payloads:
        part(key)
    return SimpleNamespace(root=root, cache=cache, config=lock_path, lock=lock, part=part, payloads=payloads)


def run(fixture, **kwargs):
    return restore.restore(fixture.config, fixture.root, fixture.cache, **kwargs)


def test_profiles_idempotent_and_verify_never_downloads_or_writes(bundle,monkeypatch):
    outcome=run(bundle,offline=True)
    assert outcome['restored']==1 and (bundle.root/'reports/result.json').read_bytes()==bundle.payloads['core']['reports/result.json']
    assert not (bundle.root/'exports').exists()
    (bundle.cache/'core.tar.gz').unlink()
    assert run(bundle,offline=True)['restored']==0
    monkeypatch.setattr(restore,'download',lambda *a,**k: pytest.fail('verify-only must not download'))
    before={str(p):p.stat().st_mtime_ns for p in bundle.root.rglob('*')}
    assert run(bundle,verify_only=True)['verified']
    assert before=={str(p):p.stat().st_mtime_ns for p in bundle.root.rglob('*')}
    with pytest.raises(ValueError,match='files missing'):
        run(bundle,profile='all',verify_only=True)
    assert not (bundle.root/'exports').exists()


def test_missing_manifest_verify_only_has_no_network_or_directory_creation(bundle,monkeypatch):
    (bundle.cache/'core.manifest.json').unlink()
    monkeypatch.setattr(restore,'urlopen',lambda *a,**k:pytest.fail('network called'))
    with pytest.raises(ValueError,match='Verified manifest unavailable'):
        run(bundle,verify_only=True)
    assert list(bundle.root.iterdir())==[]


def test_all_existing_conflicts_checked_before_any_install(bundle):
    target=bundle.root/'exports/physics.bin';target.parent.mkdir();target.write_bytes(b'local edits')
    with pytest.raises(ValueError,match='Existing file differs'):
        run(bundle,profile='evidence',offline=True)
    assert target.read_bytes()==b'local edits' and not (bundle.root/'reports').exists()


@pytest.mark.parametrize('name',['../escape','/absolute','reports/../escape','reports//bad','reports\\bad','C:/escape','reports/.git/config'])
def test_manifest_unsafe_paths_refused(bundle,name):
    bundle.part('core',{name:b'bad'})
    with pytest.raises(ValueError):run(bundle,offline=True)
    assert list(bundle.root.iterdir())==[]


@pytest.mark.parametrize('kind',['symlink','hardlink','duplicate','unlisted','wrong-size','directory'])
def test_entire_archive_headers_checked_before_install(bundle,kind):
    first=tarfile.TarInfo('reports/result.json');first.size=2
    extra=tarfile.TarInfo('reports/other');data=None
    if kind=='symlink':extra.type=tarfile.SYMTYPE;extra.linkname='../../outside'
    if kind=='hardlink':extra.type=tarfile.LNKTYPE;extra.linkname='reports/result.json'
    if kind=='duplicate':extra.name='reports/result.json';extra.size=2;data=b'ok'
    if kind=='unlisted':extra.name='reports/not-listed';extra.size=1;data=b'x'
    if kind=='wrong-size':extra.size=1;data=b'x'
    if kind=='directory':extra.type=tarfile.DIRTYPE
    bundle.part('core',{'reports/result.json':b'ok','reports/other':b'other'},[(first,b'ok'),(extra,data)])
    with pytest.raises(ValueError):run(bundle,offline=True)
    assert list(bundle.root.iterdir())==[]


@pytest.mark.parametrize('parent_link',[True,False])
def test_existing_symlink_never_followed(bundle,tmp_path,parent_link):
    outside=tmp_path/'outside';outside.mkdir();sentinel=outside/'result.json';sentinel.write_bytes(b'local')
    if parent_link:(bundle.root/'reports').symlink_to(outside,target_is_directory=True)
    else:
        (bundle.root/'reports').mkdir();(bundle.root/'reports/result.json').symlink_to(sentinel)
    with pytest.raises((ValueError,OSError)):run(bundle,offline=True)
    assert sentinel.read_bytes()==b'local'


def test_cross_part_path_duplicates_and_archive_hash_rejected(bundle):
    bundle.part('native',bundle.payloads['core'])
    with pytest.raises(ValueError,match='Duplicate paths across'):run(bundle,profile='evidence',offline=True)
    archive=bundle.cache/'core.tar.gz';archive.write_bytes(archive.read_bytes()+b'corrupt')
    with pytest.raises(ValueError,match='Verified cache file unavailable'):run(bundle,offline=True)
    assert list(bundle.root.iterdir())==[]


def test_member_hash_mismatch_never_published_and_no_temporary_file_left(bundle):
    info=tarfile.TarInfo('reports/result.json');info.size=4
    bundle.part('core',{'reports/result.json':b'good'},[(info,b'evil')])
    with pytest.raises(ValueError,match='Extracted file SHA'):run(bundle,offline=True)
    assert not (bundle.root/'reports/result.json').exists()
    assert list((bundle.root/'reports').iterdir())==[]


def test_concurrent_destination_creation_is_not_overwritten(bundle,monkeypatch):
    original=os.link
    def competing(source,target,**kwargs):
        fd=os.open(target,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600,dir_fd=kwargs['dst_dir_fd'])
        os.write(fd,b'concurrent edit');os.close(fd)
        return original(source,target,**kwargs)
    monkeypatch.setattr(restore.os,'link',competing)
    with pytest.raises(FileExistsError):run(bundle,offline=True)
    assert (bundle.root/'reports/result.json').read_bytes()==b'concurrent edit'
    assert len(list((bundle.root/'reports').iterdir()))==1


@contextmanager
def server(data,mode):
    requests=[]
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def do_GET(self):
            value=self.headers.get('Range');requests.append(value)
            start=int(value.removeprefix('bytes=').removesuffix('-')) if value else 0
            partial=bool(value) and mode!='ignore'
            self.send_response(206 if partial else 200)
            if partial:
                self.send_header('Content-Range',f'bytes {start+(1 if mode=="wrong-range" else 0)}-{len(data)-1}/{len(data)}')
            body=data[start:] if partial else data
            self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
    http=ThreadingHTTPServer(('127.0.0.1',0),Handler);worker=threading.Thread(target=http.serve_forever,daemon=True);worker.start()
    try:yield f'http://127.0.0.1:{http.server_port}/bundle',requests
    finally:http.shutdown();http.server_close();worker.join()


@pytest.mark.parametrize('mode',['range','ignore','wrong-range'])
def test_download_resume_validates_range_and_full_sha(tmp_path,mode):
    data=b'0123456789'*100;target=tmp_path/'part.tar.gz';partial=tmp_path/'part.tar.gz.partial';partial.write_bytes(data[:321])
    with server(data,mode) as (url,requests):
        spec=dict(url=url,bytes=len(data),sha256=sha(data))
        if mode=='wrong-range':
            with pytest.raises(ValueError,match='Content-Range'):restore.download(spec,target)
            assert not target.exists() and partial.read_bytes()==data[:321]
        else:
            restore.download(spec,target);assert target.read_bytes()==data and not partial.exists()
        assert requests==['bytes=321-']


def test_manifest_bytes_cannot_change_after_lock_and_offline_never_fetches(bundle,monkeypatch):
    manifest=bundle.cache/'core.manifest.json';manifest.write_bytes(manifest.read_bytes()+b' ')
    monkeypatch.setattr(restore,'urlopen',lambda *a,**k:pytest.fail('offline must not fetch'))
    with pytest.raises(ValueError,match='Verified cache file unavailable'):
        run(bundle,offline=True)
    assert list(bundle.root.iterdir())==[]
