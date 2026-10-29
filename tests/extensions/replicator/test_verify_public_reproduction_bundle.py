"""Tiny localhost HTTP tests; no ModelScope access or credential use."""
from contextlib import contextmanager
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest
from scripts.extensions.replicator import verify_public_reproduction_bundle as verifier


def sha(data):
    return hashlib.sha256(data).hexdigest()


@contextmanager
def fixture_server(tmp_path):
    payloads={}; counts={}; auth=[]; activity={'active':0,'max':0}; guard=threading.Lock(); fail_once=set()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def do_GET(self):
            with guard:
                counts[self.path]=counts.get(self.path,0)+1
                auth.append((self.headers.get('Authorization'),self.headers.get('Proxy-Authorization'),self.headers.get('Cookie')))
                activity['active']+=1;activity['max']=max(activity['max'],activity['active'])
                first_failure=self.path in fail_once and counts[self.path]==1
            try:
                time.sleep(.03)
                if first_failure:self.send_response(503);self.end_headers();return
                data=payloads.get(self.path)
                if data is None:self.send_response(404);self.end_headers();return
                self.send_response(200);self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
            finally:
                with guard:activity['active']-=1
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    folder=tmp_path/'publication';folder.mkdir();config=tmp_path/'config.json';receipt=folder/'receipts.json'
    prefix=f'http://127.0.0.1:{server.server_port}/bundle/'
    def spec(name,data):
        payloads['/bundle/'+name]=data
        return dict(url=prefix+name,bytes=len(data),sha256=sha(data))
    part=dict(id='core',group='inputs',**spec('core.tar.gz',b'small archive bytes'))
    part['manifest']=spec('core.manifest.json',b'{"files":[]}')
    lock=dict(schema_version=1,kind='robolaw_reproduction_bundle',bundle_id='test-v1',parts=[part],profiles={'all':['core']})
    config.write_text(json.dumps(lock));(folder/'bundle.json').write_text(json.dumps(lock))
    for name in verifier.METADATA:
        if name!='bundle.json':(folder/name).write_bytes(('metadata '+name).encode())
        payloads['/bundle/'+name]=(folder/name).read_bytes()
    value=SimpleNamespace(config=config,folder=folder,receipt=receipt,lock=lock,payloads=payloads,counts=counts,
                          auth=auth,activity=activity,fail_once=fail_once,prefix=prefix)
    try:yield value
    finally:server.shutdown();server.server_close();thread.join()


def run(fixture,**kw):
    return verifier.verify(fixture.config,fixture.folder,fixture.receipt,retry_delay=0,**kw)


def test_parallel_all_metadata_anonymous_and_resume_only_observed_success(tmp_path):
    with fixture_server(tmp_path) as f:
        report=run(f)
        assert report['passed'] and report['verified_objects']==7 and report['failed_objects']==0
        assert 2<=f.activity['max']<=4 and all(x==(None,None,None) for x in f.auth)
        assert set(report['objects'])=={'core.tar.gz','core.manifest.json',*verifier.METADATA}
        before=dict(f.counts);second=run(f)
        assert second['passed'] and len(second['reused_verified_objects'])==7 and f.counts==before
        # A superficially successful but incomplete receipt must not suppress GET.
        saved=json.loads(f.receipt.read_text());saved['objects']['core.tar.gz'].pop('observed_sha256');f.receipt.write_text(json.dumps(saved))
        third=run(f);assert third['passed'] and len(third['reused_verified_objects'])==6
        assert f.counts['/bundle/core.tar.gz']==2


def test_404_never_verified_and_recovery_skips_only_other_complete_gets(tmp_path):
    with fixture_server(tmp_path) as f:
        data=f.payloads.pop('/bundle/core.tar.gz');first=run(f)
        assert not first['passed'] and first['failed_objects']==1 and first['verified_objects']==6
        assert f.counts['/bundle/core.tar.gz']==3
        assert not first['objects']['core.tar.gz']['public_download_verified']
        f.payloads['/bundle/core.tar.gz']=data;before=dict(f.counts);second=run(f)
        assert second['passed'] and len(second['reused_verified_objects'])==6
        assert f.counts['/bundle/core.tar.gz']==4
        assert all(f.counts[k]==v for k,v in before.items() if k!='/bundle/core.tar.gz')


@pytest.mark.parametrize('bad',[b'x'*len(b'small archive bytes'),b'oversized response data that is bigger than the expected archive',b'x'])
def test_hash_and_byte_mismatch_never_produce_success_receipt(tmp_path,bad):
    with fixture_server(tmp_path) as f:
        f.payloads['/bundle/core.tar.gz']=bad
        result=run(f)
        assert not result['passed'] and result['objects']['core.tar.gz']['attempts_this_run']==3
        assert f.counts['/bundle/core.tar.gz']==3


def test_transient_503_retries_and_metadata_changes_invalidate_single_receipt(tmp_path):
    with fixture_server(tmp_path) as f:
        f.fail_once.add('/bundle/core.manifest.json');report=run(f)
        assert report['passed'] and report['objects']['core.manifest.json']['attempts_this_run']==2
        data=b'new published README';(f.folder/'README.md').write_bytes(data);f.payloads['/bundle/README.md']=data
        previous=dict(f.counts);next_report=run(f)
        assert next_report['passed'] and len(next_report['reused_verified_objects'])==6
        assert f.counts['/bundle/README.md']==previous['/bundle/README.md']+1


def test_changed_lock_invalidates_prior_verification_and_local_bundle_must_match(tmp_path):
    with fixture_server(tmp_path) as f:
        assert run(f)['passed'];before=dict(f.counts)
        f.lock['scope']='updated description';f.config.write_text(json.dumps(f.lock))
        with pytest.raises(ValueError,match='differs'):run(f)
        assert f.counts==before
        (f.folder/'bundle.json').write_text(json.dumps(f.lock));f.payloads['/bundle/bundle.json']=(f.folder/'bundle.json').read_bytes()
        second=run(f);assert second['passed'] and not second['reused_verified_objects']
        assert all(f.counts[k]==v+1 for k,v in before.items())


def test_credential_url_and_excess_retry_or_workers_refused(tmp_path):
    with fixture_server(tmp_path) as f:
        for kwargs in ({'attempts':4},{'workers':5}):
            with pytest.raises(ValueError):run(f,**kwargs)
        f.lock['parts'][0]['url']='http://user:secret@127.0.0.1/bundle/core.tar.gz'
        f.config.write_text(json.dumps(f.lock));(f.folder/'bundle.json').write_text(json.dumps(f.lock))
        with pytest.raises(ValueError,match='credential-free'):run(f)
        assert not f.counts


def test_receipt_binding_fields_and_frozen_output_protection(tmp_path):
    with fixture_server(tmp_path) as f:
        result=run(f);row=result['objects']['core.tar.gz'];spec={k:row[k] for k in ('name','url','bytes','sha256')}
        assert verifier.can_resume(row,spec)
        for key,value in [('url',row['url']+'?new'),('bytes',row['bytes']+1),('sha256','f'*64)]:
            assert not verifier.can_resume(row,dict(spec,**{key:value}))
        for key,value in [('public_download_verified',False),('authentication_used',True),('http_status',206),('observed_bytes',0),('observed_sha256','f'*64)]:
            assert not verifier.can_resume(dict(row,**{key:value}),spec)
        for target in (f.config,f.folder/'bundle.json',f.folder/'core.tar.gz',f.folder/'core.manifest.json'):
            with pytest.raises(ValueError,match='frozen publication'):
                verifier.verify(f.config,f.folder,target,retry_delay=0)


def test_subset_profile_cannot_overwrite_unselected_frozen_objects_via_aliases(tmp_path,monkeypatch):
    with fixture_server(tmp_path) as f:
        media_data=b'frozen media archive';manifest_data=b'frozen media manifest'
        media=dict(id='media',group='media',url=f.prefix+'media.tar.gz',bytes=len(media_data),sha256=sha(media_data),
                   manifest=dict(url=f.prefix+'media.manifest.json',bytes=len(manifest_data),sha256=sha(manifest_data)))
        f.lock['parts'].append(media);f.lock['profiles']={'all':['core','media'],'rerun':['core']}
        f.config.write_text(json.dumps(f.lock));(f.folder/'bundle.json').write_text(json.dumps(f.lock))
        (f.folder/'media.tar.gz').write_bytes(media_data);(f.folder/'media.manifest.json').write_bytes(manifest_data)
        alias=tmp_path/'alias';alias.symlink_to(f.folder,target_is_directory=True);monkeypatch.chdir(tmp_path)
        protected=[f.config,*(f.folder/name for name in verifier.METADATA),f.folder/'media.tar.gz',f.folder/'media.manifest.json',
                   Path('publication/media.tar.gz'),Path('publication/not-created/../media.manifest.json'),alias/'media.tar.gz']
        before={p:p.read_bytes() for p in (f.config,*f.folder.iterdir()) if p.is_file()}
        for target in protected:
            with pytest.raises(ValueError,match='frozen publication'):
                verifier.verify(f.config,f.folder,target,profile='rerun',retry_delay=0)
        assert not f.counts
        assert all(p.read_bytes()==data for p,data in before.items())
        assert not (f.folder/'not-created').exists()
