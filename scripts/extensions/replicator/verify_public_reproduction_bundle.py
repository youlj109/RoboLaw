#!/usr/bin/env python3
"""Stream public bundle objects in parallel and checkpoint verified SHA receipts.

No SDK, login, cookies, credentials, proxy environment, archive writes or extraction.
Only the receipt JSON is written. Failed objects are retried on the next invocation.
"""
from __future__ import annotations
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
import time
from urllib.parse import urlsplit, urlunsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / 'configs/extensions/replicator/reproduction_bundle_20261029.json'
FOLDER = ROOT / 'exports/robolaw_reproduction_20261029_r13_v1'
METADATA = ('bundle.json', 'README.md', 'SHA256SUMS', 'archive_verification.json',
            'isolated_restored_input_validation.json')
CHUNK = 1024 * 1024
PROTOCOL = 'anonymous-full-object-sha256-v1'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def now():
    return datetime.now(timezone.utc).isoformat()


def file_sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def public_url(value):
    parsed = urlsplit(value)
    require(parsed.scheme in ('http', 'https') and parsed.hostname and not parsed.username
            and not parsed.password and not parsed.fragment, 'Expected a credential-free HTTP(S) URL')
    return parsed


class PublicRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        public_url(newurl)
        require(not (urlsplit(req.full_url).scheme == 'https' and urlsplit(newurl).scheme != 'https'),
                'Refusing HTTPS downgrade')
        return super().redirect_request(req, fp, code, msg, headers, newurl)


def inventory(config, folder, profile='all'):
    config, folder = Path(config), Path(folder)
    lock = json.loads(config.read_text())
    require(lock.get('kind') == 'robolaw_reproduction_bundle' and lock.get('schema_version') == 1,
            'Unsupported bundle lock')
    require(profile in lock['profiles'], 'Unknown profile: ' + profile)
    selected = set(lock['profiles'][profile])
    require(selected and selected <= {p['id'] for p in lock['parts']}, 'Invalid profile members')
    require(json.loads((folder / 'bundle.json').read_text()) == lock, 'Local bundle.json differs from pinned lock')
    first = public_url(lock['parts'][0]['url'])
    prefix = urlunsplit((first.scheme, first.netloc, first.path.rsplit('/', 1)[0] + '/', '', ''))
    records = []
    for part in lock['parts']:
        if part['id'] in selected:
            records.extend([dict(part, object_kind='archive'), dict(part['manifest'], object_kind='manifest')])
    objects = []
    seen = set()
    for record in records:
        url = record['url']; public_url(url)
        require(url.startswith(prefix), 'Part URL is outside bundle directory')
        name = url[len(prefix):]
        require(name and '/' not in name and '?' not in name and name not in ('.', '..'), 'Invalid object name')
        require(type(record.get('bytes')) is int and record['bytes'] >= 0
                and re.fullmatch('[0-9a-f]{64}', str(record.get('sha256', ''))), 'Invalid size/SHA binding')
        require(name not in seen, 'Duplicate public object: ' + name); seen.add(name)
        objects.append(dict(name=name, url=url, bytes=record['bytes'], sha256=record['sha256'],
                            object_kind=record['object_kind']))
    for name in METADATA:
        path = folder / name
        require(path.is_file() and not path.is_symlink(), 'Missing local publication metadata: ' + name)
        require(name not in seen, 'Duplicate metadata object'); seen.add(name)
        objects.append(dict(name=name, url=prefix + name, bytes=path.stat().st_size,
                            sha256=file_sha(path), object_kind='publication_metadata'))
    return lock, objects, file_sha(config)


def can_resume(row, spec):
    return (isinstance(row, dict) and row.get('protocol') == PROTOCOL
            and row.get('public_download_verified') is True and row.get('authentication_used') is False
            and row.get('http_status') == 200 and isinstance(row.get('completed_at'), str)
            and all(row.get(key) == spec[key] for key in ('name', 'url', 'bytes', 'sha256'))
            and row.get('observed_bytes') == spec['bytes']
            and row.get('observed_sha256') == spec['sha256'])


def stream_public(spec, attempts=3, timeout=120, retry_delay=1):
    # A fresh opener has no cookie jar/auth handler; disable environment proxies
    # so proxy URL credentials cannot be sent implicitly.
    opener = build_opener(ProxyHandler({}), PublicRedirect())
    failures = []
    for attempt in range(1, attempts + 1):
        count = 0; digest = hashlib.sha256(); status = None; started = time.monotonic()
        try:
            request = Request(spec['url'], headers={'User-Agent': 'RoboLaw-public-verifier/1',
                                                     'Accept-Encoding': 'identity'})
            with opener.open(request, timeout=timeout) as response:
                status = response.status
                require(status == 200, 'Expected full public GET HTTP 200, got ' + str(status))
                for block in iter(lambda: response.read(CHUNK), b''):
                    count += len(block)
                    require(count <= spec['bytes'], 'Public download exceeds pinned byte count')
                    digest.update(block)
            require(count == spec['bytes'], 'Public download byte count mismatch')
            require(digest.hexdigest() == spec['sha256'], 'Public download SHA-256 mismatch')
            return dict(spec, protocol=PROTOCOL, public_download_verified=True, authentication_used=False,
                        http_status=status, observed_bytes=count, observed_sha256=digest.hexdigest(),
                        completed_at=now(), attempts_this_run=attempt, prior_attempt_errors=failures,
                        elapsed_seconds=time.monotonic()-started)
        except Exception as error:
            failures.append(dict(attempt=attempt, error=type(error).__name__ + ': ' + str(error)[:1000],
                                 observed_bytes=count, http_status=status))
            if attempt < attempts:
                time.sleep(retry_delay * attempt)
    return dict(spec, protocol=PROTOCOL, public_download_verified=False, authentication_used=False,
                attempts_this_run=attempts, failed_at=now(), prior_attempt_errors=failures)


def checkpoint(path, report):
    path = Path(path)
    require(not path.is_symlink(), 'Receipt may not be a symlink')
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name+'.', suffix='.tmp', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2); stream.write('\n')
            stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary): os.unlink(temporary)


def verify(config=CONFIG, folder=FOLDER, receipt=None, profile='all', workers=4,
           attempts=3, timeout=120, retry_delay=1):
    require(type(workers) is int and 1 <= workers <= 4, 'workers must be 1..4')
    require(type(attempts) is int and 1 <= attempts <= 3, 'attempts must be 1..3')
    require(timeout > 0 and retry_delay >= 0, 'Invalid timeout/retry delay')
    lock, objects, config_sha = inventory(config, folder, profile)
    receipt = Path(receipt) if receipt else Path(folder) / 'public_verification_receipts.json'
    # Protect every frozen object even when verifying a smaller profile.
    all_names = set(METADATA)
    first = public_url(lock['parts'][0]['url'])
    prefix = urlunsplit((first.scheme, first.netloc, first.path.rsplit('/', 1)[0] + '/', '', ''))
    for part in lock['parts']:
        for binding in (part, part['manifest']):
            url = binding['url']; public_url(url)
            require(url.startswith(prefix), 'Part URL is outside bundle directory')
            name = url[len(prefix):]
            require(name and '/' not in name and '?' not in name and name not in ('.', '..'), 'Invalid object name')
            all_names.add(name)
    protected = {Path(config).resolve(), *((Path(folder) / name).resolve() for name in all_names)}
    require(receipt.resolve() not in protected, 'Receipt cannot overwrite a frozen publication input')
    # Only a previously observed complete GET with this exact lock/spec may skip.
    old = {}
    require(not receipt.is_symlink(), 'Receipt may not be a symlink')
    if receipt.exists():
        prior = json.loads(receipt.read_text())
        if (prior.get('kind') == 'public_reproduction_bundle_verification'
                and prior.get('bundle_id') == lock['bundle_id']
                and prior.get('source_config_sha256') == config_sha):
            old = prior.get('objects', {})
    rows = {}; pending = []; reused = []
    for spec in objects:
        row = old.get(spec['name'])
        if can_resume(row, spec): rows[spec['name']] = row; reused.append(spec['name'])
        else: pending.append(spec)
    report = dict(kind='public_reproduction_bundle_verification', schema_version=1,
                  protocol=PROTOCOL, bundle_id=lock['bundle_id'], source_config_sha256=config_sha,
                  profile=profile, started_at=now(), authentication_used=False, workers=workers,
                  expected_objects=len(objects), expected_bytes=sum(x['bytes'] for x in objects),
                  reused_verified_objects=reused, objects=rows, complete=False, passed=False)
    def save():
        report['updated_at'] = now()
        report['verified_objects'] = sum(can_resume(rows.get(s['name']), s) for s in objects)
        report['failed_objects'] = sum(s['name'] in rows and not can_resume(rows[s['name']], s) for s in objects)
        report['verified_bytes'] = sum(s['bytes'] for s in objects if can_resume(rows.get(s['name']), s))
        checkpoint(receipt, report)
    save()
    with ThreadPoolExecutor(max_workers=workers) as pool:
        jobs = {pool.submit(stream_public, s, attempts, timeout, retry_delay):s for s in pending}
        for job in as_completed(jobs):
            spec = jobs[job]
            row = job.result(); rows[spec['name']] = row; save()
            print(json.dumps(dict(object=spec['name'], public_download_verified=row['public_download_verified'],
                                  verified=report['verified_objects'], total=len(objects),
                                  attempts=row['attempts_this_run']), ensure_ascii=False), flush=True)
    report['complete'] = True
    report['passed'] = all(can_resume(rows.get(s['name']), s) for s in objects)
    report['finished_at'] = now(); save()
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=CONFIG)
    parser.add_argument('--folder', type=Path, default=FOLDER)
    parser.add_argument('--receipt', type=Path)
    parser.add_argument('--profile', default='all')
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--attempts', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=120)
    args = parser.parse_args(argv)
    result = verify(**vars(args))
    print(json.dumps({k:v for k,v in result.items() if k not in ('objects','reused_verified_objects')}, ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
