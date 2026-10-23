"""Verify one declared official asset's complete dependency tree before loading.

The downloaded manifest may come from another host. Resolve by original URI,
never by its author's absolute local path. An existing root USD alone does not
prove that its payloads, geometry and textures arrived successfully.
"""
import hashlib,json
from pathlib import Path
from urllib.parse import urlparse,unquote


def verify_root(cache,uri):
    cache=Path(cache);manifest=json.loads((cache/'official_manifest.json').read_text())
    if 'dependency_edges' not in manifest:raise ValueError('Asset manifest has no dependency closure; refresh it before loading')
    pending=[uri];seen=set();verified=[]
    while pending:
        name=pending.pop()
        if name in seen:continue
        seen.add(name)
        parsed=urlparse(name);parts=Path(unquote(parsed.path).lstrip('/'))
        if parsed.scheme!='https' or not parsed.hostname or '..' in parts.parts:
            raise ValueError('Invalid official dependency URI')
        record=manifest['files'].get(name)
        if record is None:raise ValueError('Dependency lacks download evidence: '+name)
        path=cache/'official'/parsed.hostname/parts
        if not path.is_file() or path.stat().st_size!=record['bytes']:
            raise ValueError('Required official dependency has not arrived: '+str(path))
        h=hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
        if h.hexdigest()!=record['sha256']:raise ValueError('Official dependency hash differs: '+str(path))
        verified.append(dict(uri=name,sha256=h.hexdigest(),bytes=record['bytes']))
        if path.suffix.lower() in ('.usd','.usda','.usdc'):
            if name not in manifest['dependency_edges']:raise ValueError('USD dependency closure is missing: '+name)
            pending.extend(manifest['dependency_edges'][name])
    return dict(root_uri=uri,complete=True,files=sorted(verified,key=lambda x:x['uri']),
        builtin_mdl=manifest.get('builtin_mdl',[]),scope='downloaded dependency content, not scene/task acceptance')
