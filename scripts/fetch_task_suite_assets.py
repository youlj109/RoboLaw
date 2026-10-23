"""Mirror task-declared official NVIDIA USD dependency trees to a data cache.

No upload, credentials or renderer. USD relative references keep their exact
URL hierarchy. Built-in MDL resources remain in Isaac Sim's normal search path.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import posixpath
from urllib.parse import urljoin,urlparse,unquote
import time
import threading
from concurrent.futures import ThreadPoolExecutor,as_completed

ROOT=Path(__file__).resolve().parents[1]
HOSTS={'omniverse-content-staging.s3-us-west-2.amazonaws.com','omniverse-content-production.s3-us-west-2.amazonaws.com'}
CONSTANTS=dict(ARENA_NUCLEUS_DIR='https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/IsaacLab',
    ISAAC_STAGING_NUCLEUS_DIR='https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac')
BUILTIN_MDL={'OmniPBR.mdl','OmniGlass.mdl','OmniSurface.mdl','gltf/pbr.mdl'}


def dependencies(path):
    """Exclude assetInfo.identifier catalog labels, not real composition arcs.

    ExtractExternalReferences also returns assetInfo identifiers, which may be
    non-resolvable catalog names. They are not geometry/material dependencies.
    If a name also occurs in an actual reference, payload or asset attribute it
    remains a required dependency and a missing download remains an error.
    """
    from pxr import Sdf,UsdUtils
    layer=Sdf.Layer.FindOrOpen(str(path));identifiers=set();required=set(layer.subLayerPaths)
    def paths(value):
        if isinstance(value,Sdf.AssetPath):return {value.path}
        if isinstance(value,(list,tuple)) or type(value).__name__=='AssetPathArray':
            return {x.path for x in value if isinstance(x,Sdf.AssetPath)}
        return set()
    def visit(p):
        spec=layer.GetObjectAtPath(p)
        if isinstance(spec,Sdf.PrimSpec):
            identifiers.update(paths(dict(spec.assetInfo).get('identifier')))
            for editor in (spec.referenceList,spec.payloadList):
                required.update(x.assetPath for x in editor.GetAddedOrExplicitItems() if x.assetPath)
        elif isinstance(spec,Sdf.AttributeSpec):
            required.update(paths(spec.default))
            for t in layer.ListTimeSamplesForPath(p):required.update(paths(layer.QueryTimeSample(p,t)))
    layer.Traverse('/',visit)
    groups=UsdUtils.ExtractExternalReferences(str(path))
    all_paths=set().union(*map(set,groups))
    metadata_only=identifiers-required
    return sorted(all_paths-metadata_only),sorted(metadata_only)


def registry_urls():
    result={}
    def string(node):
        if isinstance(node,ast.Constant) and isinstance(node.value,str):return node.value
        if isinstance(node,ast.JoinedStr):
            return ''.join(string(v) if isinstance(v,ast.Constant) else CONSTANTS[v.value.id] for v in node.values)
        raise ValueError('Not a literal asset URI')
    directory=ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/assets'
    for file in directory.glob('*library.py'):
        for cls in ast.parse(file.read_text()).body:
            if not isinstance(cls,ast.ClassDef):continue
            fields={}
            for node in cls.body:
                if isinstance(node,ast.Assign) and len(node.targets)==1 and isinstance(node.targets[0],ast.Name):
                    try:fields[node.targets[0].id]=string(node.value)
                    except (KeyError,ValueError,AttributeError):pass
            if 'name' in fields and 'usd_path' in fields:result[fields['name']]=fields['usd_path']
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--cache',type=Path,required=True)
    p.add_argument('--task',action='append',help='Selected contract stem; omitted means the complete suite')
    p.add_argument('--inventory-only',action='store_true');args=p.parse_args()
    urls=registry_urls();assets={};sdk=[]
    for file in sorted((ROOT/'configs/risk_task_suite').glob('*.json')):
        if file.name=='suite.json' or args.task and file.stem not in args.task:continue
        task=json.loads(file.read_text());spec=task['arena_environment_spec']
        for obj in [spec['background']]+spec['objects']:
            uri=obj.get('params',{}).get('usd_path') or urls.get(obj['registry_name'])
            if uri and urlparse(uri).hostname in HOSTS:assets.setdefault(uri,[]).append(dict(task_id=file.stem,asset_id=obj['id']))
            else:sdk.append(dict(task_id=file.stem,asset_id=obj['id'],registry_name=obj['registry_name'],resolution='native SDK required'))
    args.cache.mkdir(parents=True,exist_ok=True)
    manifest=dict(roots=assets,sdk_assets=sdk,files={},errors=[],complete=False,
        builtin_mdl=sorted(BUILTIN_MDL),metadata_only_identifiers={},dependency_edges={})
    lock=threading.RLock()
    def save():
        with lock:
            tmp=args.cache/'official_manifest.json.part';tmp.write_text(json.dumps(manifest,ensure_ascii=False,indent=2));tmp.replace(args.cache/'official_manifest.json')
    save()
    if args.inventory_only:print(json.dumps(dict(official_roots=len(assets),native_sdk_assets=len(sdk))));return
    import requests
    local_sessions=threading.local();seen=set()
    def fetch(uri):
        parsed=urlparse(uri)
        if parsed.scheme!='https' or parsed.hostname not in HOSTS or parsed.query or parsed.fragment:
            raise ValueError('Unsupported external dependency URI: '+uri)
        path=posixpath.normpath(unquote(parsed.path));canonical='https://'+parsed.hostname+path
        with lock:
            if canonical in seen:return
            seen.add(canonical)
        local=args.cache/'official'/parsed.hostname/path.lstrip('/')
        local.parent.mkdir(parents=True,exist_ok=True)
        if not local.is_file():
            part=local.with_suffix(local.suffix+'.part')
            for attempt in range(3):
                try:
                    if not hasattr(local_sessions,'session'):local_sessions.session=requests.Session()
                    with local_sessions.session.get(canonical,stream=True,timeout=(20,90)) as response:
                        response.raise_for_status();total=0
                        with part.open('wb') as out:
                            for data in response.iter_content(1024*1024):out.write(data);total+=len(data)
                        expected=response.headers.get('Content-Length')
                        if expected and int(expected)!=total:raise ValueError('Incomplete content length')
                    part.replace(local);break
                except Exception:
                    if attempt==2:raise
                    time.sleep(1)
        sha=hashlib.sha256()
        with local.open('rb') as f:
            for block in iter(lambda:f.read(1024*1024),b''):sha.update(block)
        with lock:manifest['files'][canonical]=dict(path=str(local.resolve()),bytes=local.stat().st_size,sha256=sha.hexdigest())
        save()
        if local.suffix.lower() in ('.usd','.usda','.usdc'):
            references,metadata_only=dependencies(local)
            with lock:manifest['dependency_edges'][canonical]=[urljoin(canonical,dep) for dep in references if dep not in BUILTIN_MDL]
            if metadata_only:
                with lock:manifest['metadata_only_identifiers'][canonical]=metadata_only
            for dep in references:
                if dep in BUILTIN_MDL:continue
                if any(t in dep for t in ('<UDIM>','<UVTILE>')):raise ValueError('Texture tiles need explicit expansion: '+dep)
                fetch(urljoin(canonical,dep))
    with ThreadPoolExecutor(max_workers=4) as pool:
        jobs={pool.submit(fetch,uri):uri for uri in assets}
        for i,job in enumerate(as_completed(jobs)):
            try:job.result()
            except Exception as exc:manifest['errors'].append(dict(root_uri=jobs[job],error=repr(exc)))
            save();print(json.dumps(dict(root_index=i+1,total_roots=len(assets),files=len(manifest['files']),errors=len(manifest['errors']))),flush=True)
    manifest['complete']=not manifest['errors'];save()
    if not manifest['complete']:raise SystemExit(1)

if __name__=='__main__':main()
