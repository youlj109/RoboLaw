"""Restore exact original B05 assets and relative dependencies; no substitutions."""
import os,json,re,hashlib
from pathlib import Path
from urllib.request import urlopen,Request
from urllib.parse import urljoin
os.environ.pop('PXR_PLUGINPATH_NAME',None)
from pxr import Usd,Sdf
ROOT=Path(__file__).resolve().parents[3]
PREFIXES=['https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/','https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/']
STAGE=PREFIXES[1]
starts=[PREFIXES[0]+'Isaac/SimReady/Residential/Kitchen/'+s for s in ['Food/Canned_Goods/Can_M01/sm_food_beverage_can_m01_01.usd','Baskets/Plastic_Basket_A01/sm_misc_basket_plastic_a01_01.usd']]+[STAGE+'Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/objects/'+s for s in ['ycb/tuna_can.usd','hope/corn_can.usd']]+[STAGE+'Isaac/IsaacLab/Objects/Mug/mug.usd']
def local(url):
 prefix=next(x for x in PREFIXES if url.startswith(x))
 return ROOT/'assets/extensions/lightwheel_pick_place/downloads_b05'/str(PREFIXES.index(prefix))/url[len(prefix):]
def main():
 os.chdir(ROOT);pending=set(starts);seen=set();rows=[];errors=[]
 while pending:
  url=pending.pop()
  if url in seen:continue
  seen.add(url)
  try:
   print('FETCH',url,flush=True)
   path=local(url);path.parent.mkdir(parents=True,exist_ok=True)
   if not path.exists():
    with urlopen(Request(url,headers={'User-Agent':'RoboLaw original asset restore'}),timeout=90) as response: data=response.read()
    tmp=path.with_suffix(path.suffix+'.part');tmp.write_bytes(data);tmp.replace(path)
   rows.append(dict(url=url,path=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),bytes=path.stat().st_size));print('RESTORED',url,flush=True)
   if path.suffix.lower() in ('.usd','.usda','.usdc'):
    layer=Sdf.Layer.FindOrOpen(str(path.relative_to(ROOT)))
    if not layer:raise RuntimeError('Cannot open downloaded USD')
    refs=list(layer.GetCompositionAssetDependencies());text=layer.ExportToString();refs += [v for line in text.splitlines() if line.lstrip().startswith(('asset ','asset[] ')) and not line.lstrip().startswith('asset identifier =') for v in re.findall(r'@([^@]+)@',line)]
   elif path.suffix=='.mdl':refs=re.findall(r'texture_2d\(\s*"([^"\n]+)"',path.read_text(errors='replace'))
   else:refs=[]
   for ref in set(refs):
    if ref in ('OmniPBR.mdl','OmniGlass.mdl','OmniSurface.mdl','OmniSurfaceLite.mdl'):continue
    dep=urljoin(url,ref)
    if any(dep.startswith(x) for x in PREFIXES):pending.add(dep)
  except Exception as exc:errors.append(dict(url=url,error=repr(exc)));print('FAILED',url,repr(exc),flush=True)
 report=dict(files=rows,errors=errors,complete=not errors,scope='Exact original B05 assets; no geometry or physics edits')
 out=ROOT/'reports/skill_trials/lightwheel_pick_place/b05_assets.json';out.write_text(json.dumps(report,indent=2));return bool(errors)
if __name__=='__main__':raise SystemExit(main())
