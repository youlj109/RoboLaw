"""Open a scene in Isaac Sim, step physics, and render two inspection images."""
import argparse
from pathlib import Path
parser=argparse.ArgumentParser()
parser.add_argument('variant',choices=['base','human','property','robot','human_nvidia','human_mpfb'])
parser.add_argument('--steps',type=int,default=120)
args=parser.parse_args()
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1280,'height':960,'renderer':'RaytracedLighting'})
import omni.usd
import omni.timeline
import omni.replicator.core as rep
from PIL import Image
from pxr import UsdGeom
root=Path('/workspace')
omni.usd.get_context().open_stage(str(root/'scenes'/f'{args.variant}.usda'))
for _ in range(30):app.update()
products=[]
names=['Overview','Detail']
if args.variant.startswith('human_'): names.append('Portrait')
for name in names:
    rp=rep.create.render_product('/World/'+name,(1280,960))
    rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(rp)
    products.append((name,rgb))
timeline=omni.timeline.get_timeline_interface();timeline.play()
for _ in range(args.steps):app.update()
for _ in range(20):rep.orchestrator.step(rt_subframes=4)
for name,rgb in products:
    arr=rgb.get_data()
    assert arr.size, 'Camera returned empty image'
    Image.fromarray(arr).save(str(root/'previews'/f'{args.variant}_{name.lower()}.png'))
stage=omni.usd.get_context().get_stage()
bbox=UsdGeom.BBoxCache(0,['default','render'])
for name in ['Mustard','Bowl','Glass','Plate','Phone']:
    p=stage.GetPrimAtPath('/World/'+name)
    if p:print(name,bbox.ComputeWorldBound(p).ComputeAlignedRange(),flush=True)
print('PREVIEW_COMPLETE',args.variant,flush=True)
timeline.stop();app.close()
