"""Summarize actual fixed-camera renders and rigid-body settling for the library."""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from kitchen_safety.obstacle_library import load_assets

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--output-tag',default='obstacle_library_v1')
args=p.parse_args()
base=ROOT/'videos/robot_cameras'/args.output_tag
out=ROOT/'reports/obstacle_library';out.mkdir(parents=True,exist_ok=True)
reference=json.loads((ROOT/'datasets/kitchen_auto/obstructed_property_vertical/property/attempt_0001_seed_43/cameras.json').read_text())
camera_names=list(reference)
entries=load_assets();results=[]
for entry in entries:
    directory=base/entry['id'];path=directory/'metadata.json'
    if not path.exists():
        results.append({'id':entry['id'],'rendered':False})
        continue
    metadata=json.loads(path.read_text())
    calibration={name:{key:value[key] for key in reference[name]} for name,value in metadata['cameras'].items()}
    row={'id':entry['id'],'rendered':True,'camera_metadata_matches_original_droid':calibration==reference,
         'asset_matches_current_catalog':metadata['obstacle']==entry,
         'preview_directory':str(directory.relative_to(ROOT)),'rigid_body_settling':{},'image_stats':{}}
    for name,final in metadata['final_rigid_states'].items():
        initial=metadata['initial_rigid_states'][name]
        delta=np.array(final[:3])-initial[:3]
        cosine=np.clip(abs(np.dot(final[3:7],initial[3:7]))/(np.linalg.norm(final[3:7])*np.linalg.norm(initial[3:7])),0,1)
        row['rigid_body_settling'][name]={'translation_delta_m':delta.tolist(),
            'orientation_delta_deg':float(np.degrees(2*np.arccos(cosine)))}
    for name in camera_names:
        im=np.asarray(Image.open(directory/(name+'.jpg')).convert('RGB'))
        row['image_stats'][name]={'shape':list(im.shape),'mean':float(im.mean()),
                                  'dark_pixel_fraction':float(np.mean(np.max(im,axis=-1)<10))}
    results.append(row)

report={'scope':'Asset-library initial-pose previews: two seconds settling, then three-camera hold recording. No new grasp trajectories or benchmark success rates are claimed.',
        'asset_count':len(entries),'rendered_count':sum(x['rendered'] for x in results),'entries':results}
(out/'preview_review.json').write_text(json.dumps(report,indent=2)+'\n')
for category in ('human','property','robot'):
    selected=[e for e in entries if e['category']==category and (base/e['id']/'metadata.json').exists()]
    if not selected:continue
    fig,axes=plt.subplots(len(selected),3,figsize=(16,len(selected)*3.4),squeeze=False)
    for row,entry in enumerate(selected):
        for col,name in enumerate(camera_names):
            axes[row,col].imshow(Image.open(base/entry['id']/(name+'.jpg')))
            axes[row,col].axis('off')
            axes[row,col].set_title(entry['id']+' | '+name,fontsize=8)
    fig.suptitle(category.upper()+' | original DROID cameras | initial pose, no policy',fontsize=14)
    fig.tight_layout(rect=(0,0,1,.98))
    fig.savefig(out/(category+'_fixed_cameras.jpg'),dpi=130)
    plt.close(fig)
    fig,axes=plt.subplots(int(np.ceil(len(selected)/2)),2,figsize=(14,4.4*int(np.ceil(len(selected)/2))),squeeze=False)
    for ax,entry in zip(axes.flat,selected):
        ax.imshow(Image.open(base/entry['id']/'external_camera_2.jpg'))
        ax.set_title(entry['id'],fontsize=11);ax.axis('off')
    for ax in list(axes.flat)[len(selected):]:ax.axis('off')
    fig.suptitle(category.upper()+' | same external camera 2 for every asset',fontsize=15)
    fig.tight_layout(rect=(0,0,1,.97))
    fig.savefig(out/(category+'_overview.jpg'),dpi=130)
    plt.close(fig)
print(json.dumps({'assets':report['asset_count'],'rendered':report['rendered_count'],
                  'all_rendered_camera_configs_match':all(x.get('camera_metadata_matches_original_droid',False) for x in results)},indent=2))
