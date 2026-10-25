"""Package only the independently accepted episodes referenced by a B02 manifest."""
import argparse,json,zipfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('manifest',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
m=json.loads(a.manifest.read_text())
if m.get('passed') is not True:raise SystemExit('Manifest has not passed; refusing a success package')
readme=['B02 lemon shelf to plate: one physical success per safety variant.',
        'Acceptance scope: '+m['acceptance_scope'],
        'Full original review gates passed: '+str(m['full_review_gates_passed']),
        'Excluded by user: unrelated global kitchen-fixture joint neutral quality.',
        'Task-object, robot, collision, grasp, placement and effective avoidance gates remain required.',
        'Three paired planning seeds cover transfer only, not a whole-task success rate.',
        'Original videos are included without editing. Failed trials remain on the server.','']
a.output.parent.mkdir(parents=True,exist_ok=True)
with zipfile.ZipFile(a.output,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=3) as z:
 z.write(a.manifest,'acceptance_manifest.json')
 for name,row in m['episodes'].items():
  ep=Path(row['episode']);readme.append(name+': '+str(ep))
  for f in sorted(ep.glob('*.mp4')):z.write(f,name+'/'+f.name,compress_type=zipfile.ZIP_STORED)
  for f in sorted(ep.glob('*audit*.json')):z.write(f,name+'/'+f.name)
  for n in ('result.json','validation.json','cameras.json'):
   if (ep/n).is_file():z.write(ep/n,name+'/'+n)
 z.writestr('README.txt','\n'.join(readme)+'\n')
print(a.output, a.output.stat().st_size)
