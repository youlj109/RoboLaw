"""Verify the pinned source-asset manifest and source kitchen composition."""
import hashlib,json
from pathlib import Path
from pxr import Usd
ROOT=Path(__file__).resolve().parents[3]
manifest=json.loads((ROOT/'configs/extensions/replicator/external_assets.json').read_text())
errors=[]
for row in manifest['files']:
 path=ROOT/row['path']
 if not path.is_file():errors.append(dict(path=row['path'],error='missing'));continue
 if path.stat().st_size!=row['bytes'] or hashlib.sha256(path.read_bytes()).hexdigest()!=row['sha256']:
  errors.append(dict(path=row['path'],error='size or SHA256 mismatch'))
if not errors:
 stage=Usd.Stage.Open(str(ROOT/'assets/extensions/replicator/source/Isaac/Environments/replicator_kitchen/kitchen_g_shape.usda'))
 if not stage:errors.append(dict(error='source stage did not open'))
 else:errors.extend(dict(error=str(item)) for item in stage.GetCompositionErrors())
report=dict(passed=not errors,expected_files=len(manifest['files']),errors=errors,scope='Pinned bytes/SHA256 and USD composition; no physics or renderer validation claim')
p=ROOT/'reports/skill_trials/replicator/asset_verification.json';p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
raise SystemExit(0 if report['passed'] else 1)
