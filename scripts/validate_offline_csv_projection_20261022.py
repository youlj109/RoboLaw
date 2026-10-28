import csv,json
from copy import deepcopy
from pathlib import Path
from scripts.evaluate_raw_capture import OfflineObserver
from scripts.export_offline_risk_features import project
root=Path('/workspace');source=root/'datasets/diagnostic/raw_capture_only_full_20261022/env_000'
result=root/'datasets/diagnostic/offline_risk_full_20261022'
r=json.loads(next((result/'safety_decisions.jsonl').open()))
expected=project(deepcopy(r))
for old,new in [('decision_wall_time_ns','processing_start_wall_ns'),('evaluation_complete_time_ns','processing_complete_wall_ns'),('decision_monotonic_ns','processing_start_monotonic_ns'),('evaluation_complete_monotonic_ns','processing_complete_monotonic_ns')]:r[old]=r.pop(new)
out=root/'datasets/diagnostic/offline_csv_projection_validation_20261022';out.mkdir()
o=OfflineObserver(out,json.loads((source/'safety_observer_binding.json').read_text()))
o.source_directory=source;o.source_packet={'receive_wall_ns':r['source_capture_receive_wall_ns']};o.records=[None]
o._line(o.decisions,r);o.decisions.close();o.csvfile.close()
row=next(csv.DictReader((out/'risk_features.csv').open()))
assert row=={k:'' if v is None else str(v) for k,v in expected.items()}
print(json.dumps({'passed':True,'columns':len(row),'pt_nearest_asset':row['pt_nearest_asset'],'pt_distance_m':row['pt_min_included_distance_m']}))
