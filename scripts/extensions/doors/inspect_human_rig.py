"""Run inside Blender; inspect source rig without altering the source blend."""
import bpy,json
from pathlib import Path
report={'blender':bpy.app.version_string,'objects':[]}
for ob in bpy.data.objects:
 item={'name':ob.name,'type':ob.type,'modifiers':[{'name':m.name,'type':m.type,'object':m.object.name if m.type=='ARMATURE' and m.object else None} for m in ob.modifiers]}
 if ob.type=='ARMATURE':
  item['bones']=[{'name':b.name,'head':list(b.head),'tail':list(b.tail),'rotation_mode':b.rotation_mode,'parent':b.parent.name if b.parent else None,'constraints':[{'name':c.name,'type':c.type} for c in b.constraints]} for b in ob.pose.bones]
 report['objects'].append(item)
Path('reports/avoidance_trials/doors/human_source_rig.json').write_text(json.dumps(report,indent=2))
print('SOURCE_RIG',[(o['name'],o['type'],len(o.get('bones',[]))) for o in report['objects']])
