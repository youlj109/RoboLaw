import bpy,json
from mathutils import Vector
rig=bpy.data.objects['KitchenAdult.rig']
print('RIG',list(map(list,rig.matrix_world)))
for n in ('Root','pelvis','spine_01','spine_02','spine_03','clavicle_r','upperarm_r','lowerarm_r','hand_r','upperarm_l','lowerarm_l','hand_l','thigh_r','calf_r','foot_r','ball_r','thigh_l','calf_l','foot_l','ball_l','head'):
 b=rig.pose.bones[n];rest=rig.data.bones[n]
 print(n,json.dumps({'head_world':list(rig.matrix_world@b.head),'tail_world':list(rig.matrix_world@b.tail),'rest_head_world':list(rig.matrix_world@rest.head_local),'rest_tail_world':list(rig.matrix_world@rest.tail_local),'basis':list(map(list,b.matrix_basis)),'rotation_mode':b.rotation_mode}))
print('OBJECTS',[(o.name,len(o.data.vertices),[(m.name,m.type) for m in o.modifiers]) for o in bpy.data.objects if o.type=='MESH'])
