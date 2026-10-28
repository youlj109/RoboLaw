"""Blender CPU: original MPFB rig, planted-foot gait and per-frame arm IK.

Run from the repository root with Blender 4.0.2 and the original posed.blend:
  blender -b assets/people/mpfb_corridor/posed.blend --python this.py
The exported /Person is relative to the final standing position. Add the scene
reference position (3.67,-0.92,0.0025) and yaw 125 degrees exactly once when referencing it.
"""
from __future__ import annotations
import argparse,hashlib,json,math,sys
from pathlib import Path
import bpy
from mathutils import Vector,Matrix

p=argparse.ArgumentParser();p.add_argument('--asset-dir',default='assets/extensions/doors/hard_demo_v2/person');p.add_argument('--frames',type=int,default=168)
args=p.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else [])
ROOT=Path.cwd();OUT=ROOT/args.asset_dir;OUT.mkdir(parents=True,exist_ok=True)
FPS=12;FRAMES=args.frames;source=ROOT/'assets/people/mpfb_corridor/posed.blend'
rig=bpy.data.objects['KitchenAdult.rig'];scene=bpy.context.scene
scene.render.engine='BLENDER_EEVEE';scene.render.fps=FPS;scene.frame_start=0;scene.frame_end=FRAMES-1
rig.animation_data_clear()
for b in rig.pose.bones:
 b.rotation_mode='QUATERNION';b.matrix_basis=Matrix.Identity(4)
bpy.context.view_layer.update()
RIG=rig.matrix_world.copy();INV=RIG.inverted();UP=Vector((0,0,1))
rest={b.name:RIG@b.matrix.copy() for b in rig.pose.bones}
rest_heads={b.name:RIG@b.head.copy() for b in rig.pose.bones}
rest_tails={b.name:RIG@b.tail.copy() for b in rig.pose.bones}
F=((rest_heads['ball_l']+rest_heads['ball_r'])-(rest_heads['foot_l']+rest_heads['foot_r']));F.z=0;F.normalize()
RIGHT=(rest_heads['upperarm_r']-rest_heads['upperarm_l']);RIGHT.z=0;RIGHT.normalize()
# Body still faces the fridge. The approach begins 18 cm farther from the
# stove, then travels diagonally into the unchanged final standing pose.
PATH=F-RIGHT*(.18/.85)
# A root object carries locomotion; deforming bones remain in asset coordinates.
root=bpy.data.objects.new('MotionRoot',None);bpy.context.collection.objects.link(root)
meshes=[o for o in bpy.data.objects if o.type=='MESH']
for ob in [rig,*meshes]:
 world=ob.matrix_world.copy();ob.parent=root;ob.matrix_world=world
for ob in meshes:
 for mod in ob.modifiers:
  if mod.type=='ARMATURE':mod.use_deform_preserve_volume=True
# Keep the original shoe sole and low upper rigidly on the foot, while
# restoring original calf/foot skinning above the ankle. A wholly rigid high
# cuff protruded behind the sloping calf; the sole must still stay planted.
# This changes only a v2 in-memory copy of the shoe weights.
shoe=bpy.data.objects['KitchenAdult.shoes01']
original_group_names={group.index:group.name for group in shoe.vertex_groups}
original_shoe_weights={vertex.index:{original_group_names[g.group]:g.weight for g in vertex.groups} for vertex in shoe.data.vertices}
for group in list(shoe.vertex_groups):shoe.vertex_groups.remove(group)
shoe_groups={name:shoe.vertex_groups.new(name=name) for name in sorted(set(original_group_names.values())|{'foot_l','foot_r'})}
shoe_binding_counts={'rigid_sole_low_upper':0,'transition':0,'original_high_cuff':0}
for vertex in shoe.data.vertices:
 point=shoe.matrix_world@vertex.co
 side='r' if point.dot(RIGHT)>0 else 'l'
 amount=max(0.,min(1.,(point.z-.12)/.05));amount=amount*amount*(3-2*amount)
 key='rigid_sole_low_upper' if amount==0 else ('original_high_cuff' if amount==1 else 'transition')
 shoe_binding_counts[key]+=1
 weights={name:value*amount for name,value in original_shoe_weights[vertex.index].items()}
 weights['foot_'+side]=weights.get('foot_'+side,0.)+(1-amount)
 total=sum(weights.values())
 for name,value in weights.items():
  if value>1e-8:shoe_groups[name].add([vertex.index],value/total,'REPLACE')
for image in bpy.data.images:
 candidate=ROOT/'assets/people/mpfb_corridor/textures'/Path(bpy.path.abspath(image.filepath)).name
 if candidate.is_file():image.filepath=str(candidate)

def smooth(u):
 u=max(0.,min(1.,u));return u*u*(3-2*u)
def interp(a,b,u):return a+(b-a)*smooth(u)
def world_head(name):return RIG@rig.pose.bones[name].head

def put(name,mat):
 rig.pose.bones[name].matrix=INV@mat;bpy.context.view_layer.update()
def shifted_rest(name,pos):
 m=rest[name].copy();m.translation=pos;return m

def tri_frame(direction,normal):
 y=direction.normalized();x=(normal-y*normal.dot(y)).normalized();z=x.cross(y).normalized()
 return Matrix((x,y,z)).transposed()

ik_metrics=[]
def two_bone(upper_name,lower_name,end_name,target,pole):
 # Solve fixed segment lengths and an anatomical bend plane, then map both
 # original bone frames into that plane. This preserves roll coherently.
 upper=rig.pose.bones[upper_name];lower=rig.pose.bones[lower_name]
 shoulder=world_head(upper_name);a=rest_heads[lower_name]-rest_heads[upper_name];b=rest_heads[end_name]-rest_heads[lower_name]
 l1=a.length;l2=b.length;axis=target-shoulder;distance=axis.length
 assert abs(l1-l2)+1e-5<distance<l1+l2-1e-5,(upper_name,'unreachable',distance,l1+l2)
 axis.normalize();along=(l1*l1-l2*l2+distance*distance)/(2*distance)
 bend=pole-axis*pole.dot(axis);assert bend.length>1e-4;bend.normalize()
 elbow=shoulder+axis*along+bend*math.sqrt(max(0,l1*l1-along*along))
 first=elbow-shoulder;second=target-elbow;normal=first.cross(second).normalized();old_normal=a.cross(b).normalized()
 for name,old_vec,new_vec,head in ((upper_name,a,first,shoulder),(lower_name,b,second,elbow)):
  q=tri_frame(new_vec,normal)@tri_frame(old_vec,old_normal).transposed();m=q.to_4x4()@rest[name];m.translation=head;put(name,m)
 # End bone follows the lower segment's original relative frame. There is
 # intentionally no restoration of an absolute hand rotation.
 end=rig.pose.bones[end_name];end.matrix_basis=Matrix.Identity(4);bpy.context.view_layer.update()
 actual=(world_head(end_name)-target).length
 ik_metrics.append({'chain':upper_name,'target_error_m':actual,'segment_error_m':max(abs((world_head(lower_name)-shoulder).length-l1),abs((world_head(end_name)-world_head(lower_name)).length-l2))})
 return first,second

STEP_WINDOWS=[(.15,.80,'l',.425),(.85,1.50,'r',.425),(1.55,2.20,'l',.85),(2.25,2.90,'r',.85)]
def stride(t,backward=False):
 start=0. if backward else -.85;direction=-1. if backward else 1.
 u=max(0.,min(1.,t/3.));ramp=.08
 if u<ramp:progress=u*u/(2*ramp*(1-ramp))
 elif u>1-ramp:progress=1-(1-u)**2/(2*ramp*(1-ramp))
 else:progress=(u-ramp/2)/(1-ramp)
 travel=start+direction*.85*progress
 feet={s:{'distance':start,'lift':0.,'support':True,'swing_u':0.} for s in ('l','r')}
 for begin,end,side,distance in STEP_WINDOWS:
  if t<begin:continue
  old=feet[side]['distance'];goal=start+direction*distance
  if t>=end:feet[side]['distance']=goal;continue
  u=(t-begin)/(end-begin);feet[side]={'distance':interp(old,goal,u),'lift':.055*math.sin(math.pi*u)**2,'support':False,'swing_u':u}
 return travel,feet

def locomotion(t):
 if t<3:return stride(t)
 if t<9:return 0.,{s:{'distance':0.,'lift':0.,'support':True,'swing_u':0.} for s in ('l','r')}
 if t<12:return stride(t-9,True)
 return -.85,{s:{'distance':-.85,'lift':0.,'support':True,'swing_u':0.} for s in ('l','r')}
def reach_amount(t):
 if t<4:return 0.
 if t<5.5:return smooth((t-4)/1.5)
 if t<7.5:return 1.
 if t<9:return 1-smooth((t-7.5)/1.5)
 return 0.

samples=[]
for frame in range(FRAMES):
 t=frame/FPS;scene.frame_set(frame)
 # The animation is always recomputed from original rest bones, never from
 # previous-frame points or endpoint mesh interpolation.
 root.location=(0,0,0)
 for bone in rig.pose.bones:bone.matrix_basis=Matrix.Identity(4)
 bpy.context.view_layer.update()
 travel,feet=locomotion(t);reach=reach_amount(t)
 moving=(0<t<3 or 9<t<12);walktime=t if t<3 else t-9
 # Slightly flex knees: original legs are almost fully extended in rest.
 walk_envelope=max(0.,min(1.,walktime/.3,(3-walktime)/.3)) if moving else 0.
 bob=-.018-walk_envelope*(.028+.006*math.sin(4*math.pi*walktime/3.)**2)
 pelvis=rest['Root'].copy();pelvis.translation+=UP*bob;put('Root',pelvis)
 # A small forward inclination accompanies reaching without displacing feet.
 spine=RIG@rig.pose.bones['spine_01'].matrix.copy();pivot=spine.translation.copy()
 tilt=Matrix.Rotation(math.radians(8.)*reach,4,UP.cross(F).normalized())
 put('spine_01',Matrix.Translation(pivot)@tilt@Matrix.Translation(-pivot)@spine)
 foot_rows={}
 for side in ('l','r'):
  state=feet[side];ankle=rest_heads['foot_'+side]+PATH*(state['distance']-travel)+UP*state['lift']
  two_bone('thigh_'+side,'calf_'+side,'foot_'+side,ankle,F)
  # A planted foot keeps the exact original flat orientation and position.
  foot=shifted_rest('foot_'+side,ankle)
  if not state['support']:
   pitch=math.radians(4.)*math.sin(math.pi*state['swing_u'])**2
   rot=Matrix.Rotation(pitch,4,RIGHT);foot=rot@foot;foot.translation=ankle
  put('foot_'+side,foot)
  ankle_world=world_head('foot_'+side)+PATH*travel
  ball_world=world_head('ball_'+side)+PATH*travel
  foot_rows[side]={'support':state['support'],'ankle_asset_world_m':list(ankle_world),'ball_asset_world_m':list(ball_world),'lift_m':state['lift']}
 for side,sign in (('r',1.),('l',-1.)):
  shoulder=world_head('upperarm_'+side)
  swing=0.
  if moving:
   localtime=t if t<3 else t-9
   envelope=math.sin(math.pi*min(1,localtime/3.))
   swing=sign*.045*math.sin(4*math.pi*localtime/3.)*envelope
  neutral=shoulder+F*(.035+swing)+RIGHT*(sign*.03)-UP*.485
  weight=reach if side=='r' else 0.
  forward=shoulder+F*.43+RIGHT*.01-UP*.16
  target=neutral.lerp(forward,weight)
  # Elbow stays behind the wrist when the arm hangs, and below the
  # forearm when raised. The negative-forward pole keeps the same
  # anatomical bend half-plane throughout, avoiding the singular
  # forward-to-down branch flip at a half-raised wrist.
  pole=(-F*(1-weight)-UP*weight+RIGHT*(sign*.04)).normalized()
  first,second=two_bone('upperarm_'+side,'lowerarm_'+side,'hand_'+side,target,pole)
  # Set the hand's long axis to follow its forearm; retain anatomical roll
  # through the lower-arm frame and original palm. No elbow-up pole.
  hand=rig.pose.bones['hand_'+side];current=RIG@hand.matrix.copy();h=current.translation
  hand_direction=(RIG@hand.tail)-h;q=hand_direction.rotation_difference(second)
  current=Matrix.Translation(h)@q.to_matrix().to_4x4()@Matrix.Translation(-h)@current;put('hand_'+side,current)
 root.location=PATH*travel
 root.keyframe_insert(data_path='location',frame=frame)
 # Bake exact bone states. The USD exporter evaluates original Armature
 # skinning at every frame; it does not interpolate between baked mesh poses.
 for bone in rig.pose.bones:
  bone.keyframe_insert(data_path='location',frame=frame)
  bone.keyframe_insert(data_path='rotation_quaternion',frame=frame)
  bone.keyframe_insert(data_path='scale',frame=frame)
 bpy.context.view_layer.update()
 points={n:list(world_head(n)+PATH*travel) for n in ('upperarm_r','lowerarm_r','hand_r','head')}
 samples.append({'frame':frame,'time_seconds':t,'root_translation_asset_m':list(PATH*travel),'reach':reach,'feet':foot_rows,'joints_asset_m':points,'bone_head_tail_asset_m':{b.name:[list(RIG@b.head+PATH*travel),list(RIG@b.tail+PATH*travel)] for b in rig.pose.bones}})
 if frame%24==0:print('ANIMATE_FRAME',frame,'reach',reach,'travel',travel,flush=True)
for ob in (rig,root):
 if ob.animation_data and ob.animation_data.action:
  for curve in ob.animation_data.action.fcurves:
   for key in curve.keyframe_points:key.interpolation='LINEAR'
bpy.ops.object.select_all(action='DESELECT')
for ob in [root,*meshes]:ob.select_set(True)
scene.frame_set(0)
bpy.ops.wm.usd_export(filepath=str(OUT/'person.usdc'),selected_objects_only=True,export_animation=True,export_materials=True,export_textures=True,overwrite_textures=True,relative_paths=True,root_prim_path='/Person')
report={'scope':'Scripted original-armature character animation; no motion capture or physical gait validation','source':str(source.relative_to(ROOT)),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'fps':FPS,'frames':FRAMES,'duration_seconds':FRAMES/FPS,'scene_position_m':[3.67,-.92,.0025],'scene_position_scope':'Integration hint only; the scene builder records the authoritative final world placement.','scene_yaw_degrees':125.,'asset_forward':list(F),'asset_right':list(RIGHT),'asset_path':list(PATH),'walk_displacement_m':float((PATH*.85).length),'route_change':'Start and return points offset 0.18 m toward world -Y to clear the stove; final stance and arm motion unchanged. Root and each planted footprint use the same diagonal path.','bone_solution':'Every frame reset original rest transforms, analytical two-bone fixed-length IK, coherent original bone frames, original preserve-volume skinning. Hands follow lower-arm direction.','footwear_binding':'Original shoe topology and textures retained. Only v2 in-memory weights changed: vertices at asset-rest z<=0.12m remain 100 percent corresponding foot; 0.12..0.17m smoothly blend back to original calf/foot weights; z>=0.17m retains original weights. This keeps the sole planted while the high cuff follows the calf. Other human and clothing weights, all bone animation and root route unchanged.','shoe_binding_vertex_counts':shoe_binding_counts,'mesh_endpoint_interpolation':False,'root_motion_baked':True,'motion_root':'/Person/MotionRoot','phases':[{'start':0,'end':3,'name':'walk_in'},{'start':3,'end':4,'name':'settle'},{'start':4,'end':5.5,'name':'raise_hand'},{'start':5.5,'end':7.5,'name':'hold'},{'start':7.5,'end':9,'name':'lower_hand'},{'start':9,'end':12,'name':'walk_back'},{'start':12,'end':14,'name':'settle'}],'max_ik_target_error_m':max(x['target_error_m'] for x in ik_metrics),'max_segment_length_error_m':max(x['segment_error_m'] for x in ik_metrics),'samples':samples}
(OUT/'animation.json').write_text(json.dumps(report,indent=2)+'\n')
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'animated_source.blend'))
print('ANIMATION_COMPLETE',OUT/'person.usdc',report['max_ik_target_error_m'],report['max_segment_length_error_m'],flush=True)
