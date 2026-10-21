exec(open('/workspace/scripts/probe_planner.py').read().split("for xyz in [[2.49")[0])
q=kin.ik(pose([2.3,-.31,1.13],35,0),np.deg2rad([0,-36,0,-144,0,108,0]))
count=0
for tilt in [25,35,45,55,60]:
 for yaw in [-90,-75,-60,-45,-30,0,30,45,60,75,90,120,150,180]:
  try:
   g=kin.ik(pose([2.49,-.31,1.085],tilt,yaw),q,attempts=3)
   if world.valid(g,('Mustard','Bowl','Stand')):
    print('GOAL_OK',tilt,yaw,'edge',world.edge(q,g,('Mustard','Bowl','Stand')),flush=True);count+=1
  except RuntimeError:pass
print('TOTAL',count,flush=True)
g=kin.ik(pose([2.49,-.31,1.085],25,0),q,attempts=3)
try:
 path=world.plan(q,g,('Mustard','Bowl','Stand'),seed=42,max_iterations=5000)
 print('PRECISE_RRT_OK',len(path),flush=True)
 np.savez('/workspace/reports/human_precise_path.npz',path=path)
except Exception as ex:print('PRECISE_RRT_FAIL',str(ex),flush=True)
