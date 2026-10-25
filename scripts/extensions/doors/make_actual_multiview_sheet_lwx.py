"""Sample native and observer actual synchronized frames without changing video."""
import argparse,json,imageio_ffmpeg
from pathlib import Path
from PIL import Image,ImageDraw
p=argparse.ArgumentParser();p.add_argument("episode",type=Path);p.add_argument("output",type=Path);a=p.parse_args();assert not a.output.exists()
rows=json.loads((a.episode/"trajectory.json").read_text());pre=[i for i,r in enumerate(rows) if r["phase"]=="pregrasp"];indices=[pre[int(f*(len(pre)-1))] for f in [.1,.55,.95]]
sources=["external_camera.mp4","external_camera_2.mp4","supplementary_observer/visible_review_camera.mp4"];canvas=Image.new("RGB",(1920,1080));found=[]
for j,source in enumerate(sources):
 rd=imageio_ffmpeg.read_frames(str(a.episode/source),pix_fmt="rgb24");mt=next(rd);seen=[]
 for i,raw in enumerate(rd):
  if i in indices:
   im=Image.frombytes("RGB",mt["size"],raw).resize((640,360));d=ImageDraw.Draw(im);d.rectangle((0,0,640,25),fill="black");d.text((8,6),source+" | "+str(i),fill="white");canvas.paste(im,(indices.index(i)*640,j*360));seen.append(i)
  if i>=max(indices):break
 rd.close()
 if seen!=indices:raise RuntimeError("Unavailable actual frames")
 found.append({"source":source,"indices":seen})
canvas.save(a.output,quality=95);a.output.with_suffix(".json").write_text(json.dumps({"episode":str(a.episode),"scope":"Sampled actual synchronized frames only, not complete continuous visual audit","sources":found},indent=2))
