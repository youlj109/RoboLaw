"""Decode complete recorded videos and extract phase-labelled frames for human review."""
import argparse, hashlib, json
from pathlib import Path
import imageio_ffmpeg
from PIL import Image, ImageDraw


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(4*1024*1024),b''):h.update(block)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('episodes',type=Path,nargs='+')
    a=p.parse_args()
    if a.output.exists():p.error('Refusing to overwrite video review evidence')
    for ep in a.episodes:
        if not (ep/'trajectory.json').exists():p.error(str(ep)+' has no completed trajectory')
    a.output.mkdir(parents=True)
    report={'scope':'Complete video decoding plus phase-labelled keyframes; visual judgement must be recorded separately','episodes':{}}
    good=True
    for ep in a.episodes:
        trajectory=json.loads((ep/'trajectory.json').read_text())
        result=json.loads((ep/'result.json').read_text())
        phases={}
        for i,entry in enumerate(trajectory):phases.setdefault(entry['phase'],[]).append(i)
        labels={0:'initial',len(trajectory)-1:'terminal'}
        for phase in ['pregrasp','approach_handle','grasp_handle','pull_door','success_hold','released_hold']:
            ids=phases.get(phase,[])
            if ids:labels[ids[len(ids)//2]]=phase
        receipt=ep/'comparison_audit.json'
        if receipt.exists():
            measurements=json.loads(receipt.read_text())['measurements']
            for side in [0,1]:
                minimum=min((row for row in measurements if row['side']==side),key=lambda row:row['distance_m'])
                if minimum['frame']<len(trajectory):labels[minimum['frame']]='counterfactual_minimum' if side==0 else 'actual_minimum'
        frames={}; videos={}
        for filename in ['external_camera.mp4','external_camera_2.mp4','wrist_camera.mp4','all_cameras.mp4','baseline_comparison.mp4']:
            path=ep/filename
            if not path.exists():
                if filename!='baseline_comparison.mp4':good=False
                continue
            generator=imageio_ffmpeg.read_frames(str(path),pix_fmt='rgb24',input_params=['-xerror','-err_detect','explode'])
            meta=next(generator);size=tuple(meta['size']);fps=meta['fps'];count=0
            for raw in generator:
                if filename in ['all_cameras.mp4','baseline_comparison.mp4'] and count in labels:
                    frames[(filename,count)]=Image.frombytes('RGB',size,raw)
                count+=1
            expected=json.loads((ep/'comparison_audit.json').read_text())['frames'] if filename=='baseline_comparison.mp4' else result['frames']
            passed=count==expected and count>0 and abs(fps-15)<.011
            good=good and passed
            videos[filename]={'sha256':sha(path),'decoder':'imageio_ffmpeg strict decode','decoded_frames':count,'expected_frames':expected,'fps':fps,'shape_hw':[size[1],size[0]],'passed':passed}
        sheets=[]
        for filename in ['all_cameras.mp4','baseline_comparison.mp4']:
            keys=[i for i in sorted(labels) if (filename,i) in frames]
            if not keys:continue
            width=1440;sample=frames[(filename,keys[0])];height=round(sample.height*width/sample.width);bar=28
            sheet=Image.new('RGB',(width,(height+bar)*len(keys)),(24,24,24));draw=ImageDraw.Draw(sheet)
            for row,i in enumerate(keys):
                y=row*(height+bar);draw.text((12,y+6),f'{ep.name} | {filename} | frame {i} | {labels[i]} | door={trajectory[i]["obs"]["door_q"]:.4f} rad',fill='white')
                sheet.paste(frames[(filename,i)].resize((width,height)),(0,y+bar))
            dest=a.output/(ep.name+'_'+Path(filename).stem+'.jpg');sheet.save(dest,quality=92);sheets.append(str(dest))
        report['episodes'][str(ep)]={'videos':videos,'keyframes':labels,'contact_sheets':sheets}
    report['complete_decode_passed']=good
    (a.output/'video_decode.json').write_text(json.dumps(report,indent=2)+'\n')
    print('VIDEO_DECODE',good,'episodes',len(a.episodes),flush=True)
    return 0 if good else 1


if __name__=='__main__':raise SystemExit(main())
