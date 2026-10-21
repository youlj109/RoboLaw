"""Fetch selected CC0 assets using ZIP byte ranges instead of a 267 MB pack."""
import concurrent.futures
import io
import struct
import zipfile
import zlib
from pathlib import Path
import requests

ROOT=Path(__file__).resolve().parents[1]
URL='https://files.makehumancommunity.org/asset_packs/makehuman_system_assets/makehuman_system_assets_cc0.zip'
OUT=ROOT/'assets/people/mpfb_library'

def get_range(start,end):
    for attempt in range(3):
        try:
            r=requests.get(URL,headers={'Range':f'bytes={start}-{end}'},timeout=90)
            r.raise_for_status()
            if r.status_code!=206 or not r.headers['Content-Range'].startswith(f'bytes {start}-'):
                raise RuntimeError('Server did not return requested range')
            return r.content
        except Exception:
            if attempt==2:raise

tail=requests.get(URL,headers={'Range':'bytes=-65536'},timeout=90)
tail.raise_for_status()
offset=int(tail.headers['Content-Range'].split()[1].split('-')[0])
archive=zipfile.ZipFile(io.BytesIO(tail.content))
prefixes=('skins/young_asian_male/','clothes/male_casualsuit01/',
          'clothes/shoes01/','hair/short02/','eyes/high-poly/','eyebrows/eyebrow001/')
items=[i for i in archive.infolist() if not i.is_dir() and
       (i.filename.startswith(prefixes) or i.filename in ('eyes/materials/brown.mhmat','eyes/materials/brown_eye.png'))]

def fetch(item):
    p=OUT/item.filename
    if p.exists() and p.stat().st_size==item.file_size:return
    start=item.header_offset+offset
    data=get_range(start,start+item.compress_size+1024)
    hdr=struct.unpack('<4s5H3L2H',data[:30])
    assert hdr[0]==b'PK\x03\x04'
    begin=30+hdr[-2]+hdr[-1]
    packed=data[begin:begin+item.compress_size]
    raw=zlib.decompress(packed,-15) if item.compress_type==8 else packed
    assert len(raw)==item.file_size
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_bytes(raw)
    print('ASSET',item.filename,flush=True)

with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
    list(pool.map(fetch,items))
print('MPFB_SUBSET_COMPLETE',len(items),flush=True)
