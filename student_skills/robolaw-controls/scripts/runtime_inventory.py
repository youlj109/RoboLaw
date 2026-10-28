#!/usr/bin/env python3
"""Read only installed distribution metadata; no CUDA/Kit imports or file writes."""
import importlib.metadata
import json
import platform
import sys

names=('isaaclab-arena','isaaclab','isaacsim','torch','warp-lang','numpy','python-fcl','nvidia-curobo','h5py','scipy','Pillow')
packages={}
for name in names:
    try: packages[name]=importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError: packages[name]=None
print(json.dumps({'python_executable':sys.executable,'python_version':platform.python_version(),'packages':packages,'missing_distributions':[k for k,v in packages.items() if v is None],'scope':'distribution metadata only; package importability, native libraries, GPU and simulation are not tested','gpu_started':False,'runtime_verified':False},ensure_ascii=False,indent=2))
