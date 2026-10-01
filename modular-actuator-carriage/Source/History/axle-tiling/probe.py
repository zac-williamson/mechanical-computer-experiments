from pathlib import Path
import sys,json,gzip,base64
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];A=R/'neck-candidate'
sys.path.insert(0,str(R.parents[1]/'latest-register-analysis/planar-register/register-from-multiplexer/Source'))
from ldraw_mesh import LDraw
def part(n):
 lib=LDraw('/Applications/Studio 2.0/ldraw/parts/'+n+'.dat');a=lib.mesh()[0].reshape(-1,3)*.4;assert not lib.missing
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);t.apply_translation(-t.bounds.mean(0));axis=int(np.argmax(t.extents));t.apply_transform(trimesh.geometry.align_vectors(np.eye(3)[axis],[1,0,0]));return t
for n in ['6538a','59443','3737','3705']:
 t=part(n);print(n,t.bounds.tolist(),t.is_watertight,flush=True)
 t.export(R/'axle-tiling'/(n+'.stl'),file_type='stl_ascii')
