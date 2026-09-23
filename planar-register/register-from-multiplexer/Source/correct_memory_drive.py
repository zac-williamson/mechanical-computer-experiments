"""Run after adjacent_lock, before finish_adjacent_gears: remove contradictory mesh."""
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'Planar register'
h=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];a={p['id']:v[p['offset']//3:p['offset']//3+p['vertices']].copy() for p in h}
old=np.array([16,10.2+np.sqrt(80),-8]);new=np.array([16,10.2+np.sqrt(33.75),-10.5])
# Native 10928 8T gear, not a scaled 16T stand-in.
a['Memory — A-input']=a['Memory — A-idler'].copy()-old+[16,10.2,-16]
for p in h:
 if p['id'].startswith('Memory — A-idler'):a[p['id']]+=new-old
 if p['id']=='Memory — A-input':p['lego_part']='10928';p['geometry_note']='Native 8T; two meshes retain direction at half power-shaft speed.'
seq=[]
for p in h:p['offset']=sum(x.size for x in seq);p['vertices']=len(a[p['id']]);seq.append(a[p['id']])
np.savez_compressed(O/'hardware.npz',vertices=np.concatenate(seq));(O/'hardware.json').write_text(json.dumps(h,indent=2))
def cy(r,x0,x1,y,z):return m.Manifold.cylinder(x1-x0,r,circular_segments=80).rotate([0,90,0]).translate([x0,y,z])
meta=json.loads((O/'printed-parts.json').read_text())
for p in meta:
 if p['id'] not in ['Memory — Right side frame','Memory — Right inner bearing wall']:continue
 t=trimesh.load(O/(p['id']+'.stl'));s=m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
 x0,x1=(24.2,31.8) if 'side' in p['id'] else (4.2,11.8)
 s+=cy(2.8,x0,x1,old[1],old[2])
 if 'inner' in p['id']:s+=cy(4.05,7.8,11.8,old[1],old[2])
 s-=cy(2.6,x0-.1,x1+.1,new[1],new[2])
 if 'inner' in p['id']:s-=cy(3.9,7.8,11.9,new[1],new[2])
 z=s.to_mesh64();t=trimesh.Trimesh(np.asarray(z.vert_properties)[:,:3],np.asarray(z.tri_verts),process=False);assert t.is_watertight and len(t.split())==1
 t.export(O/(p['id']+'.stl'),file_type='stl_ascii');p['bounds']=t.bounds.tolist();p['watertight']=True;p['solids']=1
(O/'printed-parts.json').write_text(json.dumps(meta,indent=2))
print('Removed unintended positive direct mesh; relocated idler bearings',new.tolist())
