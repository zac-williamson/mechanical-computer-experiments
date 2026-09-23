"""Reconstruct displaced input bearing bosses around actual shaft axes.
Preserve rear mounting interfaces and upper actuator/clutch supports.
"""
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development'
meta=json.loads((O/'printed-parts.json').read_text());report=[]
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,y,z):return m.Manifold.cylinder(b-a,r,circular_segments=128).rotate([0,90,0]).translate([a,y,z])
for p in meta:
 n=p['id'];memory=n.startswith('Memory');write=n.startswith('Write')
 if not ((memory and any(n.endswith(k) for k in ['Left side frame','Right side frame','Left inner bearing wall','Right inner bearing wall'])) or (write and any(n.endswith(k) for k in ['Right side frame','Right inner bearing wall']))):continue
 inner='inner' in n;left='Left' in n;x0,x1=(-11.8,-4.2) if inner and left else (4.2,11.8) if inner else (-31.8,-24.2) if left else (24.2,31.8)
 if write:x0-=85;x1-=85
 z= -16 if memory else 0;axes=[(10.2,z)]
 if memory and left:axes.append((10.2+np.sqrt(33.75),-10.5))
 t=trimesh.load(O.parent/'register-before-left-cam'/(n+'.stl'));
 if write:t.apply_translation([-181.8,0,0])
 s=m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
 if write and not inner:s-=box([-65,23.5,39.4],[-40,33,65])
 # Remove the whole old boss region, not just an axle clearance envelope.
 s-=box([x0-.01,-10,z-20],[x1+.01,23.2,z+11.1])
 for y,zz in axes:
  s+=cy(4.0 if inner and len(axes)>1 and zz==-10.5 else 5.5,x0,x1,y,zz)+box([x0,y,zz-3.8],[x1,28.4,zz+3.8])
 for y,zz in axes:
  s-=cy(2.65,x0-.1,x1+.1,y,zz)
  if inner and zz!=-10.5:s-=cy(3.9,x0-.1 if left else x1-4,x0+4 if left else x1+.1,y,zz)
 for pinz in ([z-2.4,z+6.8] if inner else [z-2.4,z+26]):
  s-=m.Manifold.cylinder(12,2.45,circular_segments=96).rotate([-90,0,0]).translate([(x0+x1)/2,22.8,pinz])
 a=s.to_mesh64();t=trimesh.Trimesh(a.vert_properties[:,:3],a.tri_verts,process=True)
 assert t.is_watertight and len(t.split())==1,n
 # Confirm material surrounds the shaft for 360 degrees in the working bearing land.
 station=(x1-1.8 if left else x0+1.8) if inner else (x0+x1)/2
 theta=np.linspace(0,2*np.pi,720,endpoint=False)
 for y,zz in axes:
  ring=np.c_[np.full(len(theta),station),y+2.9*np.cos(theta),zz+2.9*np.sin(theta)]
  assert t.contains(ring).all(),('Incomplete bearing circumference',n,y,zz)
  bore=np.c_[np.full(len(theta),station),y+2.5*np.cos(theta),zz+2.5*np.sin(theta)]
  assert not t.contains(bore).any(),('Obstructed bore',n)
  report.append(dict(part=n,axis_YZ_mm=[y,zz],bore_diameter_mm=5.3,boss_diameter_mm=8 if inner and zz==-10.5 else 11,bearing_land_length_mm=3.6 if inner and zz!=-10.5 else 7.6,circumference_samples=720,complete_circumference=True))
 t.export(O/(n+'.stl'),file_type='stl_ascii');p.update(bounds=t.bounds.tolist(),watertight=True,solids=1)
(O/'printed-parts.json').write_text(json.dumps(meta,indent=2));(O/'Bearing support checks.json').write_text(json.dumps(dict(bearings=report,scope='Nominal bore alignment and complete supporting circumference. Physical load qualification and updated whole-assembly clearance remain separate.'),indent=2));print(json.dumps(report,indent=2))

# The idler gear is retained between its outer LEGO bush and the full-length inner bearing.
# Eliminate the redundant buried bush and its counterbore; keep a complete 8 mm OD boss.
hs=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];arrays=[];kept=[]
for h in hs:
 if h['id']=='Memory — A-idler-bush-10':continue
 a=v[h['offset']//3:h['offset']//3+h['vertices']].copy();h['offset']=sum(x.size for x in arrays);arrays.append(a);kept.append(h)
(O/'hardware.json').write_text(json.dumps(kept,indent=2));np.savez_compressed(O/'hardware.npz',vertices=np.concatenate(arrays))
