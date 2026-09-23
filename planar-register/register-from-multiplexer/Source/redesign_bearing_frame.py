"""Generate shaft-centred bearing walls and matching two-pin base mounts from one datum table."""
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development'
meta=json.loads((O/'printed-parts.json').read_text());hw=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];A={h['id']:v[h['offset']//3:h['offset']//3+h['vertices']].copy() for h in hw}
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,axis,c):
 s=m.Manifold.cylinder(b-a,r,circular_segments=96)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 q=list(c);q[axis]=a;return s.translate(q)
def so(t):return m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
def load(n):return so(trimesh.load(O/(n+'.stl')))
def beam(a,b,r=3.8):return (cy(r,32.8,40.4,1,[a[0],0,a[1]])+cy(r,32.8,40.4,1,[b[0],0,b[1]])).hull()
def export(n,s,p):
 q=s.to_mesh64();t=trimesh.Trimesh(q.vert_properties[:,:3],q.tri_verts,process=True);t.merge_vertices(digits_vertex=6);t.update_faces(t.nondegenerate_faces(height=1e-8));t.remove_unreferenced_vertices();assert t.is_watertight and len(t.split())==1,(n,[(c.volume,c.bounds.tolist()) for c in t.split()]);t.export(O/(n+'.stl'),file_type='stl_ascii');p.update(bounds=t.bounds.tolist(),watertight=True,solids=1);return t
oldbase=load('Unified rear backbone');base=oldbase^box([-160,0,40],[60,50,100])
# Preserve only the functional carriage guide rails, not the old base or hole layout.
for x,z in [(0,0),(-85,16)]:
 guide=oldbase^box([x-23.8,15.5,z+9.0],[x+23.8,32.8,z+12.2]);base+=guide
 base+=box([x-20.8,28.4,z+9.1],[x+20.8,40.4,z+12.1])+box([x-28,32.8,z+9.1],[x+28,40.4,z+12.1])
 for zz in [z-20,z+16]:base+=beam([x-28,zz],[x+28,zz])
 for xx in [x-28,x+28]:base+=beam([xx,z-20],[xx,z+16])
base+=beam([-57,12],[-28,-4])
# Reconnect the lock guide to the new rear frame using two continuous ribs.
for xx,tip in [(-28,-15.05),(28,4.95)]:base+=beam([xx,16],[tip,56])
wallnames={p['id'] for p in meta if p['id'].endswith(('side frame','inner bearing wall'))};meta=[p for p in meta if p['id'] not in wallnames]
plans=[];bearings=[];mounts=[]
for bank,x,z in [('Memory',0,0),('Write',-85,16)]:
 for side,station,inner in [('Left',-28,False),('Right',28,False),('Left',-8,True),('Right',8,True)]:
  if bank=='Write' and side=='Left' and inner:continue
  cx=x+station;lo,hi=cx-3.8,cx+3.8;n=f'{bank} — {side} '+('inner bearing wall' if inner else 'side frame')
  axes=[(10.2,z-16,'power' if bank=='Memory' else 'data')]
  if not inner:axes += [(10.2,z,'clutch'),(10.2,z+16,'worm input')]
  if bank=='Memory' and side=='Left':axes.append((10.2+np.sqrt(33.75),-10.5,'idler'))
  pins=[z-20,z-12] if inner else [z-16,z+16]
  s=box([lo,24.8,min(pins)-3.8],[hi,32.4,max(pins)+3.8])
  for y,zz,role in axes:
   radius=4 if inner and role=='idler' else 5.5
   s+=cy(radius,lo,hi,0,[0,y,zz])+box([lo,y,zz-3.2],[hi,28.6,zz+3.2])
  if not inner:
   s+=box([lo,7, z-16],[hi,13.4,z+16])
  if side=='Right' and not inner:
   # Preserve the actuator's established cheek mating interface only; all shaft
   # bearings, support ribs, feet and base mounts are generated above from datums.
   terminal=trimesh.load(O.parent/'register-before-left-cam'/'Memory — Right side frame.stl')
   terminal.apply_translation([x,0,z]);s+=so(terminal)^box([x+31.7,0,z+6],[x+50,24,z+40])
  for y,zz,role in axes:
   s-=cy(2.65,lo-.1,hi+.1,0,[0,y,zz]);counter=inner and role!='idler'
   if counter:s-=cy(3.9,lo-.1 if side=='Left' else hi-4,lo+4 if side=='Left' else hi+.1,0,[0,y,zz])
   bearings.append(dict(part=n,axis_YZ_mm=[y,zz],role=role,x_span_mm=[lo,hi],bearing_land_mm=3.6 if counter else 7.6,bore_radius_mm=2.65,boss_radius_mm=4 if inner and role=='idler' else 5.5,counterbore=counter,side=side))
  for i,zz in enumerate(pins):
   s-=cy(2.5,24.7,32.5,1,[cx,0,zz])+cy(3.35,31.7,32.5,1,[cx,0,zz]);base+=cy(4.3,32.8,40.4,1,[cx,0,zz]);mounts.append(dict(part=n,x=cx,z=zz,index=i))
  if inner:base+=beam([cx,min(pins)],[cx,max(pins)])
  p=dict(id=n,bank=bank,motion='fixed',source='shaft datum frame redesign',print_rotation_axis=[0,1,0],print_rotation_angle=np.pi/2)
  export(n,s,p);meta.append(p)
for pin in mounts:base-=cy(2.5,32.7,40.5,1,[pin['x'],0,pin['z']])+cy(3.35,32.7,33.5,1,[pin['x'],0,pin['z']])
p=next(p for p in meta if p['id']=='Unified rear backbone');export(p['id'],base,p)
# Replace every obsolete wall-mount pin with the same genuine native 2L friction pin.
template=next(h for h in hw if 'Baseboard pin' in h['id'] or h['id'].startswith('Frame pin '));pinmesh=A[template['id']].copy();pinmesh-=(pinmesh.min(0)+pinmesh.max(0))/2
hw=[h for h in hw if 'Baseboard pin' not in h['id'] and not h['id'].startswith('Frame pin ') and h['id']!='Memory — A-idler-bush-10']
for pin in mounts:
 n=f"Frame pin {pin['part']} {pin['index']}";A[n]=pinmesh+[pin['x'],32.6,pin['z']];hw.append(dict(id=n,bank='Frame',kind='native',motion='fixed',color=[54,133,180],lego_part=template.get('lego_part','2780'),vertices=len(pinmesh)))
arr=[]
for h in hw:h['offset']=sum(a.size for a in arr);arr.append(A[h['id']])
np.savez_compressed(O/'hardware.npz',vertices=np.concatenate(arr));(O/'hardware.json').write_text(json.dumps(hw,indent=2));(O/'printed-parts.json').write_text(json.dumps(meta,indent=2));(O/'Bearing frame datums.json').write_text(json.dumps(dict(bearings=bearings,mounts=mounts,frame_plane_Y_mm=[32.8,40.4],wall_mount_Y_mm=[24.8,32.4],pin_center_Y_mm=32.6),indent=2));print('Redesigned',len(set(b['part'] for b in bearings)),'walls',len(bearings),'bearings',len(mounts),'matching mounts')
