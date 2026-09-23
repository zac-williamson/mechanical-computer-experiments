"""Two-core engineering layout. Preserves worm/reaction/lever/guide geometry.
Storage transmission uses one powered axle, one direct 16T mesh and one idler path.
Write mux has an EMPTY positive side. No latch/cam integration is asserted here.
"""
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m,math
R=Path(__file__).resolve().parents[1];B=R.parent/'work/register-mux-reference/multiplexer';O=R/'Core layout';O.mkdir(exist_ok=True)
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def mesh(s):
 z=s.to_mesh64();return trimesh.Trimesh(np.asarray(z.vert_properties)[:,:3],np.asarray(z.tri_verts),process=False)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cyl(r,a,b,axis=0,c=(0,0,0)):
 s=m.Manifold.cylinder(b-a,r,circular_segments=96)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 t=list(c);t[axis]=a;return s.translate(t)
meta=json.loads((B/'printed-parts.json').read_text());hm=json.loads((B/'hardware.json').read_text());v=np.load(B/'hardware.npz')['vertices'];P={p['id']:solid(trimesh.load(B/(p['id']+'.stl'))) for p in meta};H={p['id']:v[p['offset']//3:p['offset']//3+p['vertices']].copy() for p in hm};shift=np.array([96.8,0,16]);rot=np.diag([-1,1,-1]);parts=[];hw=[];V=[]
old_idler_y=10.2+np.sqrt(144-9.2**2);new_idler_y=10.2+np.sqrt(144-8**2)
for bank in ['Memory','Write']:
 for p in meta:
  n=p['id'];s=P[n]
  if bank=='Memory' and n in ['Left side frame','Right side frame','Left inner bearing wall','Right inner bearing wall']:
   left=n.startswith('Left');a,b=((-31.8,-24.2) if 'side' in n else (-11.8,-4.2)) if left else ((24.2,31.8) if 'side' in n else (4.2,11.8))
   for yy,zz in [(10.2,-18.4),(old_idler_y,-9.2)]:s+=cyl(2.8,a,b,0,(0,yy,zz))
   s-=cyl(2.6,a-.1,b+.1,0,(0,10.2,-16))
   if not left:s-=cyl(2.6,a-.1,b+.1,0,(0,new_idler_y,-8))
   if 'inner' in n:
    ca,cb=(-11.9,-7.8) if left else (7.8,11.9)
    s-=cyl(3.9,ca,cb,0,(0,10.2,-16))
    if not left:s-=cyl(3.9,ca,cb,0,(0,new_idler_y,-8))
  if bank=='Write':
   if n=='Right inner bearing wall':continue
   s=s.rotate([0,180,0]).translate(shift)
  out=f'{bank} — {n}';t=mesh(s);assert t.is_watertight and len(t.split())==1,out;t.export(O/(out+'.stl'));parts.append(dict(id=out,bank=bank,source=n,motion=p['motion'],bounds=t.bounds.tolist(),watertight=True,print_rotation_axis=p['print_rotation_axis'],print_rotation_angle=p['print_rotation_angle']))
 for p in hm:
  n=p['id'];a=H[n].copy();d=dict(p)
  if bank=='Memory':
   if n in ['A-shaft','B-shaft'] or n.startswith('B-idler'):continue
   if n.startswith(('A-input','B-input')):a[:,2]+=2.4
   if n.startswith('A-idler'):a[:,1]+=new_idler_y-old_idler_y;a[:,2]+=1.2
   if n=='C-shaft':
    # 12L LEGO axle (3708), translated +4.8mm, in place of original centred 11L.
    a[:,0]*=96/88;a[:,0]+=4.8;d['lego_part']='3708';d['geometry_note']='Length-adjusted standard axle mesh; cross section unchanged; end detail approximate.'
  else:
   if n=='L072' or n.startswith('A-') or n.startswith('Baseboard pin X8 '):continue
   a=a@rot.T+shift
  d.update(id=f'{bank} — {n}',source=n,bank=bank,offset=sum(x.size for x in V),vertices=len(a));hw.append(d);V.append(a)
# One common 11L power axle, sharing the original four input bearings.
a=H['O-shaft'].copy();a[:,2]-=16
hw.append(dict(id='Memory — common 1 power axle',source='common-power',bank='Memory',motion='power',lego_part='23948',offset=sum(x.size for x in V),vertices=len(a)));V.append(a)
# Conservative envelope, deliberately not passed off as native LEGO geometry.
c=cyl(4.1,44.8,60.8,0,(0,10.2,16));a=mesh(c).triangles.reshape(-1,3)
hw.append(dict(id='2L LEGO axle joiner — envelope only',source='coupler-envelope',bank='link',motion='data-gated',lego_part='6538c',geometry_note='Conservative filled cylinder, radius 4.1mm, length 16mm. Exact part/axle insertion fit must be checked.',offset=sum(x.size for x in V),vertices=len(a)));V.append(a)
(O/'printed-parts.json').write_text(json.dumps(parts,indent=2));(O/'hardware.json').write_text(json.dumps(hw,indent=2));np.savez_compressed(O/'hardware.npz',vertices=np.concatenate(V))
# Native printed cores clearance, independently sampled full carriage ranges.
# Constant q-independent swept union for each bank, including bounded lever sweep.
cores={};names={}
for bank in ['Memory','Write']:
 s=m.Manifold();names[bank]=[]
 for p in parts:
  if p['bank']!=bank:continue
  t=solid(trimesh.load(O/(p['id']+'.stl')))
  if p['motion']=='carriage':
   # Extrude convex swept hull is conservative for translation but preserves each part's extent.
   a=mesh(t);b=a.copy();b.apply_translation([9.2,0,0]);a.apply_translation([-4.6,0,0]);b.apply_translation([-4.6,0,0]);t=solid(trimesh.util.concatenate([a,b]).convex_hull)
  if p['motion']=='rocker':
   pv=np.array(json.loads((B/'parameters.json').read_text())['pivot']);pv=pv if bank=='Memory' else pv@rot.T+shift
   # Conservative convex envelope of quarter-degree samples, padded later by separation check.
   verts=[];a=mesh(t)
   for angle in np.arange(-29,29.01,.25):verts.append(trimesh.transform_points(a.vertices,trimesh.transformations.rotation_matrix(np.radians(angle),[0,1,0],pv)))
   t=solid(trimesh.convex.convex_hull(np.concatenate(verts)))
  names[bank].append((p['id'],t))
 collisions=[]
for na,a in names['Memory']:
 for nb,b in names['Write']:
  vol=(a^b).volume()
  if vol>1e-4:collisions.append(dict(a=na,b=nb,volume_mm3=float(vol)))
coupler_hits=[]
for bank in names:
 for n,s in names[bank]:
  vol=(s^c).volume()
  if vol>1e-4:coupler_hits.append(dict(part=n,volume_mm3=float(vol)))
allbounds=[np.array(p['bounds']) for p in parts]+[np.array([a.min(0),a.max(0)]) for a in V];lo=np.min(allbounds,axis=0)[0];hi=np.max(allbounds,axis=0)[1]
report=dict(status='TWO CORES AND DRIVE ISOLATION LAYOUT — latch/cam/mount integration incomplete',offset_write=shift.tolist(),orientation_write='180 degrees about Y',nominal_envelope_mm=(hi-lo).tolist(),inter_core_swept_print_collisions=collisions,coupler_to_print_envelope_collisions=coupler_hits,power_train=dict(power_axis=[0,10.2,-16],direct_16T_centres_mm=16,positive_idler_axis=[16,new_idler_y,-8],positive_idler_centres_mm=[12,12],positive_path='16T -> 8T -> 16T, direction retained, ratio +1',negative_path='16T -> 16T, direction reversed, ratio -1'),removed_write_parts='Positive clutch gear and all positive input routing; right inner bearing wall and its two mount pins. Existing clutch-ring connector and end support remain.',limitations=['Gear phases not yet tuned or qualified.','Inter-core hardware-pair checks and modified bearing checks pending.','No structural joining bridge, memory lock, cam, or lost-motion coupling installed.','No claim of a functional complete register or minimum volume.'])
(O/'Checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
