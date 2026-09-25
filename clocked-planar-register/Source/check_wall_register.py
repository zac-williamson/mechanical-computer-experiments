"""Independent CAD screens; failures remain visible and block release."""
from pathlib import Path
import json,hashlib,sys,math
import numpy as np
import trimesh
import manifold3d as m
from wall_pose import vertices,example_frames,transform
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'Wall register'
P=json.loads((OUT/'parts.json').read_text());V=np.load(OUT/'geometry.npz')['vertices'].reshape(-1,3)
# A second row checks the repeated rod joints and neighbouring fixed structure.
import copy
P += [dict(copy.deepcopy(p),id=p['id']+' [row 2]',row_offset=112.) for p in P if p['module'] in ['bit','coupler']]
mode=sys.argv[1] if len(sys.argv)>1 else '9'
if mode in ['all','ends']:
 report=json.loads((ROOT/'Compact layout/Compact contact-resolved operation.json').read_text());frames=[];seen=set()
 for case in report['cases']:
  fs=case['frames']
  for ix in np.linspace(0,len(fs)-1,17 if mode=='all' else 2,dtype=int):
   f=fs[ix];key=tuple(round(float(z),6) for z in [f['rail'],f['master_lift'],f['slave_lift'],f.get('master_gate_lag',0),f.get('slave_gate_lag',0)]+[f[b][k] for b in ['master','slave','write','clock'] for k in ['q','b']])
   if key not in seen:seen.add(key);frames.append(f)
 sample=list(range(len(frames)))
else:
 frames=example_frames();sample=sorted(set(np.linspace(0,len(frames)-1,int(mode),dtype=int)))
invalid=[];printed=[]
for p in P:
 if p['kind']!='printed':continue
 a=V[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 if not t.is_watertight:invalid.append(p['id'])
 printed.append((p,a,m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))))
hits={};count=0;components=[];pair_cache={};unique_checks=0
for fi in sample:
 solids=[]
 for p,a,base in printed:
  v=vertices(p,a,frames[fi]);v[:,2]+=p.get('row_offset',0)
  if p.get('motion') in ['fork-band','actuator-band','elastic']:
   t=trimesh.Trimesh(v,np.arange(len(v)).reshape(-1,3),process=True)
   s=m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)));pose_key=None
  else:
   tf=transform(p,frames[fi]);tf[2,3]+=p.get('row_offset',0);s=base.transform(tf[:3,:]);pose_key=tuple(tf.ravel())
  solids.append((p,v.min(0),v.max(0),s,pose_key))
  if fi==sample[0]:components.append(dict(part=p['id'],connected_solids=len(s.decompose()),volume_mm3=s.volume()))
 for i,(p,lo,hi,s,pk) in enumerate(solids):
  for q,ll,hh,ss,qk in solids[i+1:]:
   if np.any(np.minimum(hi,hh)-np.maximum(lo,ll)<=1e-6):continue
   count+=1
   cache_key=(p['id'],q['id'],pk,qk) if pk is not None and qk is not None else None
   cached=pair_cache.get(cache_key) if cache_key is not None else None
   if cached is None:
    over=s^ss;vol=over.volume();unique_checks+=1
    bounds=None
    if vol>.01:
     vv=over.to_mesh64().vert_properties[:,:3];bounds=[vv.min(0).tolist(),vv.max(0).tolist()]
    if cache_key is not None:pair_cache[cache_key]=(vol,bounds)
   else:vol,bounds=cached
   if vol>.01:
    key=(p['id'],q['id'])
    if key not in hits:print('new intersection',key,vol,flush=True)
    if key not in hits or vol>hits[key]['volume_mm3']:
     hits[key]=dict(a=key[0],b=key[1],frame=int(fi),volume_mm3=vol,bounds=bounds)
 if fi%50==0:print('frame',fi,'colliding pairs',len(hits),flush=True)
# Exact displacement equivalence over the full specified stroke.
links=[]
for key,rout,rin,stroke in [('clock',20,20,9.375),('write',24,24,3.75)]:
 errors=[];rod_errors=[]
 for s in np.linspace(-stroke,stroke,1001):
  theta=-math.asin(s/rout)
  output_x=-rout*math.sin(theta);input_z=rin*math.sin(theta)
  errors.append(abs(output_x-s));rod_errors.append(abs(input_z+rin/rout*s))
 links.append(dict(control=key,output_stroke_mm=2*stroke,rod_stroke_mm=2*stroke*rin/rout,input_radius_mm=rin,output_radius_mm=rout,max_output_error_mm=max(errors),max_rod_error_mm=max(rod_errors),rod_opposes_output=True,input_slot_arc_excursion_mm=rin*(1-math.sqrt(1-(stroke/rout)**2))))

out=dict(geometry_sha256=hashlib.sha256((OUT/'geometry.npz').read_bytes()).hexdigest(),scope='Two adjacent rows plus controller: sampled printed collisions using inherited contact-resolved poses; repeated rows share the same data trace; no physical dynamics claim',frames=len(sample),case_coverage='112 inherited traces, 17 samples each, unique printed poses' if mode=='all' else 'one inherited rising capture',narrow_checks=count,unique_pair_checks=unique_checks,cache_policy="Exact part IDs and full rigid transforms; deforming parts are not cached",invalid_solids=invalid,parts=components,intersections=sorted(hits.values(),key=lambda x:-x['volume_mm3']),linkage_checks=links,printed_motion_pass=not hits and not invalid and all(p['connected_solids']==1 for p in components),mechanically_qualified=False)
(OUT/'Development checks.json').write_text(json.dumps(out,indent=2));print('collisions',len(hits),'invalid',invalid)
for h in out['intersections'][:30]:print(h['a'],'/',h['b'],round(h['volume_mm3'],3),h['bounds'])
