from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'Planar register';ps=json.loads((O/'printed-parts.json').read_text());hs=json.loads((O/'hardware.json').read_text());vv=np.load(O/'hardware.npz')['vertices'];P={p['id']:trimesh.load(O/(p['id']+'.stl')) for p in ps};H={h['id']:vv[h['offset']//3:h['offset']//3+h['vertices']].copy() for h in hs};meta={p['id']:p for p in ps+hs}
# Import pose implementation without running diagnostic inventory.
s=(R/'Source/full_contact_inventory.py').read_text();ns={};exec(s[s.index('def pose'):s.index('records={}')],dict(np=np,trimesh=trimesh,trace=json.loads((R.parent/'work/register-mux-reference/multiplexer/Switching trace.json').read_text())['frames']),ns);pose=ns['pose']
rows=[]
for r in json.loads((O/'Full contact inventory.json').read_text())['contacts']:
 a,b=r['pair'];qm,qe=r['sample'];ta=pose(meta[a],qm,qe);tb=pose(meta[b],qm,qe)
 if a in P and b in P:
  def solid(t,T):return m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True))).transform(T[:3,:])
  vol=float((solid(P[a],ta)^solid(P[b],tb)).volume());rows.append(dict(pair=[a,b],volume_mm3=vol,sample=[qm,qe]));continue
 if a not in P and b in P:a,b=b,a;ta,tb=tb,ta
 if a not in P:continue
 points=H[b];tri=points.reshape(-1,3,3);points=np.unique(np.concatenate([points,tri.mean(1),(tri[:,0]+tri[:,1])/2,(tri[:,1]+tri[:,2])/2,(tri[:,2]+tri[:,0])/2]),axis=0);points=trimesh.transform_points(points,np.linalg.inv(ta)@tb);bounds=P[a].bounds;points=points[np.all((points>=bounds[0]-.01)&(points<=bounds[1]+.01),axis=1)]
 depth=float(trimesh.proximity.signed_distance(P[a],points).max()) if len(points) else 0
 rows.append(dict(pair=[a,b],max_sampled_surface_interior_mm=depth,sample=[qm,qe]))
(O/'Inventory penetration checks.json').write_text(json.dumps(rows,indent=2));print(json.dumps([r for r in rows if r.get('volume_mm3',0)>.005 or r.get('max_sampled_surface_interior_mm',0)>.03],indent=2))
