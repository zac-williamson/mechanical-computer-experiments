from pathlib import Path
import json,numpy as np,trimesh
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import affinity
R=Path(__file__).resolve().parents[1];O=R/'Planar register';hm=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];arr={h['id']:v[h['offset']//3:h['offset']//3+h['vertices']].copy() for h in hm}
def outline(a):
 tri=a.reshape(-1,3,3)[:,:,[1,2]];return unary_union([Polygon(t) for t in tri if abs(np.linalg.det([t[1]-t[0],t[2]-t[0]]))>1e-9])
# Check every positive pair, including the formerly missed direct pair.
centres={'A-input':(10.2,-16),'A-idler':(10.2+np.sqrt(33.75),-10.5),'L072':(10.2,0),'B-input':(10.2,-16),'L102':(10.2,0)}
phases={n:0. for n in centres};shapes={n:outline(arr['Memory — '+n]) for n in centres};reports=[]
for na,nb,ra,rb,pitch in [('A-input','A-idler',1,-1,45),('A-idler','L102',-1,.5,22.5),('B-input','L072',1,-1,22.5)]:
 best=None
 for phase in np.arange(0,pitch,.25):
  areas=[affinity.rotate(shapes[na],ra*t+phases[na],origin=centres[na]).intersection(affinity.rotate(shapes[nb],rb*t+phase,origin=centres[nb])).area for t in np.arange(0,45,1.25)]
  score=(max(areas),sum(areas),float(phase))
  if best is None or score<best:best=score
 phases[nb]=best[2];areas=[];gaps=[]
 for t in np.arange(0,720,.5):
  ga=affinity.rotate(shapes[na],ra*t+phases[na],origin=centres[na]);gb=affinity.rotate(shapes[nb],rb*t+phases[nb],origin=centres[nb]);areas.append(ga.intersection(gb).area);gaps.append(ga.distance(gb))
 reports.append(dict(pair=[na,nb],max_overlap_mm2=max(areas),min_gap_mm=min(gaps),max_gap_mm=max(gaps),samples=1440))
for n,phase in phases.items():arr['Memory — '+n]=trimesh.transform_points(arr['Memory — '+n],trimesh.transformations.rotation_matrix(np.radians(phase),[1,0,0],[0,*centres[n]]))
# Rotation-independent tip-circle bound proves non-meshing for every phase.
radii={n:float(np.linalg.norm(arr['Memory — '+n][:,1:]-centres[n],axis=1).max()) for n in ['A-input','L102']}
clearance=16-sum(radii.values());assert clearance>2
(O/'Memory gear mesh.json').write_text(json.dumps(dict(meshes=reports,positive_direct_pair_tip_circle_clearance_mm=clearance,positive_ratio=.5,negative_ratio=-1,qualification='Geometric checks only; no load qualification.'),indent=2));print('Memory meshes',reports,'unintended-pair clearance',clearance)
a=outline(arr['Write — B-input']);b=outline(arr['Write — L102']);best=None
for phase in np.arange(0,22.5,.25):
 areas=[affinity.rotate(a,float(t),origin=(10.2,0)).intersection(affinity.rotate(b,float(phase-t),origin=(10.2,16))).area for t in np.arange(0,22.5,.625)]
 score=(max(areas),sum(areas),float(phase))
 if best is None or score<best:best=score
phase=best[2];arr['Write — L102']=trimesh.transform_points(arr['Write — L102'],trimesh.transformations.rotation_matrix(np.radians(phase),[1,0,0],[0,10.2,16]))
areas=[]
for t in np.arange(0,360,.5):areas.append(affinity.rotate(a,float(t),origin=(10.2,0)).intersection(affinity.rotate(b,float(phase-t),origin=(10.2,16))).area)
out=[]
for h in hm:h['offset']=sum(x.size for x in out);h['vertices']=len(arr[h['id']]);out.append(arr[h['id']])
np.savez_compressed(O/'hardware.npz',vertices=np.concatenate(out));(O/'hardware.json').write_text(json.dumps(hm,indent=2))
(O/'WRITE gear mesh.json').write_text(json.dumps(dict(layout='Direct WRITE worm axle; D direct 16T mesh to opposite clutch gear',data_ratio=-1,phase_deg=phase,samples=720,max_projected_overlap_mm2=max(areas)),indent=2));print('D direct mesh',max(areas))
