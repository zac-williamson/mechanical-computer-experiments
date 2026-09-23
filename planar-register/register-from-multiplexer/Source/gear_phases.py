"""Tune the three changed memory gear meshes using native LEGO tooth outlines.
Mesh silhouettes check angular clearance; they do not establish contact ratio or strength.
"""
from pathlib import Path
import json,numpy as np,trimesh
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import affinity
R=Path(__file__).resolve().parents[1];O=R/'Assembly';C=R/'Core layout'
h=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];arrays={p['id']:v[p['offset']//3:p['offset']//3+p['vertices']].copy() for p in h}
def project(a):
 tri=a.reshape(-1,3,3)[:,:,[1,2]]
 return unary_union([Polygon(p) for p in tri if abs(np.linalg.det(np.array([p[1]-p[0],p[2]-p[0]])))>1e-9])
centres={'A-input':(10.2,-16),'A-idler':(10.2+np.sqrt(80),-8),'L072':(10.2,0),'B-input':(10.2,-16),'L102':(10.2,0)}
shapes={n:project(arrays['Memory — '+n]) for n in centres}
phases={n:0. for n in centres};reports=[]
for a,b,sa,sb,pitch in [('A-input','A-idler',1,-2,45),('A-idler','L072',-2,1,22.5),('B-input','L102',1,-1,22.5)]:
 theta=np.linspace(0,22.5,37)[:-1]
 aa=[affinity.rotate(shapes[a],sa*t+phases[a],origin=centres[a]) for t in theta]
 best=None
 for phase in np.arange(0,pitch,.25):
  areas=[x.intersection(affinity.rotate(shapes[b],sb*t+float(phase),origin=centres[b])).area for x,t in zip(aa,theta)]
  cost=max(areas)
  if best is None or (cost,sum(areas))<(best[0],best[1]):best=(cost,sum(areas),float(phase))
 phases[b]=best[2]
 gaps=[];areas=[]
 for t in np.linspace(0,360,721)[:-1]:
  ga=affinity.rotate(shapes[a],sa*t+phases[a],origin=centres[a]);gb=affinity.rotate(shapes[b],sb*t+phases[b],origin=centres[b]);gaps.append(ga.distance(gb));areas.append(ga.intersection(gb).area)
 reports.append(dict(a=a,b=b,phase_a_deg=phases[a],phase_b_deg=phases[b],speed_ratio=[sa,sb],centre_distance_mm=float(np.linalg.norm(np.array(centres[a])-centres[b])),overlap_area_mm2_max=max(areas),minimum_outline_gap_mm=min(gaps),maximum_outline_gap_mm=max(gaps),verification_samples=720))
for n,ang in phases.items():
 if not ang:continue
 key='Memory — '+n;c=[0,*centres[n]];T=trimesh.transformations.rotation_matrix(np.radians(ang),[1,0,0],c);arrays[key]=trimesh.transform_points(arrays[key],T)
a=[]
for p in h:
 p['offset']=sum(x.size for x in a);a.append(arrays[p['id']])
 if p['bank']=='Memory' and p['source'] in phases:p['phase_deg']=phases[p['source']]
np.savez_compressed(O/'hardware.npz',vertices=np.concatenate(a));(O/'hardware.json').write_text(json.dumps(h,indent=2));report=dict(method='Native-mesh YZ silhouettes, sampled 0.5 degree full input revolution; phase search 0.25 degree. Projection overlaps may be intentional bevel/chamfer projections and require 3D review.',meshes=reports,limitations='No load, tooth deformation, backlash under tolerance, or dynamic contact simulation.')
(O/'Gear phase checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
