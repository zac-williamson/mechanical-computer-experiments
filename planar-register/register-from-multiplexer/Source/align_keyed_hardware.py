"""Align keyed gear/bush bores to their shaft; preserve each part's actual teeth.
Run after geometry generation, before phase-coupled simulation and viewer publication.
"""
from pathlib import Path
import numpy as np,trimesh,json
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development'
h=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];arr={p['id']:v[p['offset']//3:p['offset']//3+p['vertices']].copy() for p in h};report=[]
def phase(a,axis,center,station=None):
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);c=np.array(center,dtype=float);c[axis]=(t.bounds[0,axis]+t.bounds[1,axis])/2 if station is None else station
 sec=t.section(plane_origin=c,plane_normal=np.eye(3)[axis]);axes=[i for i in range(3) if i!=axis]
 if sec is None:raise ValueError('Missing bore section')
 p=(sec.vertices-c)[:,axes];r=np.linalg.norm(p,axis=1);mn=r[r>.5].min();points=p[np.abs(r-mn)<.002]
 if len(points)<4 or not 1.05<mn<1.45:raise ValueError(('No cross-corner evidence',mn,len(points)))
 angles=np.arctan2(points[:,1],points[:,0]);ph=np.angle(np.exp(4j*angles).mean())/4-np.pi/4
 # Y rotation is negative angle in the XZ plane.
 return np.degrees(ph)*(1 if axis==0 else -1)
for p in h:
 n=p['id'];bank=p.get('bank');axis=0;center=None;shaft=None
 if bank=='Memory' and ('input' in n or 'power axle' in n):center=[0,10.2,-16];shaft='Memory — common 1 power axle'
 if bank=='Memory' and 'idler' in n:center=[0,10.2+np.sqrt(33.75),-10.5];shaft='Memory — A-idler-shaft'
 if bank=='Write' and ('B-input' in n or 'B-shaft' in n):center=[0,10.2,0];shaft='Write — B-shaft'
 if n.startswith('Cam roller'):center=[-5.05,0,53.3];shaft='Cam roller 2L axle';axis=1
 if center is None or n==shaft:continue
 try:angle=(phase(arr[shaft],axis,center)-phase(arr[n],axis,center)+45)%90-45
 except ValueError as e:report.append(dict(part=n,status='UNRESOLVED',reason=str(e)));continue
 T=trimesh.transformations.rotation_matrix(np.radians(angle),np.eye(3)[axis],center);arr[n]=trimesh.transform_points(arr[n],T);p['key_alignment_deg']=float(angle);report.append(dict(part=n,shaft=shaft,correction_deg=float(angle),status='ALIGNED CROSS SECTION'))
# Align the sleeve/ring as a unit, and the keyed retainers, to each split output shaft.
for bank,x,z in [('Memory',0,0),('Write',-85.0,16)]:
 ref=arr[bank+' — O-right 5L axle'];center=[0,10.2,z];target=phase(arr['Memory — C-shaft'],0,[0,10.2,16]) if bank=='Write' else phase(ref,0,center)
 for suffix in ['O-left 5L axle','O-right 5L axle']:
  name=bank+' — '+suffix;angle=(target-phase(arr[name],0,center)+45)%90-45;arr[name]=trimesh.transform_points(arr[name],trimesh.transformations.rotation_matrix(np.radians(angle),[1,0,0],center))
 sleeve=bank+' — L097';angle=(target-phase(arr[sleeve],0,center,x+8)+45)%90-45
 for name in [sleeve,bank+' — L099']:
  arr[name]=trimesh.transform_points(arr[name],trimesh.transformations.rotation_matrix(np.radians(angle),[1,0,0],center));report.append(dict(part=name,correction_deg=float(angle),status='SLEEVE/RING ASSEMBLY ALIGNED'))
 for suffix in ['L069','L105','O-left','O-right']:
  name=bank+' — '+suffix
  if name not in arr:continue
  angle=(target-phase(arr[name],0,center)+45)%90-45;arr[name]=trimesh.transform_points(arr[name],trimesh.transformations.rotation_matrix(np.radians(angle),[1,0,0],center));report.append(dict(part=name,correction_deg=float(angle),status='OUTPUT RETAINER ALIGNED'))
name='2L LEGO axle joiner 59443';center=[0,10.2,16];angle=(phase(arr['Memory — C-shaft'],0,center)-phase(arr[name],0,center,-36.4)+45)%90-45
arr[name]=trimesh.transform_points(arr[name],trimesh.transformations.rotation_matrix(np.radians(angle),[1,0,0],center));report.append(dict(part=name,correction_deg=float(angle),status='WORM COUPLING ALIGNED'))
arrays=[]
for p in h:p['offset']=sum(a.size for a in arrays);arrays.append(arr[p['id']])
np.savez_compressed(O/'hardware.npz',vertices=np.concatenate(arrays));(O/'hardware.json').write_text(json.dumps(h,indent=2));(O/'Keyed hardware alignment.json').write_text(json.dumps(dict(parts=report,limits='Cross-section angular alignment only. Does not certify interference fit, tooth meshing, or physical tolerances. Teeth are not edited.'),indent=2));print(json.dumps(report,indent=2))
