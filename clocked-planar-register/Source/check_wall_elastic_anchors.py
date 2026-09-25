"""Verify material exists at both named anchor centres for every elastic loop.
This checks anchor presence and kinematic attachment, not band force or retention.
"""
from pathlib import Path
import json,hashlib,math
import numpy as np,trimesh
from wall_pose import transform
R=Path(__file__).resolve().parents[1];O=R/'Wall register';P=json.loads((O/'parts.json').read_text());V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
traces=json.loads((R/'Compact layout/Compact contact-resolved operation.json').read_text())['cases'];frames=[c['frames'][i] for c in traces for i in np.linspace(0,len(c['frames'])-1,17,dtype=int)]
meshes={}
for p in P:
 if p['kind']=='printed':
  a=V[p['offset']//3:p['offset']//3+p['vertices']];meshes[p['id']]=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
lookup={p['id']:p for p in P};out=[]
for p in P:
 if p['kind']!='elastic':continue
 bank=p['bank'];motion=p['motion'];anchors=[[],[]];hosts=[]
 for f in frames:
  if motion=='actuator-band':
   x,z,sign={'master':(0,0,1),'slave':(106,0,1),'write':(-76,-16,-1),'clock':(40,-16,-1)}[bank]
   pivot=np.array([13.192323604,32.128448698]);a=np.array({'write':[30.192323604,29.295115365],'clock':[29.192323604,29.461782031]}.get(bank,[31.192323604,29.128448698]));t=math.radians(f[bank]['b']);rot=np.array([[math.cos(t),math.sin(t)],[-math.sin(t),math.cos(t)]]);b=pivot+rot@(np.array([22.192323604,30.628448698])-pivot)
   points=[np.array([x+sign*q[0],10.2,z+sign*q[1]]) for q in [a,b]]
   rr=np.array(p.get('assembly_rotation',np.eye(3)));tt=np.array(p.get('assembly_translation',[0,0,0]));points=[rr@q+tt for q in points]
   moving=('Control ' if p['module']=='control' else '')+bank+' Short lever';hosts=[None,moving]
  elif motion=='fork-band':
   gx,bias=(-60,1) if bank=='master_gate' else (60,-1)
   points=[[gx-12+f['rail']+f.get(bank+'_lag',0),19.55,7.8],[gx-12+f['rail']+6.5*bias,19.55,7.8]];hosts=[bank+' Carriage fork and roof','Local clock cam and fork bar']
  else:
   bx=-5.05+(106 if bank=='slave' else 0);points=[[bx,37.3,50],[bx,37.3,58.6+f[bank+'_lift']]];hosts=[None,bank+' lock bolt']
  for i,point in enumerate(points):anchors[i].append(point)
 for i,host in enumerate(hosts):
  points=np.array(anchors[i]);matched=[];inside=np.zeros(len(points),dtype=bool)
  candidates=[lookup[host]] if host else [q for q in P if q['kind']=='printed' and q['module']==p['module'] and q.get('motion','fixed')=='fixed']
  for q in candidates:
   local=np.array([trimesh.transform_points(pt[None,:],np.linalg.inv(transform(q,f)))[0] for pt,f in zip(points,frames)])
   tm=meshes[q['id']];bbox=tm.bounds;eligible=np.all((local>=bbox[0])&(local<=bbox[1]),axis=1)
   found=np.zeros(len(points),dtype=bool)
   if eligible.any():found[eligible]=tm.contains(local[eligible])
   if found.any():matched.append(q['id'])
   inside|=found
  out.append(dict(band=p['id'],anchor=i+1,hosts=matched,poses=len(points),missing_anchor_poses=int((~inside).sum())))
result=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),scope=__doc__,anchors=out,anchor_presence_pass=all(a['missing_anchor_poses']==0 for a in out),mechanically_qualified=False)
(O/'Elastic anchor checks.json').write_text(json.dumps(result,indent=2));print('Anchor presence',result['anchor_presence_pass'])
for x in out:
 if x['missing_anchor_poses']:print(x)
