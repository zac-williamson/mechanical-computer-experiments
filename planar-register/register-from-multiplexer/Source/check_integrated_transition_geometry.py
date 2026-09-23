import sys,json,numpy as np,trimesh,manifold3d as m,itertools
from pathlib import Path
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development';meta=json.loads((O/'printed-parts.json').read_text());pars=json.loads((O/'Integrated cam parameters.json').read_text());ts={p['id']:trimesh.load(O/(p['id']+'.stl')) for p in meta}
def solid(t):return m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
ss={k:solid(v) for k,v in ts.items()};frames=json.loads((O.parent/'register-mux-reference/multiplexer/Switching trace.json').read_text())['frames']
def lift(q):
 r=3.6;s=1.2;c=pars['crest'];u=pars['bolt_x']-q;t=c+r*s/np.sqrt(1+s*s)
 z=pars['high']+r if u<=c else pars['high']+np.sqrt(max(0,r*r-(u-c)**2)) if u<=t else pars['intercept']-s*u+r*np.sqrt(1+s*s)
 return max(0,z-53.3)
hits={}
cases=json.loads((O/'Transition timing audit.json').read_text())['cases']
poses=sorted({(round(f['qm'],5),round(f['qe'],5),round(f['s'],5)) for c in cases for f in c['frames']})
for qm,qe,up in poses:
 shapes={};bounds={};up=up
 for p in meta:
  n=p['id'];a=ss[n];q=qm if p['bank']=='Memory' else qe
  if p['motion']=='carriage':a=a.translate([q,0,0])
  if p['motion']=='bolt':a=a.translate([0,0,up])
  if p['motion']=='rocker':
   b=min(frames,key=lambda f:abs(f['q']-q))['b'];pv=np.array([13.192323604+(pars['write_x'] if p['bank']=='Write' else 0),10.2,32.128448698+(16 if p['bank']=='Write' else 0)]);a=a.translate((-pv).tolist()).rotate([0,b,0]).translate(pv.tolist())
  shapes[n]=a;bounds[n]=np.array(a.bounding_box()).reshape(2,3)
 for p1,p2 in itertools.combinations(meta,2):
  n1,n2=p1['id'],p2['id'];b1,b2=bounds[n1],bounds[n2]
  if np.any(np.minimum(b1[1],b2[1])-np.maximum(b1[0],b2[0])<=1e-5):continue
  vol=(shapes[n1]^shapes[n2]).volume()
  if vol>.01:
   key=n1+' / '+n2
   if key not in hits or vol>hits[key]['volume']:hits[key]=dict(volume=vol,qm=qm,qe=qe)
print(json.dumps(hits,indent=2));(O/'Transition printed intersections.json').write_text(json.dumps(hits,indent=2))
