"""Conservative full-rotation outline from the actual native clutch surface."""
import numpy as np
import manifold3d as m
def clutch_envelope(t,radial_allowance=0):
 tri=t.triangles;edges=np.concatenate([tri[:,[0,1]],tri[:,[1,2]],tri[:,[2,0]]]);xs=sorted(set(np.round(t.vertices[:,0],5)))
 stations=sorted(set([xs[0],xs[-1]]+[x for a in xs for x in [a-.0001,a+.0001] if xs[0]<x<xs[-1]]));prof=[]
 for x in stations:
  lo=edges[:,0,0];hi=edges[:,1,0];ok=(np.minimum(lo,hi)<=x+1e-7)&(np.maximum(lo,hi)>=x-1e-7)&(abs(hi-lo)>1e-9)
  ee=edges[ok];w=(x-ee[:,0,0])/(ee[:,1,0]-ee[:,0,0]);p=ee[:,0]+w[:,None]*(ee[:,1]-ee[:,0]);r=np.linalg.norm(p[:,1:]-[10.2,0],axis=1)
  if len(r):prof.append([float(x),float(max(r))+radial_allowance])
 poly=[[0,prof[0][0]]]+[[rr,x] for x,rr in prof]+[[0,prof[-1][0]]]
 s=m.CrossSection([poly]).revolve(circular_segments=96).rotate([0,90,0]).translate([0,10.2,0])
 assert s.status()==m.Error.NoError and s.volume()>1000,(s.status(),s.volume())
 return s
