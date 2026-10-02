import os
import numpy as np,math,trimesh
TURN=np.diag([-1.,1.,-1.])
def lift(q):return -2.8*np.clip((q+1.85)/2.35,0,1)
def transform(actor,motion,st,actors):
 T=np.eye(4);d=np.array(actors.get(actor,[0,0,0]));p=st.get(actor,dict(q=0,b=0,w=0,g=0,offset=0));q=p['q'];R=np.eye(3)
 if actor=='link':T[0,3]=-st['W']['q'];return T
 if motion in ['carriage','carriage-global']:T[:3,3]=R@np.array([q,0,0])
 if motion=='clutch-ring':T[:3,3]=R@np.array([np.sign(q)*max(0,abs(q)-.4),0,0])
 if motion in ['carriage-worm','reaction-rotating','rocker']:
  axis=R@np.array([1,0,0] if motion=='carriage-worm' else [0,0,1]);angle=p['w'] if motion=='carriage-worm' else p['g'] if motion=='reaction-rotating' else p['b'];T[:3,:3]=trimesh.transformations.rotation_matrix(math.radians(angle),axis)[:3,:3]
  c=d+R@np.array([0,10.2,32] if motion=='carriage-worm' else [0,2.2,32] if motion=='reaction-rotating' else [14.500924592586399,-3.261892187851191,0]);T[:3,3]=c-T[:3,:3]@c
  if motion=='carriage-worm':T[:3,3]+=R@np.array([q+p['offset'],0,0])
 if motion=='bolt':T[2,3]=lift(st['W']['q'])
 return T
