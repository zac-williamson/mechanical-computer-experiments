"""Exact rigid transforms consumed by the coupled viewer (angles in degrees)."""
import numpy as np,trimesh

def pose(p,f):
 T=np.eye(4);mo=p['motion'];mem=p.get('bank')=='Memory'
 if 'joint' in p:
  j=p['joint'];T=trimesh.transformations.rotation_matrix(np.radians(f[j['field']]*j['ratio']),np.eye(3)['XYZ'.index(j['axis'])],j['center'])
 if mo in ['carriage','worm']:T[0,3]+=f['qm'] if mem else f['qe']
 if mo=='clutch-ring':T[0,3]+=f['rm'] if mem else f['re']
 if mo=='bolt':T[2,3]+=f['s']
 if mo=='rocker':T=trimesh.transformations.rotation_matrix(np.radians(f['bm'] if mem else f['be']),[0,1,0],[13.192323604+(0 if mem else -85.0),10.2,32.128448698+(0 if mem else 16)])
 return T
