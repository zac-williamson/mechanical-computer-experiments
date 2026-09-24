"""Recover the source worm shaft's keyed assembly angle from its actual mesh.

The copied worm and input-clutch sleeve already carry this phase. New standard
axles must use the same phase; rotating their axes is not necessary.
"""
import numpy as np
import trimesh

def source_worm_phase(lookup,vertices):
 p=lookup['master C-shaft'];a=vertices[p['offset']//3:p['offset']//3+p['vertices']]
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 centre=np.array([0.,10.2,16.]);sec=t.section(plane_origin=centre,plane_normal=[1,0,0])
 if sec is None:raise ValueError('Missing source worm axle cross section')
 points=(sec.vertices-centre)[:,1:];radius=np.linalg.norm(points,axis=1)
 inner=radius[radius>.5].min();corners=points[abs(radius-inner)<.002]
 if len(corners)!=4 or not 1.05<inner<1.2:raise ValueError('Source axle cross section is not the expected LEGO cross')
 angles=np.arctan2(corners[:,1],corners[:,0]);phase=np.degrees(np.angle(np.exp(4j*angles).mean())/4-np.pi/4)
 return float((phase+45)%90-45)
