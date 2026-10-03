"""Strict face-normal and layer-growth checks in the recorded print orientation."""
import numpy as np,trimesh,manifold3d as m
def solid(t):return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
def qualify(t,axis,sign):
 T=trimesh.geometry.align_vectors(np.eye(3)[axis]*sign,[0,0,1]);t=t.copy();t.apply_transform(T);shift=-t.bounds[0];t.apply_translation(shift)
 bad=(t.face_normals[:,2]<-np.sqrt(.5)-1e-5)&(t.triangles_center[:,2]>.001);s=solid(t);step=.2;previous=s.slice(.001);peak=0;issues=[]
 for z in np.arange(.101,t.bounds[1,2],step):
  now=s.slice(float(z));dz=z-(.001 if z<.2 else z-step);growth=(now-previous.offset(float(dz+.002),m.JoinType.Round,2,32)).area()
  peak=max(peak,growth)
  if growth>.025:issues.append(dict(height_mm=float(z),unsupported_growth_mm2=float(growth)))
  previous=now
 area=float(t.area_faces[bad].sum());r=dict(assembly_to_print_rotation=T.tolist(),translation_mm=shift.tolist(),size_mm=t.extents.tolist(),volume_mm3=float(t.volume),watertight=bool(t.is_watertight),solids=len(t.split()),unsupported_face_area_mm2=area,layer_height_mm=step,maximum_unsupported_layer_growth_mm2=peak,layer_growth_failures=issues,print_geometry_pass=bool(t.is_watertight and len(t.split())==1 and area<.01 and not issues))
 return t,r
