import numpy as np, math, trimesh, manifold3d as m
CX=-5.05;Y=27.6;R=1.75;THICK=.25
# Two parallel strands follow the same U route. End arcs loop beneath each
# X-axis anchor. Width refers to the bolt head, not its obsolete side hooks.
def route(half,lift=0,anchor_z=42.8):
 left=[(CX-8.05,anchor_z),(CX-8.05,48+lift)]
 fillet=0 if half==6 else 1
 corner=np.array([CX-half+fillet,52.2-fillet+lift])
 left += [tuple(corner+(fillet+.45)*np.array([math.cos(t),math.sin(t)])) for t in np.linspace(math.pi,math.pi/2,13)]
 right=[(2*CX-x,z) for x,z in left[::-1]]
 return np.array(left+right)
def points(half=5,lift=0,anchor_z=42.8):
 p=route(half,lift,anchor_z);front=np.column_stack((p[:,0],np.full(len(p),Y-R),p[:,1]));rear=np.column_stack((p[::-1,0],np.full(len(p),Y+R),p[::-1,1]))
 def endarc(x,reverse=False):
  # Rounded rectangular loop, tangent to the underside of the ramped hook.
  b=BOTTOM_Z;rad=.35
  yz=[(Y-R,anchor_z),(Y-R,b+rad)]
  yz += [(Y-1.4+rad*math.cos(t),b+rad+rad*math.sin(t)) for t in np.linspace(math.pi,1.5*math.pi,9)[1:]]
  yz += [(Y+1.4,b)]
  yz += [(Y+1.4+rad*math.cos(t),b+rad+rad*math.sin(t)) for t in np.linspace(1.5*math.pi,2*math.pi,9)[1:]]
  yz += [(Y+R,anchor_z)]
  a=np.array([[x,y,z] for y,z in yz])
  return a[::-1] if reverse else a
 right=endarc(CX+8.05)[1:-1];left=endarc(CX-8.05,True)[1:-1]
 return np.concatenate((front,right,rear,left))

def length(half,lift=0,anchor_z=42.8):
 p=points(half,lift,anchor_z);return float(np.linalg.norm(np.roll(p,-1,axis=0)-p,axis=1).sum())
ANCHOR_Z=42.8983397163
# Match the former measured closed loop length with the new rounded wrap.
BOTTOM_Z=41.7
for _ in range(2):
 BOTTOM_Z+=(length(5,0,ANCHOR_Z)-72.981)/4
ROOT_Z=BOTTOM_Z+.35-.85

def mesh(lift=0):
 p=points(5,lift,ANCHOR_Z);vs=[];faces=[];N=12
 for i,q in enumerate(p):
  tangent=p[(i+1)%len(p)]-p[(i-1)%len(p)];tangent/=np.linalg.norm(tangent);ref=np.array([0.,1.,0.])
  if abs(tangent@ref)>.9:ref=np.array([1.,0.,0.])
  a=np.cross(tangent,ref);a/=np.linalg.norm(a);b=np.cross(tangent,a)
  vs.extend([q+THICK*(math.cos(t)*a+math.sin(t)*b) for t in np.linspace(0,2*math.pi,N,endpoint=False)])
 for i in range(len(p)):
  for j in range(N):
   a=i*N+j;b=i*N+(j+1)%N;c=((i+1)%len(p))*N+(j+1)%N;d=((i+1)%len(p))*N+j;faces.extend([[a,b,c],[a,c,d]])
 t=trimesh.Trimesh(vs,faces,process=True);t.fix_normals();return t
