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
 right=np.array([[CX+8.05,Y-R*math.cos(t),anchor_z-R*math.sin(t)] for t in np.linspace(0,math.pi,17)[1:-1]])
 left=np.array([[CX-8.05,Y+R*math.cos(t),anchor_z-R*math.sin(t)] for t in np.linspace(0,math.pi,17)[1:-1]])
 return np.concatenate((front,right,rear,left))
def length(half,lift=0,anchor_z=42.8):
 p=points(half,lift,anchor_z);return float(np.linalg.norm(np.roll(p,-1,axis=0)-p,axis=1).sum())
# Set the closed-state total loop centreline reduction to 3.6 mm.
ANCHOR_RISE=(3.6-(length(6)-length(5)))/4
ANCHOR_Z=42.8+ANCHOR_RISE
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
