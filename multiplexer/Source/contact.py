from common import *
from shapely.geometry import Point, Polygon
from shapely import affinity, area, intersection
from shapely.ops import unary_union
DATA=Path(__file__).parent/'Data'
pads=unary_union([Point(*(pv+[-8,-4])).buffer(2.6,quad_segs=24),Point(*(pv+[8,-4])).buffer(2.6,quad_segs=24)])
t=trimesh.load(ROOT/'Left carriage half.stl');section=t.section(plane_origin=[0,0,40],plane_normal=[0,0,1]);st=Polygon()
for ring in section.discrete:st=st.symmetric_difference(Polygon(ring[:,:2]))
c=np.load(DATA/'contact.npz');free=c['free'];betas=c['betas'];gammas=c['gammas']
ls=np.array([affinity.rotate(pads,b,origin=tuple(pv)) for b in betas])
def trace(valid,d,betas=betas,gammas=gammas):
    ix=np.flatnonzero(valid[0])
    if not len(ix):return False
    b=ix[np.argmin(abs(betas[ix]))]
    for k in range(360):
        row=valid[(d*k)%len(gammas)];ix=np.flatnonzero(row)
        if not len(ix):return False
        j=ix[np.argmin(abs(ix-b))]
        if abs(betas[j]-betas[b])>2:return False
        lo=hi=j
        while lo>0 and row[lo-1]:lo-=1
        while hi<len(row)-1 and row[hi+1]:hi+=1
        b=int(np.clip(np.argmin(abs(betas)),lo,hi))
    return True
