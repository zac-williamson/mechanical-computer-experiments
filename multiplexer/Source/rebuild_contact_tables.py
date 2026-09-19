"""Regenerate the numerical gear-contact inputs for the current tooth profile."""
from common import *
import re,gzip,base64
from shapely.geometry import Polygon
from shapely import affinity,area,intersection
from shapely.ops import unary_union
DATA=Path(__file__).parent/'Data'
v=json.loads(re.search('id="data">(.*?)</script>',(ROOT/'Viewer.html').read_text()).group(1))
p=next(p for p in v['parts'] if p['id']=='U022')
a=np.frombuffer(gzip.decompress(base64.b64decode(v['geometry'])),dtype='<f4')[p['offset']:p['offset']+p['vertices']*3].reshape(-1,3,3)
gear=unary_union([Polygon(t[:,:2]) for t in a if Polygon(t[:,:2]).area>1e-8])
D=np.linalg.norm(pv-[0,2.2]);angle=np.degrees(np.arctan2(pv[1]-2.2,pv[0]))
g=affinity.rotate(affinity.translate(gear,-pv[0],-pv[1]),-angle,origin=(0,0)).buffer(.0001).buffer(-.0001).simplify(.0001)
a=np.asarray(g.exterior.coords)-[-D,0];rho=np.linalg.norm(a,axis=1);theta=np.arctan2(a[:,1],a[:,0]);phase=-np.degrees(np.angle(np.sum(rho**12*np.exp(8j*theta))))/8
g=affinity.rotate(g,phase,origin=(-D,0));profile=Polygon(np.load(DATA/'tooth_profile.npz')['xy'])
for filename,betas,gammas,offsets in [('contact.npz',np.arange(-40,40.001,.1),np.arange(0,45,.25),[(0,0)]),('tolerance_free.npz',np.arange(-35,35.001,.2),np.arange(0,45,.5),[(0,0),(.6,0),(-.6,0),(0,.6),(0,-.6),(.424,.424),(-.424,-.424),(.424,-.424),(-.424,.424)])]:
    ls=np.array([affinity.rotate(profile,b,origin=(0,0)) for b in betas]);matrices=[]
    for dx,dy in offsets:
        gs=[affinity.translate(affinity.rotate(g,a,origin=(-D,0)),dx,dy) for a in gammas]
        matrices.append(np.array([area(intersection(ls,z))<.0001 for z in gs]))
    data=dict(free=matrices[0] if len(offsets)==1 else np.array(matrices),betas=betas,gammas=gammas)
    if len(offsets)>1:data['offsets']=offsets
    np.savez_compressed(DATA/filename,**data);print(filename)
