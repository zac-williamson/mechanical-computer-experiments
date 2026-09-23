"""Opposite-side WRITE; integral carriage cam; base-supported long bolt guide.
Development generator. Never represents hardware qualification.
"""
from pathlib import Path
import json, numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];B=R.parent/'work/register-before-left-cam';O=R.parent/'work/integrated-cam-development';SRC=R.parent/'work/register-mux-reference/multiplexer';O.mkdir(exist_ok=True)
WX=-85.0;DX=WX-96.8;BX=-5.05;SLOPE=1.2;RAD=3.6;CZ=53.3;LIFT=5.4;HIGH=CZ+LIFT-RAD
INTERCEPT=CZ+SLOPE*(BX+3.75)-RAD*np.sqrt(1+SLOPE*SLOPE);CREST=(INTERCEPT-HIGH)/SLOPE

def solid(t):return m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
def mesh(s):
 a=s.to_mesh64();return trimesh.Trimesh(np.array(a.vert_properties)[:,:3],np.array(a.tri_verts),process=False)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,axis,c):
 s=m.Manifold.cylinder(b-a,r,circular_segments=64)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 v=list(c);v[axis]=a;return s.translate(v)
def xz(poly,y0,y1):return m.CrossSection([np.array(poly)],m.FillRule.EvenOdd).extrude(y1-y0).transform([[1,0,0,0],[0,0,1,y0],[0,1,0,0]])
def src(n):return solid(trimesh.load(SRC/(n+'.stl')))
meta=json.loads((B/'printed-parts.json').read_text());P={p['id']:solid(trimesh.load(B/(p['id']+'.stl'))) for p in meta}
for p in meta:
 if p['bank']=='Write':P[p['id']]=P[p['id']].translate([DX,0,0])
for n in ['Write — Direct cam plate','Direct lock bolt','Unified rear backbone']:P.pop(n)
P['Memory — Front bearing cheek']=src('Front bearing cheek')
P['Memory — Carriage fork and roof']=src('Carriage fork and roof')
for x in [-9,-1.1]:P['Memory — Carriage fork and roof']-=box([x-2.9,16.8,38.8],[x+2.9,22.8,45.4])
# Integral plate grows from original roof; no added material beneath the bearing bed plane.
P['Write — Carriage fork and roof']=src('Carriage fork and roof').translate([WX,0,16])
cam=xz([[WX-15.6,54.9],[WX+18,40],[ -25,40],[-20,46.5],[-1,46.5],[-1,INTERCEPT+SLOPE],[CREST,HIGH],[WX+21,HIGH],[WX+16,61.3],[WX-15.6,61.3]],25.6,31)
P['Write — Carriage fork and roof']+=cam+box([WX-15.6,20,54.9],[WX+16,31,61.3])
# Only relieve the unused back/top of the existing support, clear of cheek pins and axle bearings.
P['Write — Right side frame']-=box([WX+20,23.5,39.4],[WX+45,33,65])
# Bolt: broad guided head, narrow tip. Roller axle passes through head.
bolt=box([BX-6,18,49.6],[BX+6,25.2,71])+box([BX-1.95,18,42.9],[BX+1.95,22,49.7])
bolt+=(box([BX-1.55,18.4,42.1],[BX+1.55,21.6,42.11])+box([BX-1.95,18,42.89],[BX+1.95,22,42.91])).hull()
bolt-=cy(2.6,17.9,25.3,1,[BX,0,CZ])
# Raised side guides bear close to roller axis, with a through-window for its axle/bush.
guide=box([BX-10,14,52],[BX+10,29.2,70])-box([BX-6.4,17.6,51.9],[BX+6.4,25.6,70.1])
guide-=box([BX-4.6,13.9,51.9],[BX+4.6,29.3,70.1])
guide-=box([BX-10.1,25.2,51.9],[BX+10.1,29.3,56])
# Lower tip guide constrains lock-tip play; front uprights clear the rear cam.
guide+=box([BX-10,14,46],[BX-6.4,17.6,56.1])+box([BX+6.4,14,46],[BX+10,17.6,56.1])
guide+=box([BX-10,14,46],[BX+10,24.8,49.2])-box([BX-2.25,17.7,45.9],[BX+2.25,22.3,49.3])
# Rear spine and diagonal ribs carry guide loads directly into the existing base.
base=solid(trimesh.load(R/'Superseded planar parts/Memory — Common baseboard.stl'))^box([-50,0,-40],[50,50,18])
wbase=(solid(trimesh.load(R/'Superseded planar parts/Write — Common baseboard.stl'))^box([50,0,-20],[150,50,34])).translate([DX,0,0])
base+=wbase+box([WX+32.2,32.8,-10.7],[-26.8,38.4,-2.7])
left=src('Left side frame').translate([WX,0,16]);left-=box([WX-33,-5,-12],[WX-22,26.8,9.8]);left+=cy(5.5,WX-31.8,WX-24.2,0,[0,10.2,0])+box([WX-31.8,4.7,0],[WX-24.2,15.7,10.2]);left-=cy(2.65,WX-31.9,WX-24.1,0,[0,10.2,0]);base+=left
for z in [-2.4,26]:base+=cy(2.8,22.8,38.4,1,[WX-28,0,z])
# Pair of sloping open ribs, set behind carriage swept depth.
base+=xz([[-34,-18],[-26,-18],[BX-6.5,56],[BX-6.5,70],[BX-10,70],[-34,0]],32.8,38.4)
base+=xz([[26,-18],[34,-18],[34,0],[BX+10,70],[BX+6.5,70],[BX+6.5,56]],32.8,38.4)
# Guide halves integrated into rear ribs, avoiding a cap fastener joint.
base+=box([BX-10,25.6,56],[BX-6.4,38.4,70])+box([BX+6.4,25.6,56],[BX+10,38.4,70])+guide
# Centered rear elastic anchors. Band loops in XZ, stays behind mechanism.
def lug(z):return cy(3.2,31.4,32.4,1,[BX,0,z])+cy(2.4,32.4,33.6,1,[BX,0,z])+cy(3.2,33.6,34.6,1,[BX,0,z])
base+=box([BX-4,31.4,45],[BX+4,32.4,51])+lug(48)
# Join anchor to both rear ribs without crossing the cam.
base+=box([BX-10,31.4,45],[BX+10,38.4,49])
base-=box([BX-4,32.4,44],[BX+4,39.7,52])
base+=lug(48)
bolt+=box([BX-4,24.8,67],[BX+4,32.4,71])+lug(69)
for xx in [-28,28]:
 for zz in [-18.4,11.5]:base-=cy(3.35,32.79,38.5,1,[xx,0,zz])
P['Direct lock bolt']=bolt;P['Unified rear backbone']=base
meta=[p for p in meta if p['id'] in P and p['id'] not in ['Direct lock bolt','Unified rear backbone']]
for n,bank,mo in [('Direct lock bolt','Lock','bolt'),('Unified rear backbone','Lock','fixed')]:meta.append(dict(id=n,bank=bank,motion=mo,source='integrated cam revision'))
for p in meta:
 t=mesh(P[p['id']].simplify(.001));t.merge_vertices(digits_vertex=6);t.update_faces(t.nondegenerate_faces(height=1e-9));t.update_faces(t.unique_faces());t.remove_unreferenced_vertices();assert t.is_watertight; t.export(O/(p['id']+'.stl'),file_type='stl_ascii');p.update(bounds=t.bounds.tolist(),watertight=bool(t.is_watertight),solids=len(t.split()))
(O/'printed-parts.json').write_text(json.dumps(meta,indent=2))
hs=json.loads((B/'hardware.json').read_text());v=np.load(B/'hardware.npz')['vertices'];arr={h['id']:v[h['offset']//3:h['offset']//3+h['vertices']].copy() for h in hs}
for h in hs:
 if h['bank']=='Write':arr[h['id']]+=[DX,0,0]
hs=[h for h in hs if not h['id'].startswith('Cam attachment friction pin') and h['id'] not in ['Write — O-right','Memory — U072']]
for h in hs:
 a=arr[h['id']]
 if h['id'].startswith('Cam roller'):a[:,0]+=BX-38;a[:,2]-=4
 if h['id'] in ['Memory — C-shaft','Write — O-shaft','Write — B-shaft']:
  center=0 if h['bank']=='Memory' else WX-4 if h['id']=='Write — B-shaft' else WX;a[:,0]=(a[:,0]-(a[:,0].min()+a[:,0].max())/2)*80/np.ptp(a[:,0])+center;h['lego_part']='3737';h['geometry_note']='Existing LEGO axle mesh length adjusted to standard 10L; verify against genuine part.'
 if h['id']=='2L LEGO axle joiner — envelope only':a[:,0]+=-40.4-(a[:,0].min()+a[:,0].max())/2
arrays=[]
for h in hs:h.update(offset=sum(a.size for a in arrays),vertices=len(arr[h['id']]));arrays.append(arr[h['id']])
(O/'hardware.json').write_text(json.dumps(hs,indent=2));np.savez_compressed(O/'hardware.npz',vertices=np.concatenate(arrays))
(O/'Integrated cam parameters.json').write_text(json.dumps(dict(write_x=WX,bolt_x=BX,slope=SLOPE,radius=RAD,closed_z=CZ,lift=LIFT,high=HIGH,intercept=INTERCEPT,crest=CREST,penetration=3.2,guide_span=14,guide_clearance=.4),indent=2))
print([(p['id'],p['solids']) for p in meta if p['solids']!=1]);print('Generated',len(meta),'printed',len(hs),'hardware')
