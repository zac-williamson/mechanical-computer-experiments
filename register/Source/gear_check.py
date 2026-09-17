from pathlib import Path
import sys,json,math
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import affinity
ROOT=Path(__file__).resolve().parent;import os;OUT=Path(os.environ.get('REGISTER_OUTPUT',str(ROOT.parent)));D=json.loads((OUT/'Assembly manifest.json').read_text());sys.path.insert(0,str(ROOT));from render_ldraw import LDraw
lib=LDraw(str(Path(os.environ.get('LDRAW_DIR','/Applications/Studio 2.0/ldraw'))/'parts/23948.dat'));rr={r['record_id']:r for r in D['records']};poly={};phase={};ratios={};pairs=[]
def outline(id):
 if id in poly:return poly[id]
 r=rr[id];t,_=lib.mesh(r['part']);a=math.radians(-r.get('phase_deg',0));c,si=math.cos(a),math.sin(a);rx=np.array([[1,0,0],[0,c,-si],[0,si,c]]);t=t*.4@(rx@np.array(r['matrix']).reshape(3,3)).T;p=unary_union([Polygon(a[:,1:]) for a in t if Polygon(a[:,1:]).area>1e-8]);poly[id]=p;return p
def at(id,angle):
 y,z=np.array(rr[id]['pos'])[1:]*.4;return affinity.translate(affinity.rotate(outline(id),angle,origin=(0,0)),y,z)
def link(a,b,teeth_b,ratio_b):
 scores=[];tt=np.arange(0,22.5,2.5);pa=[at(a,phase[a]+t*ratios[a]) for t in tt]
 period=360/teeth_b
 for ang in np.arange(0,period,.125):scores.append((max(x.intersection(at(b,ang+t*ratio_b)).area for x,t in zip(pa,tt)),float(ang)))
 area=min(x[0] for x in scores);good=[x[1] for x in scores if x[0]<=area+1e-8];ang=float(np.angle(np.mean(np.exp(2j*np.pi*np.array(good)/period)))%(2*np.pi)*period/(2*np.pi));phase[b]=ang;ratios[b]=ratio_b;pairs.append((a,b));print(a,'->',b,'phase',ang,'intersection area',area,flush=True)
phase['K power gear -16']=phase['K power gear 16']=0;ratios['K power gear -16']=ratios['K power gear 16']=1
link('K power gear 16','K direct idler',8,-2);link('K direct idler','K L072',16,1)
link('K power gear -16','K first reversing idler',8,-2);link('K first reversing idler','K second reversing idler',8,2);link('K second reversing idler','K L102',16,-1)
phase['W L072']=0;ratios['W L072']=1;link('W L072','W port gear',16,-1)
phase['E L102']=0;ratios['E L102']=1;link('E L102','E port gear',16,-1)
worst=[]
for a,b in pairs:
 vals=[(at(a,phase[a]+t*ratios[a]).intersection(at(b,phase[b]+t*ratios[b])).area,float(t)) for t in np.arange(0,360,2.5)]
 area,ang=max(vals);worst.append(dict(a=a,b=b,max_outline_overlap_mm2=area,input_angle=ang))
print(json.dumps(worst,indent=2));(OUT/'Gear checks.json').write_text(json.dumps(dict(method='Projected native tooth silhouettes; conservative full-width envelopes, not loaded tooth-contact simulation',phases_degrees=phase,ratios=ratios,steps=144,pairs=worst),indent=2)+'\n')
