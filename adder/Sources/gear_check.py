import os
from pathlib import Path
import sys,json,math
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import affinity, make_valid, set_precision
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT;D=json.loads((OUT/'Assembly manifest.json').read_text());sys.path.insert(0,str(Path(__file__).resolve().parent));from render_ldraw import LDraw
lib=LDraw(str(Path(os.environ.get('LDRAW_PATH','/Applications/Studio 2.0/ldraw'))/'parts/23948.dat'));rr={r['record_id']:r for r in D['records']};poly={};phase={};ratios={};pairs=[]
def outline(id):
 if id in poly:return poly[id]
 r=rr[id];t,_=lib.mesh(r['part']);a=math.radians(-r.get('phase_deg',0));c,si=math.cos(a),math.sin(a);rx=np.array([[1,0,0],[0,c,-si],[0,si,c]]);t=t*.4@(rx@np.array(r['matrix']).reshape(3,3)).T;p=unary_union([Polygon(a[:,1:]) for a in t if Polygon(a[:,1:]).area>1e-8]);p=make_valid(p);poly[id]=p;return p
def at(id,angle):
 y,z=np.array(rr[id]['pos'])[1:]*.4;return set_precision(make_valid(affinity.translate(affinity.rotate(outline(id),angle,origin=(0,0)),y,z)),1e-6)
def link(a,b,teeth_b,ratio_b):
 scores=[];tt=np.arange(0,22.5,2.5);pa=[at(a,phase[a]+t*ratios[a]) for t in tt]
 period=360/teeth_b
 for ang in np.arange(0,period,.125):scores.append((max(x.intersection(at(b,ang+t*ratio_b)).area for x,t in zip(pa,tt)),float(ang)))
 area=min(x[0] for x in scores);good=[x[1] for x in scores if x[0]<=area+1e-8];ang=float(np.angle(np.mean(np.exp(2j*np.pi*np.array(good)/period)))%(2*np.pi)*period/(2*np.pi));phase[b]=ang;ratios[b]=ratio_b;pairs.append((a,b));print(a,'->',b,'phase',ang,'intersection area',area,flush=True)


phase['X B direct']=phase['X B reverse drive']=0
ratios['X B direct']=ratios['X B reverse drive']=1
link('X B direct','X L072',16,-1)
link('X B reverse drive','X compound back',8,-2)
phase['X compound front']=phase['X compound back'];ratios['X compound front']=-2
link('X compound front','X L102',16,1)
for name in ['P direct input','P reversing input','C inverted Bprime input']:
 phase[name]=phase['X L072'];ratios[name]=-1
link('P direct input','P L072',16,1)
link('P reversing input','P compound back',8,2)
phase['P compound front']=phase['P compound back'];ratios['P compound front']=2
link('P compound front','P L102',16,-1)
link('C inverted Bprime input','C L072',16,1)
for ac,end in [('C','L102'),('S','L072')]:
 phase[ac+' A reversing drive']=0;ratios[ac+' A reversing drive']=1
 link(ac+' A reversing drive',ac+' A compound back',8,-2)
 phase[ac+' A compound front']=phase[ac+' A compound back'];ratios[ac+' A compound front']=-2
 link(ac+' A compound front',ac+' '+end,16,1)
phase['S A direct']=0;ratios['S A direct']=1
link('S A direct','S L102',16,-1)
worst=[]
for a,b in pairs:
 vals=[(at(a,phase[a]+t*ratios[a]).intersection(at(b,phase[b]+t*ratios[b])).area,float(t)) for t in np.arange(0,360,2.5)]
 area,ang=max(vals);worst.append(dict(a=a,b=b,max_outline_overlap_mm2=area,input_angle=ang))
print(json.dumps(worst,indent=2));(OUT/'Gear checks.json').write_text(json.dumps(dict(method='Projected native tooth silhouettes over a full revolution; shared compound gears use one phase offset. Not loaded contact simulation.',phases_degrees=phase,ratios=ratios,steps=144,pairs=worst),indent=2))
