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

phase['P route gear 0']=0;ratios['P route gear 0']=1
for i,t in enumerate([8,8,16,8,8,16],1):link('P route gear '+str(i-1),'P route gear '+str(i),t,(-1)**i*16/t)
phase['Cin route gear 0']=0;ratios['Cin route gear 0']=1
for i in range(1,6):link('Cin route gear '+str(i-1),'Cin route gear '+str(i),16,(-1)**i)
for ac in ['P','X','C','S']:
 data_phase=phase['P route gear 6'] if ac=='S' else 0
 data_ratio=1
 for side in ['left','right']:
  n=ac+' R-P-'+side;phase[n]=phase['Cin route gear 5'] if ac=='C' and side=='left' else data_phase;ratios[n]=-1 if ac=='C' and side=='left' else data_ratio
 link(ac+' R-P-left',ac+' L102',16,-ratios[ac+' R-P-left'])
 link(ac+' R-P-right',ac+' R-P-idler-back',8,-2*data_ratio)
 phase[ac+' R-P-idler-front']=phase[ac+' R-P-idler-back'];ratios[ac+' R-P-idler-front']=ratios[ac+' R-P-idler-back']
 link(ac+' R-P-idler-front',ac+' L072',16,data_ratio)
worst=[]
for a,b in pairs:
 vals=[(at(a,phase[a]+t*ratios[a]).intersection(at(b,phase[b]+t*ratios[b])).area,float(t)) for t in np.arange(0,360,2.5)]
 area,ang=max(vals);worst.append(dict(a=a,b=b,max_outline_overlap_mm2=area,input_angle=ang))
print(json.dumps(worst,indent=2));(OUT/'Gear checks.json').write_text(json.dumps(dict(method='Projected native tooth silhouettes over a full revolution; shared compound gears use one phase offset. Not loaded contact simulation.',phases_degrees=phase,ratios=ratios,steps=144,pairs=worst),indent=2))
