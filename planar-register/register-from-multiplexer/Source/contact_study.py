from investigate import B,O,p,h,rows,pv,gc,proj
from shapely import affinity
import numpy as np,json
gear=proj(h['U022']);lever=proj(p['Short lever']);roof=proj(p['Carriage fork and roof'])
data=[]
for r in rows:
 g=affinity.rotate(gear,-r['g'],origin=tuple(gc[[0,2]]))
 l=affinity.rotate(lever,-r['b'],origin=tuple(pv[[0,2]]))
 c=affinity.translate(roof,xoff=r['q'])
 data.append(dict(q=r['q'],b=r['b'],g=r['g'],moving=r['moving'],gear_lever_gap=l.distance(g),gear_lever_overlap=l.intersection(g).area,roof_lever_gap=l.distance(c),roof_lever_overlap=l.intersection(c).area))
move=[x for x in data if x['moving']]; rest=[x for x in data if not x['moving']]
r={'scope':'2D XZ projections of native meshes along supplied kinematic trace; not contact/force simulation. Axial face overlap checked separately.','frames':len(data),'moving_frames':len(move),'moving_gear_lever_gap_mm':[min(x['gear_lever_gap'] for x in move),max(x['gear_lever_gap'] for x in move)],'moving_gear_lever_overlap_mm2_max':max(x['gear_lever_overlap'] for x in move),'all_gear_lever_overlap_mm2_max':max(x['gear_lever_overlap'] for x in data),'all_roof_lever_overlap_mm2_max':max(x['roof_lever_overlap'] for x in data),'face_overlap_Y_mm':max(0,min(p['Short lever'].bounds[1,1],h['U022'].bounds[1,1])-max(p['Short lever'].bounds[0,1],h['U022'].bounds[0,1])),'warning':'Tooth engagement under elastic deflection and friction is not established by a prescribed no-load trace.'}
(O/'Contact study.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
