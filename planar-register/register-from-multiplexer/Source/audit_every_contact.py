from pathlib import Path
exec((Path(__file__).parent/'integrated_poses.py').read_text())
records={}
for qm in [-3.75,0,3.75]:
 for qe in [-3.75,0,3.75]:
  for p in meta:cm.set_transform(p['id'],pose(p,qm,qe))
  _,pairs,data=cm.in_collision_internal(return_names=True,return_data=True)
  for d in data:
   key=tuple(sorted(d.names));r=records.setdefault(key,dict(pair=key,max_depth=0,sample=[qm,qe],contact_point=d.point.tolist()))
   if d.depth>r['max_depth']:r.update(max_depth=float(d.depth),sample=[qm,qe],contact_point=d.point.tolist())
rows=sorted(records.values(),key=lambda r:-r['max_depth']);(O/'Every contact inventory.json').write_text(json.dumps(dict(poses=9,parts=len(meta),contacts=rows,scope='Every printed and modelled LEGO part included. Surface contacts require classification; nine poses are diagnostic, not full transition validation.'),indent=2));print(json.dumps(rows[:60],indent=2));print('Pairs',len(rows))
