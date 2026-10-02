exec(open(__file__.replace('mount_check.py','native_audit.py')).read().split('hits=[]')[0])
hits=[];pins=[]
for r in D['records']:
 if r['part']!='2780.dat' or r['motion']!='fixed':continue
 c=np.array(r['pos'])*.4
 core=m.Manifold.cylinder(16,2.39,circular_segments=48,center=True).rotate([90,0,0]).translate(c.tolist())
 flange=m.Manifold.cylinder(.7,3.15,circular_segments=48,center=True).rotate([90,0,0]).translate(c.tolist())
 pin=core+flange;pins.append((r['record_id'],pin))
 for name,part in fixed.items():
  aa=np.array(pin.bounding_box());bb=np.array(part.bounding_box())
  if np.any(np.minimum(aa[3:],bb[3:])-np.maximum(aa[:3],bb[:3])<.001):continue
  vol=(pin^part).volume()
  if vol>.05:hits.append([r['record_id'],name,round(vol,3)])
for i,(name,p) in enumerate(pins):
 for other,q in pins[i+1:]:
  vol=(p^q).volume()
  if vol>.05:hits.append([name,other,round(vol,3)])
(O/'Mount pin checks.json').write_text(json.dumps(dict(pins=len(pins),hits=hits,method='Fixed base-pin cores and collars versus printed parts and each other; friction ribs excluded.'),indent=2))
print('Mount pin hits',len(hits));print(json.dumps(hits,indent=2))
