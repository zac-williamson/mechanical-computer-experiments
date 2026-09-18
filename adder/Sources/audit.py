from pathlib import Path
import json,itertools,numpy as np,trimesh,manifold3d as m
O=Path(__file__).resolve().parents[1];D=json.loads((O/'Assembly manifest.json').read_text());rr={r['record_id']:r for r in D['records']}
teeth={'4019.dat':16,'10928.dat':8,'18946.dat':16,'32905.dat':None}
gear_count=sum(r['part'] in teeth for r in D['records'])
pairs=[]
for a,b in D['gear_pairs']:
 ra,rb=rr[a],rr[b];pa,pb=np.array(ra['pos'])*.4,np.array(rb['pos'])*.4
 distance=np.linalg.norm((pa-pb)[1:]);expected=(teeth[ra['part']]+teeth[rb['part']])*.5
 assert abs(distance-expected)<.001,(a,b,distance,expected)
 assert abs(pa[0]-pb[0])<.001,(a,b,'different planes')
 pairs.append(dict(a=a,b=b,distance_mm=distance,pitch_sum_mm=expected))
def output(signal,meshes):return signal^ (meshes%2)
rows=[]
for a,b,sub,ci in itertools.product(range(2),repeat=4):
 nb=output(b,2 if sub else 1)
 bp=nb^1
 p=output(nb,2 if ci else 1)
 carry=output(a,2) if p else output(nb,1)
 summ=output(a,1 if p else 2)
 expected=a+(b^sub)+ci
 assert summ==expected%2 and carry==expected//2
 rows.append(dict(A=a,B=b,Sub=sub,Cin=ci,Bprime=bp,P=p,Sum=summ,Cout=carry))
assert gear_count==30,gear_count
report=dict(gears=gear_count,previous_gears=45,actuators=3,clutches=4,logic_cases_passed=len(rows),rows=rows,pitch_checks=pairs,limits='Pitch and direction checks only; no tooth phase, load, or full clearance proof.')
(O/'Logic and pitch checks.json').write_text(json.dumps(report,indent=2))
T={p['id']:trimesh.load(O/p['path']) for p in D['prints']}
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
ss={n:solid(t) for n,t in T.items()}
fixed=[p['id'] for p in D['prints'] if p['motion']=='fixed' and p['id']!='Base']
hits=[]
for i,n in enumerate(fixed):
 for k in fixed[i+1:]:
  if np.any(np.minimum(T[n].bounds[1],T[k].bounds[1])-np.maximum(T[n].bounds[0],T[k].bounds[0])<.001):continue
  v=(ss[n]^ss[k]).volume()
  if v>.01:hits.append([n,k,v])
tie=ss['Shared carry and sum carriage tie'];tie_hits=[]
for q in [-4.35,0,4.325]:
 for n in fixed+['Base']:
  v=(tie.translate([q,0,0])^ss[n]).volume()
  if v>.01:tie_hits.append([q,n,v])
(O/'Preliminary clearance checks.json').write_text(json.dumps(dict(fixed_part_collisions=hits,shared_tie_collisions=tie_hits),indent=2))
print('Gears',gear_count,'logic',len(rows),'gear pitches',len(pairs),'fixed collisions',hits,'tie',tie_hits,flush=True)
