from pathlib import Path
import json,csv,hashlib,numpy as np,trimesh,manifold3d as m
O=Path(__file__).resolve().parents[1]
P=json.loads((O/'parts.json').read_text());V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
rows=[];solids={}
for p in P:
 a=V[p['offset']//3:p['offset']//3+p['vertices']]
 row=dict(part=p['id'],kind=p['kind'],module=p['module'],lego_part=p.get('lego_part'),motion=p.get('motion','fixed'))
 if p['kind']=='printed':
  mesh=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
  solids[p['id']]=m.Manifold(m.Mesh64(mesh.vertices.astype(float),mesh.faces.astype(np.uint64)))
  opts=[]
  for ax in range(3):
   for sign in [-1,1]:
    n=np.eye(3)[ax]*sign;bed=(a@n).min();bad=(mesh.face_normals@n<-.70712)&(mesh.triangles_center@n>bed+.001)
    opts.append(dict(up='XYZ'[ax]+('+' if sign==1 else '-'),unsupported_area_mm2=float(mesh.area_faces[bad].sum())))
  best=min(opts,key=lambda x:x['unsupported_area_mm2'])
  row.update(watertight=bool(mesh.is_watertight),connected_solids=len(mesh.split()),dimensions_mm=mesh.extents.tolist(),best_six_axis_screen=best,orientations=opts)
 rows.append(row)
def box(lo,hi):return m.Manifold.cube(np.array(hi)-lo).translate(lo)
guides=[]
for mod,key,x,y,zs in [('bit','CLOCK',-116,10,[-22,21.5]),('bit','WRITE',-128,26,[-12,32]),('control','CLOCK',-116,10,[-132,-72]),('control','WRITE',-128,26,[-204,-140,-72])]:
 for z in zs:
  ring=box([x-6,y-2,z-.1],[x+6,y+8,z+.1])-box([x-3.4,y-.4,z-.2],[x+3.4,y+6.4,z+.2])
  owners=[]
  for n,s in solids.items():
   if 'fixture' in n or 'chassis' in n:
    frac=(ring^s).volume()/ring.volume()
    if frac>.01:owners.append(dict(part=n,ring_fraction=frac))
  guides.append(dict(module=mod,rod=key,z=z,opening_mm=[6.8,6.8],owners=owners))
report=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),scope='All 323 components inventoried; all 66 printed meshes screened in six cardinal orientations. Surface-normal screen only, not slicing, strength, or assembly-path certification.',parts=rows,guide_membership=guides)
(O/'Manufacturing audit/Component audit.json').write_text(json.dumps(report,indent=2))
with (O/'Manufacturing audit/Component inventory.csv').open('w') as f:
 w=csv.writer(f);w.writerow(['Part','Kind','Module','LEGO part','Motion','Watertight','Solids','Dimensions mm','Best cardinal up','Downward area over 45deg mm2'])
 for r in rows:
  b=r.get('best_six_axis_screen',{});w.writerow([r.get(k,'') for k in ['part','kind','module','lego_part','motion','watertight','connected_solids','dimensions_mm']]+[b.get('up',''),b.get('unsupported_area_mm2','')])
print('PRINTED',sum(r['kind']=='printed' for r in rows),'CARDINAL SCREEN PASSES',sum(r.get('best_six_axis_screen',{}).get('unsupported_area_mm2',1e9)<.01 for r in rows))
print('GUIDES',json.dumps(guides,indent=2))
print('PRINT FLAGS',[(r['part'],r['best_six_axis_screen']) for r in rows if r['kind']=='printed' and r['best_six_axis_screen']['unsupported_area_mm2']>.01])
