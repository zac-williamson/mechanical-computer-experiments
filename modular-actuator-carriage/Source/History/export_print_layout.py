from pathlib import Path
import json,trimesh,runpy
clean=runpy.run_path(str(Path(__file__).resolve().parents[2]/"latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source/clean_print_mesh.py"))["clean"]
r=Path(__file__).resolve().parent/'adapted'
normals={'left':[-1,0,0],'right':[1,0,0],'front':[0,-1,0],'rear':[0,1,0],'bottom':[0,0,-1]}
parts=[]
checks=json.loads((r/'Bed orientation checks.json').read_text())
for p in checks:
 t=trimesh.load(r/(p['part']+'.stl'),force='mesh')
 assert t.is_watertight
 t.apply_transform(trimesh.geometry.align_vectors(normals[p['bed_face']],[0,0,-1]))
 t.apply_translation(-t.bounds[0]);t.apply_translation(p['bounds'][0]);parts.append(t)
m=trimesh.util.concatenate(parts);dest=r/'Print layout.stl';m.export(dest)
v=clean(trimesh.load(dest,force='mesh'));v.export(dest);v=trimesh.load(dest,force='mesh')
print('Export watertight:',v.is_watertight,'solids:',len(v.split()),'expected:',len(parts),flush=True)
assert v.is_watertight and len(v.split())==len(parts)
report={'parts':[p['part'] for p in checks],'dimensions_mm':v.extents.tolist(),'watertight':bool(v.is_watertight),'solid_count':len(parts),'note':'One module, printed parts only. Uses audited bed orientations; local supports and physical fit testing remain required. Frame joining bridges are in the separate connector layout.'}
(r/'Print layout checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
