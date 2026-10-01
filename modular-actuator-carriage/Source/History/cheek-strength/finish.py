from pathlib import Path
import os,json,runpy
R=Path(__file__).resolve().parents[1];O=R/'strength-candidate';os.environ['PLANAR_OUTPUT']=str(O)
s=(R/'band-access/check.py').read_text().replace("O=R/'band-candidate'","O=R/'strength-candidate'");s=s[:s.index('# Actual-mesh inspection')];exec(compile(s,'band insertion','exec'),{'__name__':'__main__','__file__':str(R/'band-access/check.py')})
runpy.run_path(str(R/'cheek-strength/audit.py'),run_name='__main__')
s=(R/'export_print_layout.py').read_text().replace("parent/'adapted'","parent/'strength-candidate'");exec(compile(s,'print layout','exec'),{'__name__':'__main__','__file__':str(R/'export_print_layout.py')})
# Matching separate print-oriented files for the parts changed in this revision.
import trimesh
for n,normal in [('Upper actuator cheek',[0,0,-1]),('Lower actuator cheek',[0,0,-1]),('Locking bolt guide',[0,1,0])]:
 t=trimesh.load(O/(n+'.stl'));t.apply_transform(trimesh.geometry.align_vectors(normal,[0,0,-1]));t.apply_translation(-t.bounds[0]);t.export(O/(n+' print.stl'))
runpy.run_path(str(R/'render_review.py'),run_name='__main__')
print('CHECKS COMPLETE',flush=True)
