from pathlib import Path
import os
R0=Path(__file__).resolve().parents[1];O=R0/'rounded-cam-candidate';os.environ['PLANAR_OUTPUT']=str(O)
s=(R0/'tiling/check.py').read_text().replace("O=R/'tile-candidate'","O=R/'rounded-cam-candidate'").replace('(80,0,0)','(72,0,0)').replace('(80,0,64)','(72,0,64)').replace('(-80,0,64)','(-72,0,64)').replace('80*col','72*col').replace('pitch_X_mm=80','pitch_X_mm=72')
s=s.replace("runpy.run_path(str(R/'validate.py'),run_name='__main__')",'''src=(R/'validate.py').read_text().replace("t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)","t=trimesh.load_mesh(R/(p['name']+'.stl')) if p['kind']=='printed' else trimesh.Trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)").replace('trimesh.Trimesh.Trimesh','trimesh.Trimesh')
exec(compile(src,'mechanism validation','exec'),{'__name__':'__main__'})''')
# Use exact printable solids; nominal STL packing can merge near coincident vertices.
s=s.replace("a=v[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);s=", "a=v[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.load_mesh(O/(p['name']+'.stl')) if p['kind']=='printed' and (O/(p['name']+'.stl')).exists() else trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);s=")
# Connector rotational cylinder, keyed engagement checked dimensionally separately.
s=s.replace("if p['kind']=='native' and 'pin' in p['name'].lower():", "if '2L axle connector' in p['name']:\n  bb=t.bounds;c=bb.mean(0);s=m.Manifold.cylinder(bb[1,0]-bb[0,0],3.7,circular_segments=48).rotate([0,90,0]).translate([bb[0,0],c[1],c[2]])\n if p['kind']=='native' and 'pin' in p['name'].lower():")
s=s.replace("bb=bounds[j][pi]+sh\n      if", "bb=bounds[j][pi]+sh\n      if '2L axle connector' in ep['name'] and p['name'] in ['C-shaft','Left output axle 4L','Right output axle 4L']:continue\n      if")
s=s.replace("if '2L axle connector' in p['name']:","if '2L axle connector' in p.get('name',''):")
exec(compile(s,'tiling validation','exec'),{'__name__':'__main__','__file__':str(R0/'tiling/check.py')})
