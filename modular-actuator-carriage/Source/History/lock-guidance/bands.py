from pathlib import Path
R0=Path(__file__).resolve().parents[1]
s=(R0/'band-access/check.py').read_text().split('# Actual-mesh inspection images')[0].replace("O=R/'band-candidate'","O=R/'lock-guidance-candidate'")
s=s.replace("prefix=s[:s.index('trace=json.loads')]", "prefix=s[:s.index('trace=json.loads')].replace(\"t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)\",\"t=trimesh.load_mesh(R/(p['name']+'.stl')) if p['kind']=='printed' else trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)\")")
exec(compile(s,'band checks','exec'),{'__file__':str(R0/'band-access/check.py')})
