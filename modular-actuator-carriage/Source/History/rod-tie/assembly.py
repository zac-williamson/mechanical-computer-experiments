from pathlib import Path
R0=Path(__file__).resolve().parents[1]
s=(R0/'open-carriage/assembly.py').read_text().replace("O=R0/'open-carriage-candidate'","O=R0/'rod-tie-candidate'")
s=s.replace("exec(compile((R0/'validate.py').read_text().split('printed_hits=[]')[0],'load','exec'))", "source=(R0/'validate.py').read_text().split('printed_hits=[]')[0].replace(\"t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)\",\"t=trimesh.load_mesh(R/(p['name']+'.stl')) if p['kind']=='printed' else trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)\");exec(compile(source,'load','exec'))")
a=s.index('# Preload joining pins');b=s.index("trial('Right half",a);s=s[:a]+s[b:]
a=s.index("for n in ['Carriage joining pin");b=s.index("trial('Left half",a);s=s[:a]+s[b:]
s=s.replace("['Carriage body','Carriage joining pin 0.5','Carriage joining pin 32','U015']","['Carriage body','U015']").replace('onto right half and preloaded pins','toward positioned right half')
s += "\nassert passed\n"
exec(compile(s,'assembly','exec'),{'__file__':str(R0/'open-carriage/assembly.py')})
