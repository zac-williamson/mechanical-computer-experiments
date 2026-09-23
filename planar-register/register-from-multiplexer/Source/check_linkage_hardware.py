from pathlib import Path
# Reuse the explicit assembly joint ownership inventory, not a blanket pin exclusion.
S=Path(__file__).resolve().parent
src=(S/'hardware_check.py').read_text().split('seen={};')[0].replace("O=R/'Assembly'","O=R/'Linkage development'")
exec(compile(src,str(S/'hardware_check.py'),'exec'))
changed|={'Write — Right carriage bearing support','Lock crosshead','Lock rocker','Write cam','Rear linkage frame','Upper cam rail'}
for h in hm:
 n=h['id']
 owners=[]
 if n.startswith('Write — Carriage support pin'):owners=['Write — Right carriage bearing support']
 if n=='Rocker pivot axle':owners=['Rear linkage frame'] # native stop shoulder seats on rear cheek; no shaft interference allowed by bore dimensions
 if n.startswith('Crosshead pin'):owners=['Lock crosshead','Memory — Lock bolt']
 if n.startswith('Cam attachment pin'):owners=['Write cam','Write — Right carriage bearing support']
 if n.startswith('Linkage mount pin'):owners=['Rear linkage frame','Common two-core baseboard']
 if n.startswith('Cam rail pin'):owners=['Rear linkage frame','Upper cam rail']
 allowed.update((p,n) for p in owners)
def transform(p,qm,qe,b):
 T=np.eye(4);mo=p['motion'];bank=p['bank'];q=qm if bank=='Memory' else qe
 if mo in ['carriage','worm']:T[0,3]=q if bank=='Memory' else -q
 if mo=='clutch-ring':T[0,3]=(1 if bank=='Memory' else -1)*np.sign(q)*max(abs(q)-.4,0)
 if mo=='write-cam':T[0,3]=-qe
 if mo=='bolt':T[2,3]=b
 if bank=='Linkage' and mo=='rocker':T=trimesh.transformations.rotation_matrix(np.arcsin((5.8-15+20*np.clip((qe-.9)/1.5,0,1))/50),[0,1,0],[50,0,2.7])
 return T
seen={}
for qe in np.linspace(-4.6,4.6,47):
 for qm in [-4.3,4.3]:
  b=min(0,float(-15+20*np.clip((qe-.9)/1.5,0,1)))
  for p in meta:cp.set_transform(p['id'],transform(p,qm,qe,b))
  for p in hm:ch.set_transform(p['id'],transform(p,qm,qe,b))
  _,pairs=cp.in_collision_other(ch,return_names=True)
  for a,c in pairs:
   if a not in changed and c not in new_hardware:continue
   if (a,c) in allowed:continue
   seen.setdefault((a,c),dict(printed=a,hardware=c,memory_q=qm,write_q=float(qe)))
report=dict(method='Native mesh contacts over 94 prescribed write-transition poses; named friction-pin joints excluded.',unexpected_contacts=list(seen.values()),limitations=['Gear phases fixed; not a full rotating-hardware swept-volume proof.','LEGO 6538c joiner is a conservative envelope; two shorter linkage axles have approximate end details.','Tangency can appear as contact. No load or compliance simulation.'])
(O/'Hardware contacts.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
