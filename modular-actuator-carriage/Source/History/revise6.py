from pathlib import Path
p=Path('work/planar-module-restart/adapt.py');s=p.read_text()
s=s.replace("upperleft=fold(rawleft^uppermask);upperright=fold(rawright^uppermask)","upperleft=fold(rawleft^uppermask)^box([-100,-100,9.5],[100,100,100]);upperright=fold(rawright^uppermask)^box([-100,-100,9.5],[100,100,100])")
s=s.replace("s-=cy(2.65,-20,20,0,[0,10.2,16])\n locals()[name]=s", "s-=cy(2.65,-20,20,0,[0,10.2,16])\n for cx,yy in [(0,18.2),(13.192323604,26.328448698)]:\n  head=cy(3.4,5.8,7.8,2,[cx-3.85,yy,0])+cy(3.4,5.8,7.8,2,[cx+3.85,yy,0])+box([cx-3.85,yy-3.4,5.8],[cx+3.85,yy+3.4,7.8])\n  head-=cy(7.6,-100,100,0,[0,10.2,0])\n  s-=head\n locals()[name]=s")
s=s.replace("if local in ['reaction-stop-axle','pivot-stop-axle']:a[:,2]-=.6", "if local in ['reaction-stop-axle','pivot-stop-axle']:a[:,2]-=.6\n if local in ['L069','L105']:a[:,0]+=(26.6 if local=='L069' else -26.6)-a[:,0].mean()")
s=s.replace("def add(n,s,motion='fixed',bed='rear',color=None):\n t=mesh(s);", "def add(n,s,motion='fixed',bed='rear',color=None):\n from clean_print_mesh import clean\n t=mesh(s.simplify(.0001));t.vertices=t.vertices.astype('<f4');t=clean(t);t.fix_normals();assert t.is_watertight,n\n assert abs(t.volume-s.volume())<.05,(n,'mesh cleanup volume change')\n s=solid(t);assert s.status()==m.Error.NoError,n\n ")
s=s.replace('parts=[];provenance=[]',"sys.path.insert(0,str(ROOT/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source'))\nparts=[];provenance=[]")
p.write_text(s)
