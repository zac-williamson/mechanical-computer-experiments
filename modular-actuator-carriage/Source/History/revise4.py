from pathlib import Path
p=Path('work/planar-module-restart/adapt.py');s=p.read_text()
s=s.replace("left-=keeper_void\nfor name", "left-=keeper_void\n# Rear posts join the original lower fork to the rotated upper mechanism.\n# The right post is behind the original roof, with a 0.3 mm separation.\nleft+=box([-15.6,35.2,2],[-9.5,43.8,36])+box([-15.6,24,2],[-9.5,43.8,6])\nright+=box([8,39.8,2],[15.6,43.8,36])+box([8,24,2],[15.6,43.8,6])+box([8,30.4,28],[15.6,43.8,36])\nfor name")
s=s.replace('rear-=front\nlever=','rear-=front\nrear=rear^box([-100,-100,-100],[40,100,100])\nlever=')
s=s.replace("if local in ['U022','reaction-stop-axle','reaction-retainer','pivot-stop-axle','pivot-retainer']:a=trimesh.transform_points(a,foldM)","if local in ['U022','reaction-stop-axle','reaction-retainer','pivot-stop-axle','pivot-retainer']:a=trimesh.transform_points(a,foldM)\n if local in ['reaction-stop-axle','pivot-stop-axle']:a[:,2]-=.6")
s=s.replace("if local not in ['U015'", "if local in ['O-left 5L axle','O-right 5L axle','selector-right-retainer']:continue\n if local not in ['U015'")
p.write_text(s)
p=Path('work/planar-module-restart/hardware.py');s=p.read_text();s+='''
# Four-stud output axles retain the planar clutch's inner stop positions.
def lego_axle(number,axis,c,name):
 lib=LDraw('/Applications/Studio 2.0/ldraw/parts/'+number+'.dat');a=lib.mesh()[0].reshape(-1,3)*.4;assert not lib.missing
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);t.apply_translation(-t.bounds.mean(0));oldaxis=int(np.argmax(t.extents));t.apply_transform(trimesh.geometry.align_vectors(np.eye(3)[oldaxis],np.eye(3)[axis]));t.apply_translation(c);native(name,t,axis=axis)
for x in [-20.2,20.2]:lego_axle('3705',0,[x,10.2,0],('Left' if x<0 else 'Right')+' output axle 4L')
ret=rawhardware('Memory — selector-right-retainer');ret.apply_translation(-ret.bounds.mean(0))
for x in [-26.6,26.6]:
 t=ret.copy();t.apply_translation([x,10.2,16]);native(('Left' if x<0 else 'Right')+' input axle retainer',t,axis=0)
''';p.write_text(s)
