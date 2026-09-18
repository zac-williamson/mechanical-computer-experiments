"""Check the continuous pivot-axle envelope and preserve lever contact faces."""
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
O=Path(__file__).resolve().parents[1]
D=json.loads((O/'Assembly manifest.json').read_text())
def solid(path):
 t=trimesh.load(path)
 assert t.is_watertight and t.is_winding_consistent,path
 return m.Manifold(m.Mesh64(np.asarray(t.vertices),np.asarray(t.faces,dtype=np.uint64)))
px,py=14.500924592586399,-3.261892187851191
# Test an envelope slightly smaller than the relief to avoid coincident faces.
envelope=m.Manifold.batch_hull([m.Manifold.cylinder(40,2.899,circular_segments=64,center=True).translate([px-q,py,36]) for q in [-4.35,4.325]])
canonical=solid(O.parent/'multiplexer/Left carriage half.stl')
relief=m.Manifold.batch_hull([m.Manifold.cylinder(40,2.9,circular_segments=64,center=True).translate([px-q,py,36]) for q in [-4.35,4.325]])
removed=canonical^relief
# Contact-pad swept envelope over the prescribed contact trace. If the
# removed corner misses this, the working stop faces have not been cut away.
lever=solid(O.parent/'multiplexer/Direct lever and band cleat.stl')
worst=0
trace=json.loads((O.parent/'multiplexer/Actuator poses.json').read_text())
for pose in trace:
 q,b=pose['q'],pose['b']
 posed=lever.translate([-px,-py,0]).rotate([0,0,float(b)]).translate([px-q,py,0])
 worst=max(worst,(removed^posed).volume())
results=[]
for ac in ['X','P','C']:
 t=solid(O/(ac+' Left carriage half.stl')).translate((-np.array(D['actors'][ac])).tolist())
 v=(t^envelope).volume();assert v<1e-5,(ac,v)
 results.append({'stage':ac,'swept_envelope_overlap_mm3':v})
assert worst<0.001,worst  # Mesh/contact tolerance, in cubic millimetres.
report={'stroke_mm':[-4.35,4.325],'radial_clearance_mm':0.5,'axle_envelope_radius_mm':2.4,'relief_radius_mm':2.9,'removed_material_mm3':removed.volume(),'maximum_removed_material_intersection_with_lever_mm3':worst,'lever_sweep_samples':len(trace),'results':results,'limits':'Geometric envelopes; no prediction of printed friction, deformation or wear.'}
(O/'Pivot clearance checks.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
