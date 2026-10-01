from pathlib import Path
import trimesh,json,numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
R=Path(__file__).resolve().parents[1];O=R/'rod-candidate';placed=[];rodmeshes=[];locations=[(0,0),(0,16),(0,40),(71,40)]
for n,xy in zip(['Carriage control rod','Lock control rod','Left bearing wall','Right bearing wall'],locations):
 t=trimesh.load(O/(n+' print.stl'));t.apply_translation([*xy,0]);placed.append(t)
 if 'rod' in n:rodmeshes.append((n,t.copy()))
layout=trimesh.util.concatenate(placed);layout.export(O/'Rods and guides print layout.stl');t=trimesh.load(O/'Rods and guides print layout.stl');assert t.is_watertight and len(t.split())==4
(O/'Rods and guides layout checks.json').write_text(json.dumps(dict(watertight=True,solids=4,dimensions_mm=t.extents.tolist()),indent=2))
fig=plt.figure(figsize=(12,5),facecolor='#e5ebe7');ax=fig.add_subplot(111,projection='3d');ax.set_facecolor('#e5ebe7')
for (name,t),color in zip(rodmeshes,['#20869b','#c56c33']):ax.add_collection3d(Poly3DCollection(t.triangles,facecolors=color,edgecolor='none',shade=True))
ax.set(xlim=(0,96),ylim=(0,29),zlim=(0,16),xlabel='Print X',ylabel='Print Y',zlabel='Print Z');ax.set_box_aspect((96,29,16));ax.view_init(elev=32,azim=-62);fig.suptitle('Side-print orientations: both ends on the bed · cam faces upward',fontsize=14);fig.tight_layout();fig.savefig(O/'Rod print orientations.png',dpi=140);print('Four-part replacement layout saved',flush=True)
