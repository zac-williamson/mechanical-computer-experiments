"""All printed-solid intersections at the reference pose, with no pair exclusions.

This is a static necessary check, not a moving-clearance or operation proof.
Invalid meshes are reported, never silently skipped as clear.
"""
from pathlib import Path
import hashlib
import json
import numpy as np
import trimesh
import manifold3d as m

root = Path(__file__).resolve().parents[1] / 'Compact layout'
geometry_digest=hashlib.sha256((root/'geometry.npz').read_bytes()).hexdigest()
parts = json.loads((root / 'parts.json').read_text())
v = np.load(root / 'geometry.npz')['vertices'].reshape(-1, 3)
solids, invalid = [], []
for p in parts:
    if p['kind'] != 'printed':
        continue
    a = v[p['offset']//3:p['offset']//3+p['vertices']]
    mesh = trimesh.Trimesh(a, np.arange(len(a)).reshape(-1, 3), process=True)
    s = m.Manifold(m.Mesh64(mesh.vertices.astype(float), mesh.faces.astype(np.uint64)))
    if not mesh.is_watertight or s.status() != m.Error.NoError:
        invalid.append(p['id'])
    else:
        solids.append((p['id'], mesh.bounds, s))
hits = []
tested = 0
for i, (name, bounds, s) in enumerate(solids):
    for name2, bounds2, s2 in solids[i+1:]:
        tested += 1
        if np.any(np.minimum(bounds[1], bounds2[1])-np.maximum(bounds[0], bounds2[0]) <= 0):
            continue
        overlap = s ^ s2
        volume = overlap.volume()
        if volume > 0.001:
            q = overlap.to_mesh64().vert_properties[:, :3]
            hits.append(dict(a=name, b=name2, volume_mm3=volume,
                             bounds_mm=[q.min(0).tolist(), q.max(0).tolist()]))
hits.sort(key=lambda x: -x['volume_mm3'])
report = dict(scope='All printed pairs at static reference pose only',
              geometry_sha256=geometry_digest,
              pairs_tested=tested, invalid_solids=invalid, intersections=hits,
              static_printed_pass=not hits and not invalid,
              operation_pass=False)
(root/'Static printed intersections.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
