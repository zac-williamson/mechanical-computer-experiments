"""Independent CAD study using the preserved planar bolt and guide.

Exports are development geometry, not a print-ready clocked register.
No source generator is imported: the planar generators have publishing side effects.
"""
from pathlib import Path
import json, hashlib
import numpy as np
import trimesh
import manifold3d as m
from shapely.geometry import Point, Polygon
from sequence import pose, RAIL_LIMITS, RAMP_WIDTH, LIFT, RADIUS, SLOPE

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT.parent/'planar-register/register-from-multiplexer/Planar register'
OUT = ROOT/'Cam study'
BX = -5.05
SPACING = 160.0


def solid(t):
    return m.Manifold(m.Mesh64(np.array(t.vertices, copy=True),
                             np.array(t.faces, dtype=np.uint64, copy=True)))


def mesh(s):
    t = s.to_mesh64()
    return trimesh.Trimesh(np.array(t.vert_properties)[:, :3],
                           np.array(t.tri_verts), process=True)


def xz(poly, a, b):
    return m.CrossSection([np.array(poly)]).extrude(b-a).transform(
        [[1,0,0,0], [0,0,1,a], [0,1,0,0]])


def build():
    OUT.mkdir(exist_ok=True)
    used = {}
    def source(name):
        path = BASE/(name+'.stl')
        used[str(path.relative_to(ROOT.parent))] = hashlib.sha256(path.read_bytes()).hexdigest()
        return solid(trimesh.load(path))
    guide = source('Detachable bolt guide')
    bolt = source('Direct lock bolt')
    roof = source('Memory — Carriage fork and roof')
    # Full-width spine links two mirrored ramp surfaces. Positive gate grooves,
    # guides, joints and clock drive are intentionally not pretended complete.
    xmin, xmax = BX-27, BX+SPACING+27
    xs = set([xmin, xmax])
    high = 53.3 + LIFT - RADIUS
    for b, sign in [(BX, 1), (BX+SPACING, -1)]:
        crest = b + sign*(1+RAMP_WIDTH)
        xs.update([b-24, b+24, crest, crest-sign*(high-49.7)/SLOPE])
    xs = sorted(x for x in xs if xmin <= x <= xmax)
    def top(x):
        heights = [49.7]
        for b, sign in [(BX, 1), (BX+SPACING, -1)]:
            if abs(x-b) <= 24:
                crest = b + sign*(1+RAMP_WIDTH)
                heights.append(min(high, high + SLOPE*sign*(x-crest)))
        return max(heights)
    poly = [[xmin,46.5],[xmax,46.5]] + [[x,top(x)] for x in reversed(xs)]
    rail = xz(poly,25.6,31)
    parts = [('Shared cam rail — unfinished', rail)]
    for stage, dx in [('Master',0),('Slave',SPACING)]:
        for name, part in [('bolt',bolt),('guide',guide),('carriage',roof)]:
            parts.append((stage+' '+name, part.translate([dx,0,0])))
    for name, part in parts:
        t = mesh(part)
        assert t.is_watertight and t.volume > 0, name
        t.export(OUT/(name+'.stl'))

    # Actual solid intersections against the unchanged guide, bolt and roof.
    failures = []
    maxima = {}
    contact_errors = []
    profile = Polygon(poly)
    count = 401
    for s in np.linspace(*RAIL_LIMITS,count):
        moving = rail.translate([float(s),0,0])
        state = pose(float(s))
        for stage, dx in [('master',0),('slave',SPACING)]:
            centre = Point(BX+dx-float(s),53.3+state[stage+'_lift'])
            contact_errors.append(abs(profile.distance(centre)-RADIUS))
            vol = (bolt.translate([0,0,state[stage+'_lift']]) ^ guide).volume()
            maxima[stage+'/bolt-guide'] = max(maxima.get(stage+'/bolt-guide',0),vol)
            if vol > .0001:
                failures.append(dict(rail=float(s),pair=stage+'/bolt-guide',volume_mm3=vol))
            for q in [-3.755874,3.749041]:
                vol = (bolt.translate([0,0,state[stage+'_lift']]) ^ roof.translate([q,0,0])).volume()
                key = stage+'/bolt-locked-carriage'
                maxima[key] = max(maxima.get(key,0),vol)
                if vol > .0001:
                    failures.append(dict(rail=float(s),carriage=q,pair=key,volume_mm3=vol))
            for name, target in [('guide',guide.translate([dx,0,0])),
                ('bolt',bolt.translate([dx,0,state[stage+'_lift']]))]:
                vol = (moving ^ target).volume()
                key = stage+'/'+name
                maxima[key] = max(maxima.get(key,0),vol)
                if vol > .0001:
                    failures.append(dict(rail=float(s),pair=key,volume_mm3=vol))
            # Three carriage positions here are not a substitute for full swept
            # gear/lever validation; rail is wholly above the carriage roof.
            for q in [-3.755874,0,3.749041]:
                vol = (moving ^ roof.translate([dx+q,0,0])).volume()
                key = stage+'/carriage'
                maxima[key] = max(maxima.get(key,0),vol)
                if vol > .0001:
                    failures.append(dict(rail=float(s),carriage=q,pair=key,volume_mm3=vol))
    report = dict(scope='Cam rail against actual planar bolt, guide and carriage solids only',
                  rail_samples=count, max_intersections_mm3=maxima,
                  max_analytic_roller_contact_error_mm=max(contact_errors),
                  failures=failures, print_ready=False,
                  missing=['Rail guides and pin-jointed split', 'Clock lever and its supports',
                           'Positive clutch fork drives', 'Connected gear routing',
                           'Whole frame and all-parts sweep'])
    (ROOT/'Cam geometry checks.json').write_text(json.dumps(report,indent=2)+'\n')
    (ROOT/'Planar component provenance.json').write_text(json.dumps(used,indent=2)+'\n')
    for path, digest in used.items():
        assert hashlib.sha256((ROOT.parent/path).read_bytes()).hexdigest() == digest
    print(json.dumps({k:v for k,v in report.items() if k!='failures'},indent=2))
    print('Failed pair/sample checks:',len(failures))
    assert not failures, 'Sampled solid intersections found; inspect report'
    assert max(contact_errors) < 1e-6, 'Cam does not support prescribed follower trajectory'
    return parts, report


if __name__ == '__main__':
    build()
