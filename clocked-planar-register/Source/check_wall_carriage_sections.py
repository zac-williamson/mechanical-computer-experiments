"""Verify adopted local rail, jaw and anti-rocking sections in actual output solids."""
from pathlib import Path
import json,hashlib
import numpy as np,trimesh,manifold3d as m
O=Path(__file__).resolve().parents[1]/'Wall register'
P=json.loads((O/'parts.json').read_text());V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
def solid(a):
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
def box(a,b):return m.Manifold.cube(np.array(b)-a).translate(a)
records=[]
for bank,x,z,flip,rot,shift in [('master',0,0,False,np.eye(3),np.zeros(3)),('slave',106,0,False,np.eye(3),np.zeros(3)),('clock',40,-16,True,np.array([[0,0,1],[0,1,0],[-1,0,0]]),np.array([-100,-4,-64])),('write',-76,-16,True,np.array([[0,0,1],[0,1,0],[-1,0,0]]),np.array([-104,-12,-260]))]:
 sign=np.array([-1,1,-1]) if flip else np.ones(3)
 fixed=m.Manifold()
 for p in P:
  if p['kind']!='printed' or p.get('motion','fixed')!='fixed' or p['module']!=('bit' if bank in ['master','slave'] else 'control'):continue
  a=V[p['offset']//3:p['offset']//3+p['vertices']];local=((a-shift)@rot-np.array([x,0,z]))*sign
  # Only nearby fixed solids are relevant; avoid one giant assembly boolean.
  if np.all(local.max(0)>[-21,15,4]) and np.all(local.min(0)<[21,38,17]):fixed+=solid(local)
 for suffix,xa,xb in [('Carriage fork and roof',-15.6,-8),('Right carriage bearing support',8,15.6)]:
  candidates=[p for p in P if p.get('baseline_id')==bank+' '+suffix and p['kind']=='printed']
  # WRITE support was separated into the shoe and direct rod. Their union is
  # checked as the same joined carriage component.
  body=m.Manifold()
  for p in candidates:
   a=V[p['offset']//3:p['offset']//3+p['vertices']];body+=solid(((a-shift)@rot-np.array([x,0,z]))*sign)
  if body.is_empty():raise ValueError((bank,suffix,'missing carriage'))
  rear=29.7 if bank=="clock" else 32.5
  sections=[]
  for lo,hi in [(5.3,9.35),(11.85,15.9)]:
   pad_x=max(xa,-12) if bank=="slave" and suffix=="Carriage fork and roof" and lo>10 else xa
   probe=box([pad_x+.05,rear-2,lo+.01],[xb-.05,rear-.01,hi-.01]);sections.append((probe-body).volume())
  # Complete 2 mm material around both re-entrant slot corners.
  for zz in [9.35,11.85]:
   probe=m.Manifold.cylinder(xb-xa-.02,2,circular_segments=64).rotate([0,90,0]).translate([xa+.01,16.65,zz]);probe-=box([xa,16.65,9.35],[xb,40,11.85]);sections.append((probe-body).volume())
  backing=[];backing_fractions=[]
  for q in np.linspace(-3.756,3.756,9):
   pad_x=max(xa,-12) if bank=="slave" and suffix=="Carriage fork and roof" else xa
   start=pad_x+q+.15
   gap=.3
   low,high=(5.5,8.1) if bank=="clock" else (12.0,15.8)
   probe=box([start,rear+gap+.001,low],[xb+q-.15,rear+gap+.2,high]);missing=(probe-fixed).volume();backing.append(missing);backing_fractions.append(1-missing/probe.volume())
  records.append(dict(bank=bank,half=suffix,missing_section_mm3=sections,missing_backing_mm3=backing,backing_material_fraction=backing_fractions,pass_check=max(sections)<.01 and min(backing_fractions)>.99))
r=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),parts=records,carriage_sections_pass=all(x['pass_check'] for x in records),scope=__doc__,nominal_rail_mm=2,nominal_rail_clearance_mm=.25,nominal_backing_gap_mm=.3)
(O/'Carriage section checks.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
