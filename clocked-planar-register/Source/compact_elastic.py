"""Geometric elastic loops; no assumed material force law or preload proof."""
import math
import numpy as np

def actuator_band(bank,beta):
 x,z,sign={'master':(0,0,1),'slave':(106,0,1),'write':(-76,-16,-1),'clock':(40,-16,-1)}[bank]
 pivot=np.array([13.192323604,32.128448698]);a=np.array({'write':[30.192323604,29.295115365],'clock':[29.192323604,29.461782031]}.get(bank,[31.192323604,29.128448698]))
 t=math.radians(beta);rot=np.array([[math.cos(t),math.sin(t)],[-math.sin(t),math.cos(t)]])
 b=pivot+rot@(np.array([22.192323604,30.628448698])-pivot)
 direction=math.atan2(b[1]-a[1],b[0]-a[0]);outer=[];inner=[]
 for end,c in enumerate([a,b]):
  for j in range(25):
   theta=direction+(math.pi/2 if end==0 else -math.pi/2)+j*math.pi/24
   radial=np.array([math.cos(theta),math.sin(theta)])
   outer.append(c+3.2*radial);inner.append(c+2*radial)
 out=[]
 def point(p,y):return [x+sign*p[0],y,z+sign*p[1]]
 def quad(a,b,c,d):out.extend([a,b,c,a,c,d])
 for i in range(len(outer)):
  j=(i+1)%len(outer)
  for y in [9.6,10.8]:quad(point(outer[i],y),point(outer[j],y),point(inner[j],y),point(inner[i],y))
  for path in [outer,inner]:quad(point(path[i],9.6),point(path[j],9.6),point(path[j],10.8),point(path[i],10.8))
 return np.array(out)


def fork_band(bank,xx,rail,lag):
 """Loop between bar and floating fork; actual band force must be measured."""
 gx,bias=(-60,1) if bank=='master_gate' else (60,-1)
 ax=-12 if xx<0 else 11;az=7.8
 a=np.array([gx+ax+rail+lag,az]);b=np.array([gx+ax+rail+6.5*bias,az])
 if a[0]>b[0]:a,b=b,a
 path=[]
 for end,c in enumerate([a,b]):
  for j in range(25):
   theta=(math.pi/2 if end==0 else -math.pi/2)+j*math.pi/24
   path.append((c+1.2*np.array([math.cos(theta),math.sin(theta)]),c+2.1*np.array([math.cos(theta),math.sin(theta)])))
 out=[]
 def pt(v,y):return [v[0],y,v[1]]
 def quad(a,b,c,d):out.extend([a,b,c,a,c,d])
 for i in range(len(path)):
  j=(i+1)%len(path);inn,outer=path[i];innj,outerj=path[j]
  for y in [19.15,19.95]:quad(pt(inn,y),pt(innj,y),pt(outerj,y),pt(outer,y))
  for a,b in [(inn,innj),(outer,outerj)]:quad(pt(a,19.15),pt(b,19.15),pt(b,19.95),pt(a,19.95))
 return np.array(out)
