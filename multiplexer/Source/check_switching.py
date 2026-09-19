from contact import *
h=.25;dq=4*np.pi/180*h # pitch radius 4 mm; one gear step exchanges this travel
cache={}
def stopmask(qi):
 if qi not in cache:cache[qi]=area(intersection(ls,affinity.translate(st,4.325+qi*dq,0)))<.0001
 return cache[qi]
def branch(row,b):
 inds=np.flatnonzero(row)
 if not len(inds):return None
 j=int(inds[np.argmin(abs(inds-b))])
 if abs(j-b)>20:return None
 lo=j;hi=j
 while lo>0 and row[lo-1]:lo-=1
 while hi<len(row)-1 and row[hi+1]:hi+=1
 return int(np.clip(np.argmin(abs(betas)),lo,hi))
qi=0;gi=0;ui=0;b=branch(free[0]&stopmask(qi),400);rows=[];fail=[]
for segment,d,n in [('Right overrun',1,360),('Reverse: drive left',-1,1100),('Reverse: drive right',1,1100),('Reverse: drive left',-1,1100)]:
 startq=qi
 for step in range(n):
  ui+=d;found=False
  for advance in range(9):
   nq=qi+d*advance;q=4.325+nq*dq
   if not -4.35<=q<=4.325:continue
   ng=ui-nq
   nb=branch(free[ng%len(gammas)]&stopmask(nq),b)
   if nb is None:continue
   moved=nq!=qi
   qi,gi,b=nq,ng,nb;found=True;break
  if not found:fail.append(dict(segment=segment,step=step,q=4.325+qi*dq,g=gi*h,b=float(betas[b])));break
  rows.append(dict(q=round(4.325+qi*dq,6),g=gi*h,b=round(float(betas[b]),4),w=ui*h*8,d=d,moving=moved,segment=segment))
 print(segment,'q',4.325+startq*dq,'->',4.325+qi*dq,'fail',fail[-1:] if fail else [],flush=True)
 if fail:break
assert not fail,fail
assert min(r['q'] for r in rows)<-3.5 and max(r['q'] for r in rows)>3.5
print('Switching calculation completed in both directions.')
