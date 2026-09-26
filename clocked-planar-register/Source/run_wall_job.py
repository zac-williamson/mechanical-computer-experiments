"""Run one geometry job, with one TBB/BLAS worker and a 1.2 GiB RSS guard.

The global_control layout follows oneTBB global_control.h. Fail closed if the
bundled Manifold library does not expose the expected API. No other jobs spawn.
"""
import os,sys,ctypes,fcntl,resource,threading,time,runpy
from pathlib import Path
for name in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','VECLIB_MAXIMUM_THREADS','NUMEXPR_NUM_THREADS']:
 os.environ[name]='1'
lock=open(Path(__file__).resolve().parents[1]/'.wall-job.lock','w')
try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:raise SystemExit('Another wall geometry job is running; refusing parallel execution.')
os.nice(15)
import manifold3d
lib=ctypes.CDLL(manifold3d.__file__)
class Control(ctypes.Structure):
 _fields_=[('value',ctypes.c_size_t),('reserved',ctypes.c_ssize_t),('parameter',ctypes.c_int)]
control=Control(1,0,0)
create=getattr(lib,'_ZN3tbb6detail2r16createERNS0_2d114global_controlE');create.argtypes=[ctypes.POINTER(Control)];create.restype=None
active=getattr(lib,'_ZN3tbb6detail2r127global_control_active_valueEi');active.argtypes=[ctypes.c_int];active.restype=ctypes.c_size_t
create(ctypes.byref(control));assert active(0)==1,'TBB worker limit was not applied'
def guard():
 while True:
  rss=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
  if sys.platform!='darwin':rss*=1024
  if rss>1.2*1024**3:
   print('Stopped geometry job: resident memory exceeded 1.2 GiB.',file=sys.stderr,flush=True);os._exit(75)
  time.sleep(.25)
threading.Thread(target=guard,daemon=True).start()
print('Geometry job: single TBB/BLAS worker, low priority, 1.2 GiB memory guard.',flush=True)
script=Path(sys.argv[1]).resolve();sys.argv=sys.argv[1:];sys.path.insert(0,str(script.parent));runpy.run_path(str(script),run_name='__main__')
