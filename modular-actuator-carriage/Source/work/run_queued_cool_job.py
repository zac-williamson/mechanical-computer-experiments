"""Run the single-worker geometry guard at a 10% scheduling duty cycle."""
import os,sys,time,signal,subprocess,fcntl
from pathlib import Path
root=Path(__file__).resolve().parents[1]
for key,folder in [('MPLCONFIGDIR','matplotlib-cache'),('XDG_CACHE_HOME','render-cache')]:
 cache=root/'work'/folder;cache.mkdir(exist_ok=True);os.environ.setdefault(key,str(cache))
lock=open(root/'work/.cool-job.lock','w')
print('Waiting for the shared geometry slot.',flush=True)
try:fcntl.flock(lock,fcntl.LOCK_EX)
except BlockingIOError:raise SystemExit('A throttled job is already running.')
if len(sys.argv)<2:raise SystemExit('Usage: run_cool_job.py script.py [args]')
guard=root/'work/geometry_guard.py'
p=subprocess.Popen([sys.executable,str(guard),*sys.argv[1:]])
print('CPU throttle: 50 ms running, 450 ms paused; one worker, low priority.',flush=True)
def stop(*_):raise KeyboardInterrupt
signal.signal(signal.SIGTERM,stop)
try:
 while p.poll() is None:
  time.sleep(.05)
  if p.poll() is not None:break
  os.kill(p.pid,signal.SIGSTOP)
  time.sleep(.45)
  if p.poll() is None:os.kill(p.pid,signal.SIGCONT)
except (KeyboardInterrupt,SystemExit):
 if p.poll() is None:
  os.kill(p.pid,signal.SIGCONT);p.terminate()
  try:p.wait(timeout=3)
  except subprocess.TimeoutExpired:p.kill()
 raise
finally:
 if p.poll() is None:
  os.kill(p.pid,signal.SIGCONT);p.terminate();p.wait()
sys.exit(p.returncode)
