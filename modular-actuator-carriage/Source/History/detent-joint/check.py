from pathlib import Path
import os,runpy,json
R=Path(__file__).resolve().parents[1];O=R/'detent-candidate';os.environ['PLANAR_OUTPUT']=str(O)
s=(R/'carriage-print-split/check_assembly.py').read_text().replace("O=R/'split-candidate'","O=R/'detent-candidate'");exec(compile(s,'assembly','exec'),{'__name__':'__main__','__file__':str(R/'carriage-print-split/check_assembly.py')})
runpy.run_path(str(R/'detent-joint/finish.py'),run_name='__main__')
