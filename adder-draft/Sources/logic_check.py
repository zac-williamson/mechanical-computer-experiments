import json,itertools
from pathlib import Path
rows=[]
# Normal mux: selector 0 engages right data path (two meshes), 1 engages left (one).
# Mirroring reverses the selector's perceived bit and both local/global signal axes.
def normal(data,selector):return data^selector
def mirrored(data,global_selector):return data^global_selector^1
for a,b,su,ci in itertools.product(range(2),repeat=4):
 bp=normal(b,su);p=normal(a,bp)
 p_rail=p^1 # Three gear meshes: P -> carry selector.
 sum_data=p_rail^1 # Three more meshes to Sum data.
 cin_rail=ci^1 # One mesh to Sum selector, four more to carry data.
 result=mirrored(sum_data,cin_rail)
 carry_local_selector=p_rail^1
 cout=(cin_rail^1) if carry_local_selector else a
 expected=a+(b^su)+ci
 assert result==expected%2 and cout==expected//2
 rows.append(dict(A=a,B=b,SU=su,Cin=ci,Bprime=bp,P=p,carry_selector_global=p_rail,sum_data_global=sum_data,sum_selector_global=cin_rail,Sum=result,Cout=cout))
(Path(__file__).resolve().parents[1]/'Logic checks.json').write_text(json.dumps(dict(method='Gear parity and reversed actuator selector conventions, checked against integer full addition',cases=rows,passed=len(rows)),indent=2))
print(len(rows),'cases passed')
