"""Read explicitly declared detachable interfaces added by manufacturing fixes."""
import json

def extra_connections(output):
 result=[]
 for name in ['Rod guide assembly schedule.json','Print face additions schedule.json']:
  path=output/name
  if not path.exists():continue
  data=json.loads(path.read_text())
  result.extend(data.get('connections',[]))
 return result
