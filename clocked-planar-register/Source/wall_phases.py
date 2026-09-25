"""Assembly phase groups for added keyed shafts (degrees about +X)."""
def group(p):
 n=p['id']
 if n.startswith('Shared reverse'):return 'Shared reverse'
 if n=='POWER distribution 12L':return 'POWER internal'
 if n=='POWER local input 4L':return 'POWER local input'
 if 'baseline_id' in p:return 'fixed:'+n
 if n.startswith('Interstage'):return 'Interstage idler' if 'idler' in n else 'Master output'
 if n.startswith('POWER'):return 'POWER step idler' if p.get('drive')=='POWER' else 'POWER internal'
 if n.startswith('Q feedback') or n=='Feedback lower thrust bush':return 'Feedback step idler' if p.get('drive')=='Q' else 'Q inverted'
 if n.startswith('Control CLK'):return 'CLK incoming' if p.get('drive')=='CLK' else 'CLK worm'
 return n
