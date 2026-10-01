"""Strengthen the existing two carriage solids; all coordinates are assembly coordinates."""
baseline_parts={'left':left,'right':right}
# Continue the flat outer bearing faces to the wider pin-boss plane.
for name,sign in [('left',-1),('right',1)]:
 s=locals()[name];a,b=(-15.6001,-15.29) if sign<0 else (15.29,15.6001)
 face=s^box([a,-100,-100],[b,100,100])
 for distance in [.2,.4,.6,.65]:s+=face.translate([sign*distance,0,0])
 locals()[name]=s
# A supported front wall replaces the 1 mm sheet, without entering the worm space.
left+=box([-8,1.55,10.5],[7.8,4.7,24.65])
# Reinforce the central clutch bridge outside the retained rotating clutch envelope.
bridge=box([-8,4.7,7.2],[7.8,12.7,11.2])
bridge-=cy(7.6,-8.1,7.9,0,[0,10.2,0])+cy(5.3,-8.1,7.9,0,[0,10.2,16])
left+=bridge
# Remove only the unused rear fringe outside the clutch-ring clearance envelope.
fringe=box([-8,12.7,5.5],[7.8,15.5,11.3])-cy(7.4,-8.1,7.9,0,[0,10.2,0])
left-=fringe
for name,a,b in [('left',-16.25,-9.1),('right',9.1,16.25)]:
 # Join both bearing levels with a real web, preserving gear and axle clearances.
 web=box([a,1.55,5.5],[b,16.25,20])
 web+=cy(6,a,b,0,[0,10.2,16])
 web-=cy(9.15,a-.1,b+.1,0,[0,10.2,0])+cy(2.85,a-.1,b+.1,0,[0,10.2,16])
 locals()[name]+=web
# Grow the lever-support back wall from 2 to 3 mm along +Y.
left+=box([-15.6,39.3,11.6],[26.5995,40.5,20])
# Preserve a 0.3 mm gap to the other carriage half; its rear post remains 3 mm deep.
right-=box([7.99,39.79,6],[16.3,40.8,27])
# Enlarge the front and rear joining-pin surrounds.
left+=box([-16.25,19.4,-4.8],[7.8,30,7.2])+box([-16.25,29.4,26],[7.8,41.4,38])
right+=box([8,19.4,-4.8],[16.25,30,7.2])+box([8,29.4,26],[16.25,43.8,38])
right+=box([8,34.4,4],[16.25,43.8,9])+box([8,34.4,22],[16.25,43.8,27])
# Rebuild the locking pockets around the actual 3.9 x 4 mm bolt tip.
left-=box([-16.3,20.5,30.1],[4.1,31.5,40])
keeper_new=box([-16.25,20.5,30.1],[4.1,31.5,39.4])
for x in [-8.8,-1.3]:keeper_new-=box([x-2.35,23.5,35.8],[x+2.35,28.5,39.5])
left+=keeper_new
# 10 mm wide rod-pin bosses: 2.5 mm nominal bore ligaments before hardware relief.
for name,sign in [('left',-1),('right',1)]:
 a,b=(-16.25,-5.75) if sign<0 else (5.75,16.25)
 boss=box([a,14.4,26.75],[b,22.5,37.25])-clear
 locals()[name]+=boss
# End each actuator sleeve cleanly at its full-thickness section.
left-=box([-9.1,6,-8],[-8,18.2,22.1])-cy(7.6,-9.2,-7.9,0,[0,10.2,0])
right-=box([8,6,-8],[9.1,18.2,22.1])-cy(7.6,7.9,9.2,0,[0,10.2,0])
# Remove outboard gear-clearance fins. They are outside the clutch contact
# region; the load now passes through the broad upper frame and rear posts.
for name,a,b in [('left',-16.26,-9.0999),('right',9.0999,16.26)]:
 locals()[name]-=box([a,-1,-8],[b,18.2,9.15])
# Delete an unused short tail beyond the front joining block.
left-=box([-9.5,30,-8],[7.8,32.51,7.21])
# Open the rod-pin overrun behind its fully enclosed 8 mm grip length.
# A 3 mm roof and 2.75 mm outer side connect this boss to the rear frame.
right+=box([8,22.5,35.5],[16.25,30.4,38.5])
right-=box([7.9,22.5,22],[13.5,24.5,35.5])
# Re-cut the joints after reinforcement; no hidden duplicate or filled pin holes.
for name,sign in [('left',-1),('right',1)]:
 s=locals()[name]
 # Relieve only new support material against the measured hardware envelopes.
 s-=clear-baseline_parts[name]
 for y,z in [(24.7,.5),(35.4,32)]:s-=cy(2.5,.1,16.4,0,[0,y,z])+cy(3.3,7.8,8.6,0,[0,y,z])
 x=sign*11.0
 s-=cy(2.5,14.3,22.5 if sign<0 else 23.9,1,[x,0,32])+cy(3.25,14.3,14.8,1,[x,0,32])
 for cx,yy in [(0,18.2),(13.192323604,26.328448698)]:
  head=cy(3.4,5.8,7.8,2,[cx-3.85,yy,0])+cy(3.4,5.8,7.8,2,[cx+3.85,yy,0])+box([cx-3.85,yy-3.4,5.8],[cx+3.85,yy+3.4,7.8])
  head-=cy(7.6,-100,100,0,[0,10.2,0]);s-=head
 locals()[name]=s

# Open the carriage-control connector toward -Y. The old front arch trapped
# the rod and crossed both pin insertion paths. Retain the bearing below Z25
# and the reinforced pin bosses from Y14.4 rearward.
connector_access=box([-17,-1,25],[17,14.4,40])
left-=connector_access
right-=connector_access

# Replace the removed upper arch with a short web below the rod approach.
# Keep the web at the outer bearing face to clear the lever at full travel.
for name,a,b in [("left",-16.25,-12),("right",12,16.25)]:
 locals()[name]+=box([a,13.5,18],[b,17.5,27.7])-clear
