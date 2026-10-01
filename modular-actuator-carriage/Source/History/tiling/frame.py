"""Keep the 80 x 64 footprint; move connector holes clear of wall feet."""
def tile_frame(frame,box,cy):
 for x in [-32,32]:
  for z in [0,48]:
   frame+=cy(2.51,32,40.2,1,[x,0,z])+cy(3.26,32,32.41,1,[x,0,z])
 for x in [-34,34]:
  for z in [0,48]:
   frame+=box([x-4.5,32,44 if z==48 else z-4.5],[x+4.5,48.4,z+4.5])
   frame-=cy(2.5,31.9,40.1,1,[x,0,z])+cy(3.25,31.9,32.4,1,[x,0,z])
 # Recess connector seats so 16 mm pins terminate at the rear plane Y=48.4.
 for x in [-34,34]:
  xa,xb=(-40.1,-29.3) if x<0 else (29.3,40.1)
  for z in [0,48]:
   za,zb=(-8.1,4.7) if z==0 else (43.8,56.1)
   frame-=box([xa,31.9,za],[xb,40.4,zb])
   frame-=cy(2.5,40.3,48.5,1,[x,0,z])
   frame-=cy(3.25,40.3,40.9,1,[x,0,z])
   frame-=cy(2.7,48.1,48.5,1,[x,0,z])
 return frame
