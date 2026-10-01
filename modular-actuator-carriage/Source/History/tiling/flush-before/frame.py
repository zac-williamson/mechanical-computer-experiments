"""Keep the 80 x 64 footprint; move connector holes clear of wall feet."""
def tile_frame(frame,box,cy):
 for x in [-32,32]:
  for z in [0,48]:
   frame+=cy(2.51,32,40.2,1,[x,0,z])+cy(3.26,32,32.41,1,[x,0,z])
 for x in [-34,34]:
  for z in [0,48]:
   frame+=box([x-4.5,32,44 if z==48 else z-4.5],[x+4.5,48.4,z+4.5])
   frame-=cy(2.5,31.9,40.1,1,[x,0,z])+cy(3.25,31.9,32.4,1,[x,0,z])
 return frame
