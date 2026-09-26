> The latest matching carriage, track and band-anchor changes are described in [Planar fixes adopted](Planar%20fixes%20adopted.md). Use the current exports together.

# Assembly and print revision

This revision opens the nine orange-rod guides with removable, two-pin caps. It separates four rear bearing cheeks and the controller WRITE bearing shoe so their axle-bearing rings can start on the print bed. Mounting pads are wider and fixture webs thicker. Gear centres, clutch polarity, shafts and roller mechanisms are unchanged.

The frame remains the same height and row pitch (112 mm), with two separately printed bit-frame halves and their horizontal joining pins. One keeper projects 6.8 mm farther left than the previous overall bit envelope; the frame itself has not grown.

## Assembly order

1. Print frames using **Print oriented frames/**. Print transmission walls using **Print oriented bearing walls/**, remaining bearing fixtures using **Print oriented axle parts/**, and new caps/cheeks/WRITE pieces using **Print oriented assembly parts/**. Use **Print oriented rod splices/** for bridges. Do not print assembly-coordinate Development parts without orienting them.
2. Install the rear bearing cheeks on their carriers using the two existing cheek axes and LEGO **6558 3L friction pins**. The long pin ends engage the rear cheek and carrier; press the front cheek onto the exposed short ends. Do not force the centre collar through a bearing or mounting hole.
3. Fit the guide carriers to the orange rods on the bench, before fastening them to the frame or fitting follower and actuator hardware. Remove the nine guide caps first. CLOCK shanks enter from the front. WRITE shanks approach sideways at Y=17.8 mm, then move back into the grooves: from the right for the controller's lowest guide (Z=-204 mm), from the left for the other WRITE guides. Fit each detached carrier individually; do not try feeding enlarged rod shoes through the closed guides.
4. Seat each cap's two LEGO 2780 pins from the mating faces, then press the cap onto the exposed halves. Attach the carriers to the frame with their paired/four-pin mounts. Check free rod travel before continuing assembly.
5. Connect the separate WRITE bearing shoe and rod using its two 2780 pins. Fit the CLOCK/WRITE splice bridges by seating pins in the rod shoes first, then pressing on the bridges.

## Manufacturing changes

- Pin mounting pads are now 9.6 mm wide, with a nominal 1.5 mm ligament outside the 6.6 mm collar recess. Guide-cap sockets use larger 10.6 mm pads.
- Thin rear fixture connections are deeper, and controller braces have thicker sections. Each fixed bearing fixture retains at least two spaced mounting pins.
- Four rear pivot-bearing cheeks print rear-face down. The separate WRITE bearing shoe prints with its extended axle ring against the bed. A relieved CLOCK pivot sleeve reaches its common bed face without moving the axle or bush.
- New 2780 mating seats include relief for the full centre collar. No screws are used.

## Evidence and limits

See [current validation status](Validation%20status.md) and the matching geometry hashes in the reports. The guide installation screen verifies local shank paths on detached carriers; it is not a simulation of every hand/tool movement in a fully populated register. These are CAD checks, not measured printed strength, friction or fit. Static non-running overhangs are retained where removing them would require unnecessary redesign. Test pin fits and moving clearances with one bit before printing eight.
