> **Cam load review:** the present bolt guide is too short and its forces too offset to recommend printing with confidence. The guide allows more tilt than the previous timing allowance. See [loaded mechanism audit](Cam%20mechanism%20audit.md).

> **Timing audit update:** This bare latch can stop between pockets during simultaneous data reversal and WRITE closure. Do not treat earlier sampled collision passes as a functional pass. See [transition audit](Transition%20timing%20audit.md) and [Ben Eater SAP capture timing](SAP%20register%20timing.md).

# Direct-cam 1-bit register

[Open the labelled viewer](Viewer.html). This replaces the crank-and-slot revision. It remains a CAD prototype with a 0.1 Nm target, not a physically qualified mechanism.

## Mechanism and frame

The orange carriage carries one flat cam plate. A genuine LEGO half-bush on a 2L axle rolls against its rising face and lifts the lock bolt. The crest leads into a flat dwell. The elastic band inserts the bolt on HOLD; an interrupted write can leave the bolt resting on the keeper until a valid pocket returns.

The crank, both pivot brackets, pivot axle, long follower axle and spacer stack are removed. The bolt guide is integrated into the existing blue front bearing cheek, using its two original LEGO cartridge-pin connections. Its band anchor is local to that guide. One rear backbone joins the axle-bearing stations; unused upper rear extensions are removed above the mounting-pin regions. No independent lock tower is added.

The cam plate attaches at the orange carriage upright through two LEGO 2780 pins and a shallow shear key. Its continuous, deep web carries the load without the earlier tall rectangular slot or pivot-clearance cutouts. The original two-piece bearing carriages and their bearing-flat print faces are retained.

| Quantity | Previous crank revision | Direct cam |
|---|---:|---:|
| Printed parts | 20 | 19 |
| Modelled LEGO hardware pieces | 89 | 80 |
| Maximum bolt lift | about 14.2 mm | 7.8 mm |
| Sampled total height | about 103.7 mm | 95.8 mm |
| Total X / Y envelope | 185 / 42.6 mm | 185 / 42.6 mm |
| Orange rear extent | Y31 mm | Y31 mm |

## Signals and sequencing

Viewed from +X toward the origin, anticlockwise is 1 and clockwise is 0. WRITE is the directly accessible orange worm shaft, Y10.2/Z32. D enters the lower right shaft, Y10.2/Z0, and the single gear mesh supplies NOT D to the memory actuator. The blue power paths are exchanged so external Q=D during a completed WRITE. Constant 1 power enters the blue lower shaft at Y10.2/Z−16; Q is at Y10.2/Z0.

WRITE 1 withdraws the bolt before connecting D. WRITE 0 disconnects D before inserting the bolt. Q retains the last completed bit during HOLD. Q = 1 runs at half the constant-power speed; Q = 0 runs at full speed in the opposite direction. Clock sequencing must allow the slower write to complete. An interrupted stroke is not a valid stored bit.

The cam slope is 1.8. Under the stated ±0.6 mm combined position-error assumption, the calculated bolt clearance before possible D reconnection is 0.628 mm. The rounded motion around the crest comes from the LEGO roller radius; the cam has a straight ramp and flat dwell.

## Printing and assembly

Print only the 19 STLs listed in `Prototype print parts/Orientations.json`. Both original carriage bearing halves use their original minimum-X bed faces. The separate cam is 5.4 mm thick and prints with its uninterrupted Y31 rear face down; its entire projected footprint contacts the bed. Its key recess opens upwards. Cam attachment pins are at X86/Y25.5/Z44.4 and 58.

The new roller uses one LEGO 32062 2L axle and two 32123a half-bushes. The axle turns inside the printed bolt head; the front bush retains it and the rear bush is the roller. The cam remains clear of the bolt guide. Retain both original actuator bands and add the local lock band. Its nominal loop path is 33.6–49.2 mm; initially target about 0.5 N closed and at most 2 N withdrawn, verified by measurement.

Use standard Bambu PLA and a 0.4 mm nozzle. All 19 print meshes reload as single watertight solids. Small band shoulders and transverse openings still need slicer/support review; no physical print or fit qualification is claimed. The inter-core LEGO axle connector remains represented by an external envelope, so its internal fit is not proven by the model.

## Checks and limits

- All 16 ordered D/WRITE input transitions, both stored bits during HOLD, six interrupted-write/resume paths and an independent ±4.6 mm travel envelope: 2,657 unique rigid poses. No printed/printed overlap above 0.005 mm³.
- Every printed/native contact candidate is narrowed using native vertices, triangle centres and edge midpoints. Designed friction-pin interference is classified separately from moving-part collisions.
- 1,800 X-axis rotation configurations plus conservative full-rotation/axial-play envelopes for both clutch rings against their forks.
- Full 3,660-frame source lever trace for each actuator: no contact with the added carriage material.
- 921 cam/roller positions: no geometric penetration; negative overtravel allows the cam to retreat from the stopped bolt.
- 279 lock-band states and 186 states for each original band: no sampled printed-part intersections.

These are finite rigid CAD checks. Open inherited LEGO meshes, clutch engagement dynamics, force-dependent band shape and elastic deformation prevent a complete continuous all-part guarantee.

The cam has a 60.9° pressure angle, so guide friction matters. `Qualification.json` includes a friction sensitivity calculation; actual friction is unmeasured. The ±0.6 mm timing allowance is an assumption, not a demonstrated bound on printed-guide play and loaded tilt. `Flex screening.json` includes ordinary band-load and ideal blocked-worm cases. Its beam model omits joints, local contact, stress concentrations, layer adhesion and creep. Survival of 0.1 Nm on every axle under every condition remains unqualified.

Current generator: `Source/direct_cam.py`; current viewer: `Source/publish_direct_cam.py`. Both use the same `direct_cam_math.py` cam law as the checks. Earlier crank generators and archived geometry are superseded. Source multiplexer commit: `5d987d9d05220db57576819dd6835af9f23af557`. No unrelated register design was used.
