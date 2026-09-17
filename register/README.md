# Compact-lock one-bit register

The overhead lock has been replaced with a low, sideways-moving bolt, one narrow diagonal cam and a separate elastic return band. The middle rear bridge returns to the final multiplexer's original shape. The 236 × 100 × 8.2 mm base and aligned drive shafts are retained.

The lock guide and keeper are integral with the front bridge; the return-band anchors are integral with the rear bridge. The only separate printed lock parts are the moving bolt and cam. Overall height is approximately 69.5 mm. There are 37 installed printed parts, 201 LEGO components and four elastic bands.

This remains a physical prototype. Geometry checks establish clearances, not switching force, fatigue life or reliable operating speed.

## How the new lock works

The bolt enters one of two pockets cut into the **front edge** of K's carriage roof. It travels along Z rather than vertically. Its nominal engagement is 2 mm, with 0.4 mm clearance to the pocket floor. A 2.8 mm withdrawal gives nominal 0.8 mm clearance from the roof edge.

A LEGO 2780 friction pin fitted in the bolt acts as the cam follower. It slides in the cam slot; it is not a rotating roller. A single band loops around two fixed ears and a saddle on the bolt. Its symmetric pull biases the bolt toward engagement without requiring the write carriage to hold it there by force.

The nominal HOLD position is 2.5 mm inside the flat portion of the cam centreline. The closed slot limits bolt withdrawal while the follower is in that flat. The ramp withdraws the bolt before the write clutch can engage. At W local q=−4.35 the register holds; q=+4.325 writes. The cam moves by −q in world X. Its withdrawal ramp runs from q=−1.85 to +0.5.

The clearance-aware calculation gives at least **1.65 mm engagement after up to 2 mm of write-carriage disturbance from HOLD**, using a conservative 2.46 mm follower radius. The actual native pin's 2.6001 mm rib envelope is also checked for fit. At and beyond the first possible clutch contact near q=+1.2, the clearance envelope leaves at least **0.51 mm between the bolt and roof**. These margins exclude print error, structural bending, wear and backlash elsewhere in the mechanism.

This improves resistance to small wobble. It is not an independent bistable latch: sufficiently large write-carriage movement still intentionally withdraws the bolt. No write-carriage detent is included. Square pocket walls transfer carriage load to the fixed bolt guide; the band seats the bolt but does not carry the main holding load. Q̅'s gears and axles still have rotational backlash.

## Print files

Use the numbered print-plate STLs or individual files in `Print parts/`. Top-level individual STLs are in assembly coordinates. Units are millimetres; do not scale them. Exact plate sizes and membership are recorded in `Print checks.json`. Allow extra space for brims and machine exclusion zones, or rearrange the individual parts while preserving orientation.

All actuator axle-bore entrances remain directly on the print bed. Each carriage remains two pieces. The side bolt prints standing on its nose; its guide faces stand vertically and its follower **friction-pin hole** prints sideways. There are no axle bores in the new locking parts. Both bridges retain their original flat axle-bore printing faces. The integrated front guide ends at that face; its running walls are vertical and do not lie on the bed. The cam prints flat on its exterior face with vertical slot walls.

Friction-pin holes may need printer-specific compensation. Print the separate bearing coupon first: 1/2/3 edge notches identify 5.2/5.3/5.4 mm bores. New axle bearings use 5.3 mm nominal bores; inherited actuator holes retain their existing dimensions.

Review local overhang support, particularly the cap, band saddle and inherited carriage details. Keep supports off running faces and out of axle bores. No slicer validation or G-code is supplied. Printed joints use LEGO friction pins, with no screws, adhesive or delicate printed snap joints. The 62 mounting holes pass through the base and nominal pin tips finish 0.2 mm above its underside.

## Assembly

The viewer's Focus menu can isolate **Lock and cam**. K is the middle storage actuator, W is write, and E is output enable. The manifest supplies component placements and the CSV gives LEGO quantities.

1. Assemble each two-piece carriage, guides, worm, reaction gear, lever and bushes. Fit the original three actuator return bands. Confirm free travel before adding the lock.
2. Assemble shafts through the bearing walls before pinning those walls to the base. K retains the powered direct and reversing gear paths. W's green clutch hub shaft couples directly to K's purple selector through the connector at X−40, Z32. D drives a 16T gear at Z48 directly meshing with W's clutch gear.
3. Join K's intermediate output to E's clutch hub through the connector at X40, Z0. Preserve the specified shaft lengths and offsets. W's 9L control shaft spans X−112.8 to X−40.8 at Z0; E's selector spans X40.8 to X120.8 at Z32. Both clear their neighbouring shafts by 0.8 mm.
4. With the front bridge off the register, slide the complete bolt from positive Z into the guide built into the front bridge, body first, nose and band saddle trailing. Keep the follower pin and elastic band out during insertion.
5. Insert the 2780 follower pin through the top opening into the bolt: flange Y−22, centre X−7, Z18 in the locked reference pose. The upper half projects toward the cam.
6. Fit the bridges using their base friction pins. No guide mounting pins or separate housing remain. Check full 2.8 mm bolt travel.
7. Loop a lightly tensioned elastic band around the two fixed rearward ears and the moving bolt saddle, following the viewer. The fixed ears sit near X−17 and X3, Z38; the moving saddle is near X−7, Z28.5 when engaged. The band is behind the retaining heads, approximately in plane Y−26.8. Small reliefs in the ears clear its changing path.
8. With the bolt withdrawn and W in its write position, lower the diagonal cam slot over the projecting follower pin. Attach the cam to the side face of W's carriage with **one horizontal friction pin**, joint at X−60.4, Y−18.6, Z−2 in the reference assembly. The root meets the carriage side below the roof line. A broad keeper on the front bridge supports the cam against twisting; the pin is not the only rotational restraint. Check that both the follower and band remain captured throughout the stroke.

The return-band model shows routing, not a specified commercial band size or force. Select only enough tension to seat the bolt consistently. Excess tension increases write effort and cam friction. Test the native friction-pin follower for sliding friction and wear against the printed slot.

## Signals and use

Directions are defined looking from negative X toward positive X. Clockwise is 0 and anticlockwise is 1; viewing the other end reverses their apparent direction. P runs continuously clockwise. A stopped shaft is neither logic value.

| Port | Endpoint X,Y,Z, mm | Function |
|---|---|---|
| D | −112, 10.2, 48 | Data gear input |
| W | −112.8, 10.2, 0 | 1 writes; 0 holds |
| P | −48, 10.2, −24 | Clockwise power |
| Q̅ | −40, 10.2, 0 | Inverted continuous stored output; manifest key Q |
| BUS_OUT | 86, 10.2, −16 | Stored direction when OE=1 |
| OE | 120.8, 10.2, 32 | 1 connects output; 0 disconnects |

The direct data gear mesh reverses the storage selector's direction. The direct BUS_OUT gear mesh reverses it again, restoring the stored data polarity. D and BUS_OUT remain separate ports requiring external bus routing.

Start with OE=0. Establish stable D, command W=1 and wait for full storage travel. Command W=0 and wait for HOLD to seat before changing D or enabling OE. Do not command HOLD during an incomplete write, since the bolt can meet the roof between pockets. Perform a known write before relying on the first read. Power-off preserves position but produces no rotating output.

## Verification and first physical test

The included reports cover sampled printed-part travel, native hardware versus printed surfaces, fixed pin mounts, bore print orientation and the new cam/band geometry. The gear train is unchanged from the aligned rectangular design. The viewer shows inspection poses, not a dynamic contact simulation.

`Lock checks.json` records the clearance-aware follower envelope, its rib fit and the band's clearance from printed components. The geometric band relief does not account for elastic stretching, sag or twisting. `Source/` includes the generator and checks with its own regeneration instructions. `Fixed pin sweep.json` checks the complete moving cam against conservative outer envelopes of all 62 fixed friction pins in 181 positions, including their collars. Fixed pins are also included against moving printed parts in the native-mesh checks; the earlier exclusion that missed the photographed clashes has been removed.

For the first test, run slowly without a driven load. Verify both stored bits and repeated writes of the same bit. With HOLD seated, gently disturb the write carriage within its backlash and observe whether the bolt remains seated. Then measure whether the bolt fully clears before clutch engagement, and whether it reliably re-enters both pockets. Check the diagonal link and its pin joints for flex. Switching torque, timing, band tension, pin retention and long-term wear remain physical validation tasks.


## Animated lock explanation

Open Viewer.html and choose **Play explanation**. The 32-second loop shows holding, unlocking, writing the other bit, and relocking in both directions. Pause or scrub the Sequence slider; **Hide fixed lock housing** exposes the blue bolt. This is an illustrative motion sequence, not a force or timing simulation.

The pin in the yellow slot is the moving follower attached to the bolt. The horizontal pin at the other end attaches the cam to the write carriage. All four former housing fastening pins and their holes have been eliminated.

## Integrated bridges: printing and assembly

The front bridge includes a 3.0 mm guide floor, 3.7 mm side walls and 2.2 mm retaining strips, plus the cam keeper. There is no thin shell around a mounting pin. Its maximum Z is the unchanged axle-bore bed face. The rear bridge includes a broad top extension and band anchor arms, also behind its unchanged bed face. Both modified bridges are single connected, watertight meshes; bearing entrances are checked at bed height zero. Supports may still be needed on exterior details; keep them off sliding faces and axle bores.

The bolt is 20 mm long rather than 22 mm. The full-width body ends sooner and its taper begins earlier. Its 12 mm width is retained to leave 2.7 mm around the follower collar recess; arbitrarily narrowing it would reintroduce thin walls. Nominal guide overlap is 7.8 mm when locked and 10.6 mm when withdrawn. Stroke remains 2.8 mm; nominal roof engagement remains 2 mm. The carriage position is unchanged: clipping the fixed guide to the bridge bed plane avoids shifting gears or axles.

`Insertion checks.json` samples the complete bolt, including its band saddle, along its insertion path in 0.1 mm steps. This verifies the front-bridge/bolt subassembly before fitting it to the register, not arbitrary insertion into a fully assembled register. Physical friction, strength and tolerance remain to be tested.

Choose **Explode lock parts** to see both integrated bridges, the blue bolt and yellow cam. The rear bridge is offset sideways for inspection; these display offsets are not assembly motions.
