# One-bit register

A bidirectional data bus, continuous power input, write control (W) and output enable (OE). Three LEGO clutches control writing, stored-value selection and bus output. A captive HOLD bolt locks the selector position; a closed cam on the write linkage releases and seats the bolt.

The assembly contains 171 rigid LEGO pieces, 11 printed pieces and three return bands. Stationary frame bearings are 5.4 mm circular printed bores. Moving worm-bearing liftarms remain native LEGO parts. Both frame caps and all closure pins are required.

## Operation

Disable OE, establish a stable data direction and assert W until the selector fully seats. Return W to 0 while keeping data stable; the input clutch disconnects before the holding bolt seats. Only then change the bus or enable OE. Do not command HOLD midway through a write.

Viewed from the negative-X end looking along an input axle, clockwise represents 0 and anticlockwise represents 1. A disconnected bus represents neither. OE connects the selected powered output to the shared bus; external bus drivers must release it first. The output direction carries the bit value; output speed is one third of the power input.

| Port | Axis Y, Z (mm) | Exposed left tip X (mm) |
|---|---|---|
| D / BUS | 22.2, 16 | −128 |
| P | 10.2, −12 | −128 |
| W | −37.8, 64 | −120 |
| OE | −29.8, 32 | −120 |

The supplied design includes 16L axles; it does not currently satisfy an all-axles-at-most-12L inventory.

## Printing

Print only the files in `Print files`, at 100% scale in millimetres. Eleven numbered files are installed parts; the axle bore fit coupon is a separate test piece. Assembly-position STLs beside this guide are for inspection.

Print the coupon first. One to four bars identify 5.0, 5.2, 5.4 and 5.6 mm holes. The frames use 5.4 mm. Check real axle rotation, sliding and sideways play. Its horizontal holes expose bridging and hole-shape problems. Inspect support requirements and avoid damaging the running surfaces during removal.

Frames rest on their outer rail faces. Moving parts use their supplied orientations. A 0.4 mm nozzle, 0.2 mm layers and five walls are prototype starting settings, not validated performance settings. Inspect horizontal bores, the HOLD guide and carriage links in the slicer. Do not scale the whole assembly to adjust a fit.

## Assembly

Keep both frames accessible while threading each shaft and fitting gears, bushes and spacers. Some bores are entirely within one frame half: a completed shaft train cannot necessarily drop into place, and a cap cannot lower over an already fixed axle.

Connect the W/OE forks to loose clutch rings, install the HOLD follower and cam, then close the frames. Retain all axle bushes and spacers. Do not force a cap over a misaligned axle. Check axle fit again with the frames pinned together.

The bolt uses a genuine LEGO roller on a 2L axle with a half bush. Confirm free roller rotation, free guide travel and full engagement in both selector notches. Initialize with OE off and the bolt withdrawn, then complete a known write before engaging HOLD.

## Qualification

The viewer illustrates a prescribed write/hold/output cycle, not contact forces or continuous powered output. The model needs physical assembly, axle fit, friction, wear, stiffness, lock-retention and clutch-timing tests. Printed-hole play can consume the small clutch and lock margins. No speed, load or eight-bit endurance rating is established.
