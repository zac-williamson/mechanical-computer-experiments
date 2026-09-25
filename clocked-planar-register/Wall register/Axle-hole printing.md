# Axle-hole print faces

The transmission-wall correction is implemented. The complete mechanism is **not yet compliant** with the request that every axle-bearing ring starts on the print bed.

## Implemented in the model

All 18 transmission walls have an explicit print orientation with the axle axes vertical. All 32 scheduled bearing rings now reach that wall's common bed plane. Six staggered rings on four bit walls needed extensions:

| Wall | Bearing | Extension to bed |
|---|---|---:|
| 0 | Feedback | 4 mm |
| 4 | Master output; POWER reverse | 6 mm each |
| 10 | Master output; Feedback | 3.4 mm each |
| 12 | Slave worm | 0.4 mm |

Only annular feet were added. Projecting entire walls to the bed would collide with fixtures and frame rails. The added sleeves have relieved bores, up to 6.4 mm diameter, with 45-degree transitions toward the existing 5.3 mm bearing bores. The original bearing lands, shaft and gear positions, pin attachments, frame split, overall dimensions and row pitch remain unchanged. Three upstream axle collars move to the outside of the new feet, preserving their 0.2 mm axial clearance; this prevents rotating collars rubbing inside the extensions. This preserves the intended short bearing lands instead of adding the whole extension as close-fitting bearing length.

Use **Print oriented bearing walls/** for these parts. The annular feet have a 9.9 mm outer diameter; the original bearing rings remain 10 mm. These exports establish bed contact for the bearing rings, not support-free printing of every other feature: transverse mounting-pin holes and other wall features still require separate review.

The bit CLOCK pivot fixture is also corrected: a 0.4 mm extension brings the bearing ring to the rear seating plane. Its outer bush moves 0.4 mm to preserve the 0.2 mm gap and retains 3.6 mm engagement on the existing axle. The merged WRITE pivot/guide carrier also has its bearing ring extended to the rear seating face.

## Other axle-bearing parts

**Axle print-face audit.json** checks the actual native-axle interfaces against printed parts, then tests both possible orientations along the axle axis. The check measures material in the first 0.04 mm of the bed-contact annulus; merely turning a bore vertical is insufficient. **Print oriented axle parts/** includes only parts passing that contact screen. Unsupported surface area elsewhere is reported separately; these are development exports, not complete support-free print qualifications.

The remaining problem groups are:

- The four rear actuator cheek/guide fixtures: master, slave, shared CLOCK and shared WRITE. Their bearing faces and structural roots sit at different depths. Thickening the entire depth would add unnecessary material and, for several of them, intersect the carriage or amplifier. The appropriate revision is a separate flat rear bearing plate, positively located and attached to its structural carrier at two spaced points. Both pivot and reaction bores should share one plate and one bed face. Those joints have not yet been designed or validated.
- The shared WRITE carriage/rod pickup: the long rod is fused to the carriage bearing support and projects beyond both candidate print faces. This needs a separable rod pickup, with a positively located friction-pin connection outside the moving gear envelope. Orient the bearing support independently. This also needs to be coordinated with the previously identified closed-guide insertion problem; the rods' assembly access remains unresolved.
The bit frame's WRITE-pivot passage is explicitly classified as a clearance passage, not an axle bearing. The frame passes its separate bed-down overhang screen; this passage is recorded separately in the audit.

The audit deliberately reports these exceptions. No claim is made that all axle holes or the entire mechanism can now be printed without supports.
