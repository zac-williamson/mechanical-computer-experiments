# Validation status — modular wall register

**Development candidate; not a print release.**

Geometry SHA-256: `587c33eaf07f7d5056ac771d8d2b1b2dadd5b4d19142e63c1d098bf47a68f993`.

| Screen | Current evidence |
|---|---|
| [Development checks](Development%20checks.json) | PASS: 910 sampled unique poses; 0 printed intersections; two adjacent rows + controller |
| [Rotation ratio checks](Rotation%20ratio%20checks.json) | PASS: 14 discovered external meshes; unit magnitude |
| [External gear phase checks](External%20gear%20phase%20checks.json) | PASS: sampled external tooth profiles (not worm/clutch qualification) |
| [Rotating envelope screening](Rotating%20envelope%20screening.json) | 4 potential contacts retained for review; 1013 poses across the 112 inherited operating cases |
| [Axle crossing screening](Axle%20crossing%20screening.json) | 0 potential axle crossings |
| [Bearing and retention checks](Bearing%20and%20retention%20checks.json) | PASS: 32 complete bearing lands; 15 retained transmission assemblies |
| [Bearing attachment checks](Bearing%20attachment%20checks.json) | PASS: 18 removable transmission walls plus front/rear cheeks; two engaged pin axes each; two-piece frame joint verified; 11 paired-pin fixture mounts |
| [Bearing print orientations](Bearing%20print%20orientations.json) | PASS: 18 wall STLs with bearing axes normal to the bed |
| [Axle print-face audit](Axle%20print-face%20audit.json) | PASS: 47 parts screened; 0 parts need further separation or bed-face work |
| [Frame print checks](Frame%20print%20checks.json) | PASS: 3 flat-bed frame parts; no unsupported faces beyond 45 degrees; no bridge exemption |
| [Frame pin clearance checks](Frame%20pin%20clearance%20checks.json) | PASS: 60 friction-pin envelopes checked against other hardware and parts through operating poses |
| [Rod splice checks](Rod%20splice%20checks.json) | PASS: two pinned rod splices, two engaged pin sockets each |
| [Rod splice print checks](Rod%20splice%20print%20checks.json) | PASS: two splice bridges with pin holes normal to the bed and no unsupported faces beyond 45 degrees |
| [Frame rebuild verification](Frame%20rebuild%20verification.json) | PASS: source rebuild reproduces the same solids and poses; triangle ordering may differ |
| [Elastic anchor checks](Elastic%20anchor%20checks.json) | PASS: 16 anchor centres present across all sampled operating cases |
| [Force and mass review](Force%20and%20mass%20review.json) | Geometric printed-volume comparison and ideal force balance; no measured force or strength claim |
| [Revision connection checks](Revision%20connection%20checks.json) | PASS |
| [Manufacturing revision checks](Manufacturing%20revision%20checks.json) | PASS |
| [Assembly print orientations](Assembly%20print%20orientations.json) | PASS |
| [Carriage section checks](Carriage%20section%20checks.json) | PASS: eight carriage halves; reinforced corners and backing sections |
| [Band installation checks](Band%20installation%20checks.json) | PASS: closed-band insertion at both lock anchors |

## Scope

The printed screen samples 17 positions from each of 112 inherited transition/initial-state traces, deduplicates identical printed poses, and adds a neighbouring row. It checks volumetric intersections above 0.01 mm³ without contact exclusions, plus watertightness and connected-solid count. It is a sampled screen, not a continuous swept-volume proof. The linkage equations separately cover 1,001 points across each nominal stroke.

Full-revolution native envelopes are conservative. Their remaining actuator U022 / Short lever flags are reported rather than silently excluded; identifying an inherited working interface does not establish that it is correct in hardware.

## Open requirements

- The new shared linkage has not been dynamically or contact-resolved simulated under load. Imported original traces prescribe positions.
- Full native/native collision, worm contact, phase-resolved actuator gear/lever contact, retention and assembly-access qualification remains open.
- Elastic loops follow their inherited moving-anchor paths in the viewer; preload, retention and physical elastic contact are not qualified.
- Printed strength, rod buckling, tolerance, friction, wear, pin fit and eight-row actuation force have not been measured.
- The expanded printed screen uses two adjacent rows carrying the same inherited trace; independent bit states and loaded rod deflection remain unverified.
- Development parts STLs are in assembly coordinates. Use the separate oriented exports and documented rod/carrier assembly sequence; final slicer review and physical pin fit remain to be checked.
- Equal engaged gear ratios do not guarantee constant instantaneous RPM during reversal or disconnection.
- The revised keeper adds 6.8 mm to the left-hand bit envelope; frame height and 112 mm row pitch are unchanged.
