> Historical revision notes below. Superseded by [Assembly and print revision](../Assembly%20and%20print%20revision.md), including the removable guides, five corrected bearing components and revised pin counts.

# Manufacturing and assembly review — 25 September 2026

**Verdict: this is not ready for a complete print-and-build.** There are concrete assembly and manufacturing problems, plus unresolved functional risks. Passing the existing motion animation is not evidence that the device can be assembled or will move under its own applied forces.

Reviewed geometry: `492e2b00ef440a7b57109ae9ab88796a69b30568055c53163b1074b52506a96b`.

Scope: all 323 model components inventoried: 66 printed parts, 249 native hardware instances, eight elastic loops. Every printed mesh was freshly checked for connectedness, watertightness and downward faces in six cardinal orientations. Existing current-hash bearing, fastening, phase, ratio and motion reports were examined. Native hardware is reviewed by mechanical function; this is not an exhaustive native/native contact simulation. No geometry was changed for this review.

## Problems to resolve before a full print

### 1. Closed rod guides and enlarged integral rod ends conflict with assembly

**Confirmed obstruction to normal axial assembly.** Both bit control rods have 6 × 6 mm shanks, integral 10.4 × 7.6 mm end shoes at BOTH ends, and a 17 mm wide pickup. The guides have 6.8 × 6.8 mm openings. A guide cannot simply slide over an end shoe, even when detached from the frame. The controller rods also have enlarged shoes and integral branches; inserting them through an already assembled stack of guides is obstructed.

Fresh solid probes identify the current guide owners:

| Guide | Position Z, mm | One-piece fixture |
|---|---:|---|
| Bit CLOCK lower / upper | −22 / 21.5 | bit frame 0 removable fixture 0 / 2 |
| Bit WRITE lower / upper | −12 / 32 | bit frame 0 removable fixture 0 / 2 |
| Controller CLOCK lower / upper | −132 / −72 | control frame 0 removable fixture 3 / 2 |
| Controller WRITE lower / middle / upper | −204 / −140 / −72 | control frame 0 removable fixture 1 / 3 / 2 |

All these probes find complete guide rings except the bit WRITE lower guide, where some surrounding ring material has been trimmed. The complete upper ring is sufficient to obstruct ordinary bit-rod installation. This is not a proof against every imaginable tilted assembly manoeuvre, but there is no credible, documented assembly route; forcing or flexing the guides is not an acceptable solution.

**Preferred correction:** open-sided guide saddles with separately pinned retaining caps, or removable rod end shoes. Design the installation path before remounting the guides. Caps should locate on shoulders so their pins retain them without setting the running clearance. Preserve two spaced engaged pins on independent fixtures.

### 2. Support-free frames have shifted the print problem into the fixtures

**Confirmed geometry issue.** All 66 printed parts are watertight single solids. Only 14 pass the simple no-downward-overhang screen in at least one of six cardinal orientations. The other 52 have flagged surfaces even in their best screened orientation. This does NOT mean all 52 are unprintable: small lips may bridge successfully, and arbitrary tilted orientations were not searched. It does mean they are not established as support-free parts.

The worst problems are substantial, not mesh noise:

| Part or family | Smallest flagged area among six orientations, mm² | Implication |
|---|---:|---|
| Master rear fixture (`bit frame 0 removable fixture 3`) | 1,974 | Large integrated fixture needs decomposition/reworking |
| CLOCK rear fixture (`control frame 0 removable fixture 2`) | 1,849 | Same problem in rotated controller |
| WRITE rear fixture (`control frame 0 removable fixture 0`) | 950 | Substantial unsupported surfaces |
| Local clock cam and fork bar | 902 | Complex moving part is not support-free |
| Other working fixtures | 262–883 | Moving guides and rear mounting feet create undercuts |
| Master/slave carriage forks | 149–154 | Roofs/steps remain problematic |
| Master/slave gate forks | about 160 | Similar roof/step problem |
| Controller direct rod/pickup parts | 276–294 | Branches and bearing/pin features conflict in orientation |
| Short levers | about 0.72 | Small local feature; materially less serious |

All 18 transmission walls have flagged surfaces in every cardinal orientation. The orientation with the least overhang often puts their axle bores sideways, so it is not an acceptable shortcut to the requested hole orientation. Extending the bearing rings to the bed did not make their transverse mounting features and all their connecting webs support-free.

**Correction:** give each part one deliberate bed plane. Split noncoplanar mounting features from bearing plates where necessary, with positively located, two-pin connections. Do not solve this by putting permanent support scars inside sliding or bearing surfaces. Review actual sliced layers after geometry correction.

### 3. Five axle-bearing parts still fail the requested bed-contact arrangement

**Confirmed by the current axle-ring audit:**

- `bit frame 0 removable fixture 3` — master rear bearing carrier.
- `bit frame 1 removable fixture 0` — slave rear bearing carrier.
- `control frame 0 removable fixture 2` — CLOCK rear bearing carrier.
- `control frame 0 removable fixture 0` — WRITE rear bearing carrier.
- `Control WRITE direct rod and pickup`.

Their axle rings do not all start on the chosen print-bed plane. In addition, the WRITE direct rod combines an axle bore along Z with a splice pin bore along Y. Both cannot be vertical in one printing orientation. Axle-hole checks alone also miss that pin-hole conflict.

**Correction:** separate the rear cheeks from incompatible guide/mounting projections, or redesign their complete section to have a common bed face. Separate the WRITE rod connection from its bearing support if needed. Do not extend rings into moving hardware merely to obtain a flat bottom.

### 4. Two mounting pins do not by themselves make a stiff support

**Measured geometric concern, not a demonstrated fracture.** Fixture mounting pads are approximately 7.6 mm across with 5 mm bores: about 1.3 mm nominal edge ligament. A 6.6 mm collar relief reduces the local ligament to about 0.5 mm. Seating annuli use radii 3.8 and 3.3 mm, again only 0.5 mm radial width. These are vulnerable features during pin insertion and repeated removal; the surrounding body may help, but needs to be assessed at each pad.

Some controller root links are only about 1.58–1.6 mm thick in depth; the bit WRITE connecting brace is about 2.52 mm thick. Broad pin spacing stops rigid-body rotation but does not stop these connecting webs bending or twisting. Flex can misalign a bearing or rub a sliding rod before anything visibly breaks.

**Correction:** continuous broad seating shoulders, adequate material outside collar pockets, and short ribbed load paths. Orient the layers with the real bending load in mind. Test pin fit on a coupon before pressing pins into a finished thin fixture. No numerical strength claim is possible without material, print settings and force measurements.

## Mechanical risks that need resolving or physical testing

### 5. The purported rollers are axle bushes, not independent roller bearings

The four bit CLOCK/WRITE input/output followers use part `4265c`. The local LDraw library resolves this through `32123` to `32123a`, described as a half-bush with an axle hole. It is keyed to the cross axle. The `free_rolling=True` metadata does not change that physical connection.

The follower can still rotate if its whole axle turns in the round printed bellcrank hole. That makes the printed hole the journal bearing, with the other bushes rotating too. Tight retaining bushes, rough hole surfaces or side loading can therefore turn the intended rolling contact into sliding contact. The present model does not establish low rolling resistance.

**Correction/test:** explicitly specify a freely rotating axle with controlled endplay, or use a round-bore roller on a suitable smooth journal. Check all four followers under side load. Do not replace a rotating journal with a friction pin simply because structural joints use friction pins.

### 6. Bearing alignment and endplay have little tolerance margin

The transmission rings have complete scheduled lands and adequate nominal axle reach in the current checks. That is useful, but does not establish alignment after printing and pinning separate supports.

The feedback shaft has bearing stations at −98.1, 34 and 144 mm: centre-to-centre spans of 132.1 and 110 mm. Three separately located bearings can bind a slightly bowed axle or misaligned frame. The shared reverse drive and coupled shaft likewise need actual alignment and torsional-play checks. Arbitrarily adding more bearings can increase binding.

Several moved retainers have only 0.2 mm nominal face clearance. Three have 4 mm axle engagement; the CLOCK pivot bush has 3.6 mm. These are measured nominal fits, not validated retention or tolerance margins. Printed surface error, collar placement and frame distortion can consume the endplay.

**Correction/test:** establish common bearing datums, use a straight assembly mandrel, specify axial float deliberately, and prove every shaft spins freely before fitting clutches. Check shaft withdrawal and collar installation with the neighbouring gears in place. Current final-position checks do not prove insertion access.

### 7. Pin-only splices have new assembly and lost-motion considerations

The two splice plates are connected with two pins, one into each rod segment, 14 mm apart. They transmit load in shear and are no longer screw clamps. Their side lips help locate the joint, but accumulated pin/socket clearance will still contribute to reversal lost motion along a multi-row bank. Friction pins do not guarantee a preload-free, backlash-free linkage.

The central collars on the pins need an intentional assembly sequence: fit each pin half into its rod shoe, then bring the plate onto the exposed halves. A collar cannot simply be pushed through a 5 mm through-hole. Rod access must be solved first.

Use short splice and guide coupons to measure insertion force, pullout, reversal play and sliding resistance. Also inspect full-stroke clearance at the nearby upper controller guides; passing sampled collision checks does not provide a manufacturing clearance allowance.

### 8. Four actuator gear/lever contact flags remain unresolved

The current rotating-envelope screen flags `U022` against `Short lever` in master, slave, CLOCK and WRITE, with approximately 6.47–6.75 mm³ envelope overlap. These are conservative full-revolution envelopes, so they are **not proof of actual tooth penetration at the operating phase**. They are also not permission to assume contact is harmless.

The exact phase, lever motion and physical contact must be checked. An unintended contact here can jam or damage the lever. The existing printed/printed clearance pass does not address it.

### 9. Clutch shifting under power is not proved by the animation

Equal-tooth external routes preserve speed magnitude when engaged. The clutch rings still need to disengage, cross their neutral travel and engage the other rotating dogs. Timing, backlash, load and tooth-face contact determine whether a fork can actually achieve the prescribed motion. Worm/carriage motion likewise needs real contact and friction assessment.

The viewer prescribes positions from inherited operating traces; it does not solve the shared mechanism's loaded motion. Successful animation cannot rule out a blocked dog clutch, excessive shift force or master/slave timing overlap. Constant instantaneous output RPM through reversal or disconnection is not established by the 1:1 ratio checks.

**Test:** one complete bit with controlled input torque, first slowly and then at intended speed, checking hold, capture, feedback and both direction changes. Measure peak CLOCK/WRITE force before scaling to eight bits.

### 10. Carriage anti-roll and sliding contacts require load qualification

The original flat frame surface provided a reaction against carriage roll. The new separate fixtures must provide intentional reaction surfaces throughout the stroke, without relying on accidental collision with a fork or gear. A rigid roll-to-contact screen is exploratory: it does not establish acceptable contact pressure, friction or resistance to skew. It also excludes gate forks and the passive selector.

The small master/slave lock bolts, their roller axles and retainers need smooth running clearances and enough engagement to withstand an offset follower force. Their print screen also flags about 11.2 mm² each. Gate carriage supports have additional overhangs. Verify that assembly does not preload these sliding parts.

### 11. Elastic loops have anchors, but are not specified physical components

Eight loops are present. Current anchor checks establish material at the expected anchor locations; they do not specify actual band size, material, preload, retention, rubbing or fatigue life. Too much tension raises actuation load; too little can prevent reliable return or allow a band to leave an anchor.

Specify and test real bands, inspect the entire changing path and provide positively retaining anchor geometry that does not require stretching over sharp edges. An animated loop is not a procurable part specification.

### 12. Eight-bit forces and wall mounting remain system-level risks

Shared rods must move the combined mechanism. Pin play, guide friction, rod compression/buckling, frame distortion and timing variation accumulate. Adjacent-row collision screens use the same trace on both rows, not every combination of stored states. Wall mounting can distort an otherwise free-running bench assembly.

Measure one-bit peak force and then test two independently set bits before committing to eight. Use locating features at frame/row joints; do not ask a long flexible rod to align the frames. The two-pin frame seam is present, but its loaded rigidity and repeatable alignment remain unmeasured.

## What the current evidence does support

- All 66 printed meshes are single connected, watertight solids.
- The three frame parts have flat bed orientations and pass the strict 45° surface screen without a bridge exemption. Their oriented sizes are approximately 103.6 × 163.3 × 13.5, 101.6 × 136.3 × 11.1 and 160 × 89.8 × 9.8 mm. Verify these against the actual printer's usable bed.
- The bit frame remains two pieces with two horizontal joining pins.
- The 18 transmission walls have scheduled engaged bearing lands and at least two mounting pins; the 11 working fixtures have paired or four-pin mounts.
- The two splice plate exports pass their support-free surface screen.
- Sampled printed motion reports no printed/printed intersections. The new 32-pin interface screen reports no forbidden contacts with other hardware, excluding each pin's declared mating sockets.
- There are 96 native friction pins overall. That new 32-pin test must not be represented as an exhaustive insertion/contact check of all 96 pins.
- The 14 checked external gear meshes have the intended equal-magnitude ratios. This does not certify clutch dynamics.

## Recommended correction order

1. Resolve rod/guide assembly architecture, including installation and removal paths.
2. Redesign the 11 fixtures and 18 walls around print planes and robust mating shoulders; preserve gear centres and frame height.
3. Resolve follower journals, bearing alignment/endplay and the four actuator contact flags.
4. Produce an actual assembly order and parts/fastener schedule, including inherited hardware IDs and band specifications.
5. Print fit coupons, then one functional bit and its controller; measure forces and timing before repeating the bit eight times.

The full part inventory and measured screen results are in `Component inventory.csv` and `Component audit.json`. The following appendix maps every printed component to its screening result; a small flagged area is not equivalent in severity to a large unsupported roof.

## Printed-component appendix

| Component | Best screened up axis | Flagged area, mm² | Review focus |
|---|---|---:|---|
| master Short lever | Y- | 0.72 | Small print lip; unresolved U022 operating contact |
| master Carriage fork and roof | X+ | 154.44 | Fork/support print features; pin assembly, clutch clearance and skew |
| master Right carriage bearing support | X- | 0.00 | Fork/support print features; pin assembly, clutch clearance and skew |
| slave Short lever | Y- | 0.72 | Small print lip; unresolved U022 operating contact |
| slave Carriage fork and roof | X+ | 148.54 | Fork/support print features; pin assembly, clutch clearance and skew |
| slave Right carriage bearing support | X- | 0.00 | Fork/support print features; pin assembly, clutch clearance and skew |
| master lock bolt | Y+ | 11.23 | Print overhangs, guide fit, roller retention and side loading |
| slave lock bolt | Y+ | 11.23 | Print overhangs, guide fit, roller retention and side loading |
| master_gate Carriage fork and roof | X+ | 160.04 | Fork/support print features; pin assembly, clutch clearance and skew |
| master_gate Right carriage bearing support | X- | 37.58 | Fork/support print features; pin assembly, clutch clearance and skew |
| slave_gate Carriage fork and roof | X+ | 159.69 | Fork/support print features; pin assembly, clutch clearance and skew |
| slave_gate Right carriage bearing support | X- | 37.58 | Fork/support print features; pin assembly, clutch clearance and skew |
| Passive write Carriage fork and roof | X- | 54.08 | Fork/support print features; pin assembly, clutch clearance and skew |
| Local clock cam and fork bar | X- | 902.02 | Substantial overhangs; follower contact, timing and force |
| bit clock bellcrank | Y- | 0.00 | Print screen passes; axle journal freedom and bush endplay |
| bit write bellcrank | Y- | 0.00 | Print screen passes; axle journal freedom and bush endplay |
| bit clock vertical control rod | Y+ | 28.97 | Enlarged ends obstruct guide installation; pin relief overhang |
| clock pinned rod splice bridge | Y+ | 0.00 | Print screen passes; pin insertion order, fit and reversal play |
| bit write vertical control rod | Y+ | 28.97 | Enlarged ends obstruct guide installation; pin relief overhang |
| write pinned rod splice bridge | Y+ | 0.00 | Print screen passes; pin insertion order, fit and reversal play |
| master Front bearing cheek | Y+ | 0.00 | Print screen passes; paired-pin fit and rear-cheek alignment |
| slave Front bearing cheek | Y+ | 0.00 | Print screen passes; paired-pin fit and rear-cheek alignment |
| bit WRITE fork and pickup | X- | 24.26 | Fork/support print features; pin assembly, clutch clearance and skew |
| Control clock Short lever | Y- | 0.72 | Small print lip; unresolved U022 operating contact |
| Control clock Carriage fork and roof | Z+ | 176.41 | Fork/support print features; pin assembly, clutch clearance and skew |
| Control clock Right carriage bearing support | Z- | 0.00 | Fork/support print features; pin assembly, clutch clearance and skew |
| Control clock Front bearing cheek | Y+ | 0.00 | Print screen passes; paired-pin fit and rear-cheek alignment |
| Control CLOCK direct rod and pickup | Z- | 276.21 | Integral branches, guide assembly and conflicting print features |
| Control Clock amplifier front and input shoe | Y- | 31.99 | Print steps; journal fit, side load and lost motion |
| Control Clock amplifier rear and output shoe | Y+ | 27.02 | Print steps; journal fit, side load and lost motion |
| Control write Short lever | Y- | 0.72 | Small print lip; unresolved U022 operating contact |
| Control write Carriage fork and roof | Z+ | 28.85 | Fork/support print features; pin assembly, clutch clearance and skew |
| Control WRITE direct rod and pickup | Z- | 294.01 | Integral branches, guide assembly and conflicting print features; AXLE RINGS NOT ALL ON BED |
| Control write Front bearing cheek | Y+ | 0.00 | Print screen passes; paired-pin fit and rear-cheek alignment |
| bit removable bearing wall 0 | Y- | 228.89 | Axle rings bed-supported; transverse mount overhangs, alignment and insertion access |
| bit removable bearing wall 1 | Y- | 35.06 | Axle rings bed-supported; transverse mount overhangs, alignment and insertion access |
| bit removable bearing wall 2 | Y- | 27.62 | Axle rings bed-supported; transverse mount overhangs, alignment and insertion access |
| bit removable bearing wall 3 | Y- | 20.57 | Axle rings bed-supported; transverse mount overhangs, alignment and insertion access |
| bit removable bearing wall 4 | Y- | 239.12 | Axle rings bed-supported; transverse mount overhangs, alignment and insertion access |
| bit removable bearing wall 5 | Y- | 20.66 | Axle rings bed-supported; transverse mount overhangs, alignment and insertion access |
| bit removable bearing wall 6 | Y- | 20.57 | Axle rings bed-supported; transverse mount overhangs, alignment and insertion access |
| bit removable bearing wall 7 | Y- | 56.12 | Axle rings bed-supported; transverse mount overhangs, alignment and insertion access |
| bit removable bearing wall 8 | Y- | 24.70 | Axle rings bed-supported; transverse mount overhangs, alignment and insertion access |
| bit removable bearing wall 9 | Y- | 107.74 | Axle rings bed-supported; transverse mount overhangs, alignment and insertion access |
| bit removable bearing wall 10 | Y- | 233.24 | Axle rings bed-supported; transverse mount overhangs, alignment and insertion access |
| bit removable bearing wall 11 | Y- | 20.51 | Axle rings bed-supported; transverse mount overhangs, alignment and insertion access |
| bit removable bearing wall 12 | Y- | 119.09 | Axle rings bed-supported; transverse mount overhangs, alignment and insertion access |
| control removable bearing wall 0 | Y- | 20.46 | Axle rings bed-supported; transverse mount overhangs, alignment and insertion access |
| control removable bearing wall 1 | Y- | 26.87 | Axle rings bed-supported; transverse mount overhangs, alignment and insertion access |
| control removable bearing wall 2 | Y- | 20.46 | Axle rings bed-supported; transverse mount overhangs, alignment and insertion access |
| control removable bearing wall 3 | Y- | 41.91 | Axle rings bed-supported; transverse mount overhangs, alignment and insertion access |
| control removable bearing wall 4 | Y- | 78.73 | Axle rings bed-supported; transverse mount overhangs, alignment and insertion access |
| bit frame 0 removable fixture 0 | X+ | 750.62 | Print undercuts, web stiffness and assembly access |
| bit frame 0 removable fixture 1 | Y+ | 262.45 | Print undercuts, web stiffness and assembly access |
| bit frame 0 removable fixture 2 | Y- | 575.48 | Print undercuts, web stiffness and assembly access |
| bit frame 0 removable fixture 3 | X- | 1973.98 | Print undercuts, web stiffness and assembly access; AXLE RINGS NOT ALL ON BED |
| bit coordinated chassis 0 | Y- | 0.00 | Bed screen passes; seam/pin alignment and mounting distortion need testing |
| bit frame 1 removable fixture 0 | Z- | 729.17 | Print undercuts, web stiffness and assembly access; AXLE RINGS NOT ALL ON BED |
| bit frame 1 removable fixture 1 | Y+ | 320.78 | Print undercuts, web stiffness and assembly access |
| bit frame 1 removable fixture 2 | Y+ | 790.04 | Print undercuts, web stiffness and assembly access |
| bit coordinated chassis 1 | Y- | 0.00 | Bed screen passes; seam/pin alignment and mounting distortion need testing |
| control frame 0 removable fixture 0 | Y- | 950.18 | Print undercuts, web stiffness and assembly access; AXLE RINGS NOT ALL ON BED |
| control frame 0 removable fixture 1 | Z- | 883.13 | Print undercuts, web stiffness and assembly access |
| control frame 0 removable fixture 2 | Z+ | 1849.29 | Print undercuts, web stiffness and assembly access; AXLE RINGS NOT ALL ON BED |
| control frame 0 removable fixture 3 | X+ | 472.41 | Print undercuts, web stiffness and assembly access |
| control coordinated chassis 0 | Y- | 0.00 | Bed screen passes; seam/pin alignment and mounting distortion need testing |

## Native hardware and elastic inventory coverage

Each instance is listed in the CSV. Inspection priorities apply to every instance in these groups, not just the illustrated examples. Native library meshes are treated as bought LEGO components, not ready-to-print replacements.

| Group | Instances | Assessment |
|---|---:|---|
| Inherited native ID | 26 | 26 inherited instances including worm/actuator and clutch hardware; incomplete LEGO-number metadata; contact, sourcing and assembly still need explicit documentation |
| 24316 | 8 | Axle reach supported by relevant schedules; straightness, journal freedom, collar installation and extraction access require build checks |
| 32123a | 26 | Axle grip, endplay, access and contact with printed faces; four 4265c followers require whole-axle rotation |
| 2780 | 96 | 96 friction pins: final seating/paired supports checked in relevant schedules; complete insertion/tool-access sweep still needed |
| 26287 | 4 | Coupling engagement, axial retention, insertion sequence and torsional play |
| 94925 | 16 | Gear mesh phase/ratio screened; backlash, shaft bending and loaded contact unmeasured |
| 4519 | 6 | Axle reach supported by relevant schedules; straightness, journal freedom, collar installation and extraction access require build checks |
| 3706 | 1 | Axle reach supported by relevant schedules; straightness, journal freedom, collar installation and extraction access require build checks |
| 44294 | 2 | Axle reach supported by relevant schedules; straightness, journal freedom, collar installation and extraction access require build checks |
| 3737 | 5 | Axle reach supported by relevant schedules; straightness, journal freedom, collar installation and extraction access require build checks |
| 32062 | 2 | Axle reach supported by relevant schedules; straightness, journal freedom, collar installation and extraction access require build checks |
| 3705 | 8 | Axle reach supported by relevant schedules; straightness, journal freedom, collar installation and extraction access require build checks |
| 4265c | 39 | Axle grip, endplay, access and contact with printed faces; four 4265c followers require whole-axle rotation |
| 3713 | 2 | Axle grip, endplay, access and contact with printed faces; four 4265c followers require whole-axle rotation |
| 32073 | 2 | Axle reach supported by relevant schedules; straightness, journal freedom, collar installation and extraction access require build checks |
| 10928 | 2 | Gear mesh phase/ratio screened; backlash, shaft bending and loaded contact unmeasured |
| 50450 | 1 | Axle reach supported by relevant schedules; straightness, journal freedom, collar installation and extraction access require build checks |
| 59443 | 1 | Coupling engagement, axial retention, insertion sequence and torsional play |
| 3708 | 1 | Axle reach supported by relevant schedules; straightness, journal freedom, collar installation and extraction access require build checks |
| 60485 | 1 | Axle reach supported by relevant schedules; straightness, journal freedom, collar installation and extraction access require build checks |
| Elastic loops | 8 | Anchors present; actual band specification, preload, retention and path contact unresolved |

## Fresh carriage roll screen

Re-ran the exploratory roll screen against the current geometry. These are angles at first tiny rigid intersection, not permissible operating roll or proof of a good bearing surface. Five poses from one trace were sampled.

| Carriage | Range to first contact across directions/poses |
|---|---:|
| CLOCK actuator | 1.61–1.90° |
| WRITE actuator | 0.60–0.68° |
| master | 0.84–1.90° |
| slave | 0.84–1.90° |

Nonzero roll freedom can produce edge loading before the nominal mechanism visibly moves off its path. Identify the actual contact faces and test them under load. Gate forks and passive selector are outside this screen.
