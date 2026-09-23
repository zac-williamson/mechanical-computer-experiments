# Planar register — detachable bolt guide

Development assembly: not physically qualified or ready for a confidence print of the complete register.

## Detachable guide and print orientation

The bolt guide is now one separate printed part with two genuine LEGO 2L friction pins. Positive seating lands on the base locate its height; outside shoulders resist sideways movement. The pins engage the guide by approximately 6.5 mm. The base and guide retain the same working bolt locations. The fixed elastic-band anchor is 2 mm higher (Z50); the displayed band follows it. Select band length/preload for this revised spacing.

The bolt head is captured between front and rear retaining lips, with 0.4 mm nominal Y clearance per side. Rear retaining lands provide 9 mm of full-height contact throughout the 5.4 mm stroke. Their 45-degree undersides preserve printability and clear the cam and roller. The Y-restraint regression verifies actual retaining material, zero nominal overlap at 55 stroke positions, and rejection of the previously possible 5-degree pitch. An analytic bound allowing Y translation limits rigid pitch to approximately ±1.9 degrees; it does not qualify PLA flex or combined off-axis loading. See `Detachable guide Y restraint.json`.

The guide prints with its broad flat foot on the bed. Its rear cam opening has a 45-degree arched roof; the only bridge is the 1.2 mm elastic-band groove, with solid support at both ends. The base prints rear-face down; rearward ledges are filled back to that face and horizontal guide-mount holes have peaked roofs. Neither part calls for support material in these supplied orientations. Use 0.2 mm layers with the 0.4 mm nozzle. These are geometric checks, not sliced toolpaths or a physical print guarantee; Bambu Studio was not used.

The layout contains 20 printed parts. LEGO hardware is excluded. Original two-piece carriage bearing print orientations are retained. `Support-free print checks.json` records layer and overhang checks; `Detachable guide checks.json` covers the roller sweep and revised band path. Frame clearance checks cover the changed fixed parts, not a new whole-machine certification.

## Current frame

Seven new bearing walls support seventeen shaft locations. Each wall is generated around the actual shaft centre lines, with complete circular bearing surfaces; the original displaced bearing bosses and patched holes are discarded. The outer walls support power/data, clutch and worm-input shafts. The memory idler has its own two coaxial supports. The actuator's established cheek mating interface is retained at the right-hand wall; the bearing bosses, ribs, feet and base layout are new.

Every removable wall uses two genuine LEGO friction pins. Fourteen wall mounts and their matching base holes come from one coordinate table. Pin bores are 5.0 mm nominal, with seats for the native pins' central collars. The open rear frame connects those mounts, the existing functional carriage guide rails and the cam-bolt guide. The WRITE left support is now removable rather than buried in the base.

The inner idler bearing uses a full 7.6 mm long bore with an 8 mm outside diameter. Its redundant recessed bush is removed. Other bearing bosses are 11 mm outside diameter with 5.3 mm shaft bores; inner power/data supports retain counterbores for their bushes. The main bearing bores print vertically, with each wall lying on its X face. Print-oriented STLs and assembly STLs are separate.

## Checks for this revision

- `Bearing frame datums.json`: all bearing axes and matching wall/base mount coordinates.
- `Bearing support checks.json`: seventeen bearings checked against actual native shaft centres, full supporting circumference at five axial stations, matching pin holes, and 42 conservative full-rotation envelope pairs. No envelope intersections reported; moving X-axis hardware envelopes include ±4.2 mm travel.
- `Frame printed clearance.json`: 1,584 boolean comparisons after broad-phase filtering; independent carriage travel, bolt lift and lever-angle sweeps against the new fixed parts. No intersections reported in these samples.
- `Frame native contacts.json`: native surface points against the new frame at the reference pose. No unexpected penetrations reported. Friction-pin fits are explicitly listed, not treated as clearances.
- `Print orientations.json`: nineteen watertight, single-solid print-oriented parts. No slicer or physical print validation.

These are targeted checks of the redesigned frame, not a new whole-machine continuous collision proof. `Coupled assembly contacts.json` predates this frame and is historical. The known small lever/roof overlap, unresolved clutch-contact modelling and simultaneous-closing-data race are not repaired by a frame redesign. The 0.1 Nm physical requirement is still unqualified.

## Mechanism and model

WRITE is on the opposite X side of the memory actuator. The orange carriage carries the integral cam; the blue carriage contains the locking pockets; the base supports the bolt guide. Input/output and moving-part labels remain in the viewer.

The viewer uses an angle-driven rigid model: the worm drives carriage translation when the lever restrains the reaction gear; endpoint release permits reaction-gear rotation. The twelve input transitions are represented by 72 sensitivity cases. Assumed clutch take-up of 0/30/60 degrees and axial pickup positions are not measured backlash bounds. They do not establish force, wear, friction or deformation behaviour.

Both clutch connectors use separate 5L axles to avoid their internal stops. The drive bridge uses native 59443 geometry. See `Axle stop checks.json` for nominal axial clearances.
