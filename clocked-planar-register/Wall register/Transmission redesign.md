Bearing plates and attachments have since been revised; see [Bearing bracing revision](Bearing%20bracing%20revision.md).

# Rebuilt transmission and bearing layout

The transmission is now built before its supports. Legacy `Bearing support` wall solids are no longer imported. Actual shaft assemblies determine complete bearing rings and the webs joining them to the chassis. The storage gears and clutches retain their functional assignments; the D input header is repackaged to clear the WRITE linkage.

## Changes

- One shared +POWER shaft supplies both reversing branches. A keyed coupling joins its two 10L axles, with 7 mm engagement at each end. The second reversing pair at X=138 is removed. Master and slave outputs remain independent.
- The incoming POWER route changes from two 96 mm axles to one local 32 mm axle. The opposite-running main distribution changes from two axles totalling 160 mm to one 96 mm axle.
- The shared reverse branch uses two 80 mm axles instead of two separate 72 mm idlers. Its greater length is intentional: it replaces duplicated gearing.
- The controller CLOCK input changes from 48 mm to 32 mm and shifts toward its gear. This avoids the WRITE carriage over its operating travel.
- The D header moves 4 mm along its axis and to the other side of its mating gear. Both gears remain 8T. Longer local D shafts, selected-data output, Q left stub and slave worm/stub provide room for bearings, stops and keyed engagement.
- Q feedback still spans the bit. With the retained takeoff and selector gear locations, its 242 mm gear separation remains necessary; the 256 mm axle now has three scheduled bearing locations.
- Transmission axle material totals **1,272 mm rather than 1,440 mm**, a reduction of **168 mm (11.7%)**, across **20 rather than 22 axle pieces**. This excludes pivot pins, bushes, couplers and printed parts; it is not a total-mass saving.
- The two controller actuator return bands, previously omitted by the inheritance filter, are restored. All eight elastic loops now animate using their inherited anchor paths.

## Bearings and retention

The schedule assigns **32 bearing lands to 15 transmission shaft assemblies**, including the rotated controller. The nominal bore radius is 2.65 mm; the outer bearing-ring radius is 5 mm. Most bearing lands are 3.2 mm long; the slave worm assembly uses 2.4 mm lands for its confined clutch-side bearing arrangement. Supporting webs are 4 mm wide in their plane. These dimensions are not a printed-strength rating.

The route search considers sampled poses from all 112 inherited operating cases, including WRITE changes, rather than only a single rising-clock example. It does not carve holes through existing bearing walls. Rings remain complete; supporting webs route around the mechanism and join deliberately placed chassis rails. Bearings share stations where practical, including the slave-side shaft cluster. The WRITE pivot's chassis rib runs downward to avoid the CLOCK follower.

Every transmission assembly has opposed axial stops assigned, either at one bearing or across separate bearings. Existing retainers are reused where their faces match; added half-bush stops nominally leave 0.2 mm at each stopping face. Their grip, as well as keyed-hub/coupler grip, still depends on actual hardware fit. The stop schedule does not establish measured preload or stiffness.

[The bearing schedule](Bearing%20schedule.json) gives every shaft, bearing station, supporting-web path and stop assignment. [The independent support check](Bearing%20and%20retention%20checks.json) tests the complete bearing annulus against the exported chassis, verifies the bore is open and measures axle engagement through each land. It also checks the shared reverse coupling's physical overlap. Retained actuator pivots and moving carriage pins use their existing working supports.

## Assembly order to review on the single-bit prototype

1. Print the coordinated chassis pieces and separate front actuator cheeks. The existing chassis joint and front-cheek joining pins remain assembly interfaces. Development STLs are in assembly coordinates; orient/support them deliberately in the slicer.
2. Before feeding each axle through its first bearing, place the gears, clutch connectors and any inner half-bush stops between the walls in the order shown by the model. Insert the axle progressively through these parts and the next bearing; fit outside stops last. Do not press an axle through an assembled, misaligned gear train.
3. On the shared reverse shaft, seat the central coupling on the first axle, then introduce the second axle from the opposite end. The designed engagements are 7 mm each, with a 2 mm gap between the axle ends.
4. Fit the storage carriages, actuator pins, removable cheeks and eight elastic loops. Two loops provide storage-bolt returns, two bias the clocked forks, and four return the actuator levers. Confirm each loop sits on both of its named anchor surfaces throughout travel.
5. Set axial clearances and verify hand rotation before applying powered CLOCK/WRITE motion. The model contains the stops; it does not measure how firmly the hardware grips the axles.

This is a CAD assembly sequence for review, not a demonstrated physical assembly. Bearing fit, layer orientation, access with real fingers/tools, elasticity, wear, stiffness and eight-bit loading remain physical qualification work. See the current [validation status](Validation%20status.md) for geometric checks and unresolved contacts. Constant engaged speed magnitude is preserved; reversal and clutch disconnection still have transients.

## Final CAD evidence

- 910 unique printed poses from 112 inherited cases: no printed intersections above the screen threshold, all printed parts watertight and individually connected. Includes an adjacent repeated row.
- 32 complete bearing lands and 15 transmission retention assignments: pass.
- 14 expected external gear meshes: correct signed speeds, unit speed magnitude and sampled tooth-phase clearance.
- No shaft-envelope crossings or coaxial end gaps below 0.5 mm.
- All 16 anchor centres for the eight elastic loops lie in their named supporting parts across the sampled cases. This establishes anchor presence, not preload or a physical band-retention test.
- 1,013 unique native-envelope poses across the operating cases leave four conservative U022/Short lever contact flags in the inherited actuators. No new transmission-bearing or axial-stop contact is flagged. These four working interfaces still need phase-resolved or physical qualification.
- Viewer inspected at one and eight rows, at the beginning and end of the example capture.
