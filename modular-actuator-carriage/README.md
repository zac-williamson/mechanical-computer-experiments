# Modular actuator, carriage and locking bolt

Snapshot of the current tested-in-CAD prototype, saved 2 October 2026.

- [Model viewer](Viewer.html), including motion, exploded, print and tiled views.
- [Complete print layout](Print%20layout.stl).
- [Carriage-only print layout](Carriage%20print%20layout.stl).
- Individual `print.stl` files use the recorded print orientations; other component STLs use assembly coordinates. Units are millimetres.

The module tiles on a 72 mm X by 64 mm Z pitch. X follows rods/axles; Z indexes rows; +Y points toward the rear/base. Rod end coupling holes have 7 mm spacing (X = ±32.5 and ±39.5), with two separate pins per joint. Revised rod ends must mate to revised ends. One-pin joints have not been physically validated.

Recent changes include reinforced carriage fork struts, removal of a redundant carriage arm and square shoulder, rounded locking-rod cam-extension tips, and closer rod coupling holes.

## Validation and limitations

The included latest clearance report covers 26 prescribed motion poses; the tiling report covers neighbouring modules and connector insertion. These are geometric checks, not physical force, wear or strength tests. Rod print checks compare layer growth with the previous geometry; existing pin-hole roof overhang flags remain. The complete assembly is NOT certified support-free. Review the slicer preview before printing, particularly carriage overhangs and hole roofs.

[Build and validation instructions](Source/README.md) provide a self-contained rebuild from bundled mesh inputs, plus the full historical Python sources. This is a mesh-based CAD workflow, not a fully parameterized model. `Model.json`, component STLs, and the self-contained viewer preserve the current geometry. Development history is summarized in `Compact module notes.md`; historical claims there should be read with their revision context. `Snapshot hashes.json` records every exported file.
