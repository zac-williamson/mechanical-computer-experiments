# Mechanical computer experiments

Four current designs, using LEGO axles, gears, bushings and friction pins with printed frames.

| Design | Model and instructions |
|---|---|
| Multiplexer | [Viewer](multiplexer/Viewer.html) · [Build and print files](multiplexer/README.md) |
| 1-bit register | [Viewer](register/Viewer.html) · [Build and print files](register/README.md) |
| Register cam test | [Viewer](cam-test/Viewer.html) · [Build and print files](cam-test/README.md) |
| 1-bit adder | [Viewer](adder/Viewer.html) · [Design](adder/README.md) |

The multiplexer contains the common actuator meshes. Register and adder generators reference these directly; the cam rig derives its components from the register. Rebuild in that order. There are no retained alternative assemblies or embedded obsolete actuator libraries.

These are physical prototypes. CAD checks and prescribed animations do not establish friction, wear, strength or reliable switching. The adder is an engineering layout, not a released print package. See each design's current check results.

For local viewing, serve this directory with `python3 -m http.server 8766` and open the required Viewer.html.
