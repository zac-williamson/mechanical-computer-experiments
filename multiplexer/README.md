# Multiplexer — current design

This is the sole current multiplexer design. Open [Viewer.html](Viewer.html) for the complete assembly and switching animation. The ten root-level part STLs are the authoritative, editable assembly-coordinate geometry; LEGO geometry is in hardware.npz with hardware.json identifying components. There is no dependency on retired designs or exploration folders.

The clutch axle is at Y=10.2 mm, Z=0; the worm axle is at Y=10.2 mm, Z=16 mm. The 16 mm centre distance accommodates two standard LEGO 16-tooth gears. That optional gear pair is not installed in this assembly. Cheek mounts and the band anchor remain on +X. The carriage consists of two printed parts joined by two LEGO friction pins.

## Printing and assembly

Use [Complete print layout.stl](Complete%20print%20layout.stl); its ten printed components are separated and placed on the bed in their intended orientations. LEGO hardware is excluded. Print layout manifest.json records source hashes, orientations through printed-parts.json, and bed contact. Use standard PLA with a 0.4 mm nozzle. No Bambu Studio CLI is used. A bed-contact check is not a slicer-based support assessment.

Fit the rubber band before closing the bearing cheeks. Install the worm and carriage on the baseboard guide before closing the side frames. The front carriage joining pin stays below the clutch gears; the rear joining pin follows the relocated worm bearing.

## Maintenance and validation

Install Source/requirements.txt, then run `python Source/check.py`, `python Source/check_envelopes.py`, `python Source/check_fork.py`, `python Source/build_viewer.py`, and `python Source/print_layout.py` from this directory. These tools use only this current design. Assembly STLs are mesh sources, not a retained parametric generator. The small Source/Clutch interface reference.stl is a contact-test fixture, not another carriage or design.

Checks cover fixed/moving printed interference, sampled switching poses with native LEGO meshes, full-rotation clearance envelopes, the elastic band sweep, worm/guide insertion, and added material around the original clutch interface. Inspect the JSON reports; these scripts report findings rather than certifying load capacity. CAD checks do not prove low friction, wear life, or 0.2 Nm endurance for this revision. The physically tested predecessor remains available in Git history.
