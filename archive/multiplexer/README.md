# Multiplexer — current design

This is the sole current multiplexer design. Open [Viewer.html](Viewer.html) for the complete assembly and switching animation. The root-level STLs are generated outputs. Run `python Source/build.py` to regenerate the ten parts, hardware data, viewer and print layout. LEGO geometry is in hardware.npz with hardware.json identifying components.

The clutch axle is at Y=10.2 mm, Z=0; the worm axle is at Y=10.2 mm, Z=16 mm. The 16 mm centre distance accommodates two standard LEGO 16-tooth gears. That optional gear pair is not installed in this assembly. Cheek mounts and the band anchor remain on +X. The carriage consists of two printed parts joined by two LEGO friction pins.

## Printing and assembly

Use [Complete print layout.stl](Complete%20print%20layout.stl); its ten printed components are separated and placed on the bed in their intended orientations. LEGO hardware is excluded. Print layout manifest.json records source hashes, orientations through printed-parts.json, and bed contact. Use standard PLA with a 0.4 mm nozzle. No Bambu Studio CLI is used. A bed-contact check is not a slicer-based support assessment.

Fit the rubber band before closing the bearing cheeks. Install the worm and carriage on the baseboard guide before closing the side frames. The front carriage joining pin stays below the clutch gears; the rear joining pin follows the relocated worm bearing.

## Maintenance and validation

Install `Source/requirements.txt`, then run:

- `python Source/build.py` — regenerate the current design in place.
- `python Source/build.py --output /tmp/multiplexer-build` — build into a separate empty directory.
- `python Source/verify_build.py` — clean rebuild; require byte-identical part STLs and print layout against the accepted revision.
- `python Source/check.py`, `python Source/check_envelopes.py`, `python Source/check_fork.py` — geometry checks.

`Source/generate_parts.py` contains the recovered final construction operations. Its exact mesh and hardware inputs are committed in `Source/construction-inputs/`, with hashes. The original workflow was incremental: it operated on earlier geometry. Those inputs are build dependencies, not alternative current designs, and are deliberately retained rather than pretending the workflow was fully parametric. No Trash, external project, or Git-history lookup is needed to build.

`Source/construction/` preserves the recovered upstream scripts that originally created those inputs. They use their original development paths and are provenance, not supported entry points; use `Source/build.py`. This repair restores the original reproducible build, without refactoring its geometry. `Source/accepted-output-hashes.json` records the approved outputs; update it only after an intentional, reviewed geometry change.

The small `Source/Clutch interface reference.stl` is a contact-test fixture, not another carriage or design.

Checks cover fixed/moving printed interference, sampled switching poses with native LEGO meshes, full-rotation clearance envelopes, the elastic band sweep, worm/guide insertion, and added material around the original clutch interface. Inspect the JSON reports; these scripts report findings rather than certifying load capacity. CAD checks do not prove low friction, wear life, or 0.2 Nm endurance for this revision. The physically tested predecessor remains available in Git history.
