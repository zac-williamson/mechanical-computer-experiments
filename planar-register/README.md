# Planar register — preserved latch prototype

[Assembly viewer](register-from-multiplexer/Planar%20register/Viewer.html) · [Print layout STL](register-from-multiplexer/Planar%20register/Print%20layout.stl) · [Design and checks](register-from-multiplexer/Planar%20register/README.md)

This preserves the planar write-enabled latch, including the detachable pinned guide and restored rear bolt-retaining lips. It is not an edge-triggered register. The print layout has 20 printed parts. Geometry checks do not establish physical load capacity; the whole-register closing-data race remains unresolved.

The separate clock-triggered development is in `../clocked-planar-register/`. Do not overwrite this prototype with that design.

## Sources and reproducibility

The original relative directory structure is retained so source scripts keep their paths. `register-from-multiplexer/Source/` contains the generators, viewer publisher and checks. `work/register-before-left-cam` and `work/register-mux-reference/multiplexer` are frozen build inputs, not alternate current multiplexer designs. `Snapshot hashes.json` records byte-exact copies from the working design.

Run `python restore_validation_workspace.py` to reconstruct the validation working directory from the published snapshot. Install `requirements.txt` into an isolated environment. The source README describes the full development build; native-part regeneration additionally requires the BrickLink Studio LDraw library at its documented path. The full build retains assertions for unresolved whole-assembly issues and is not advertised as passing end-to-end.

The focused checks for this revision are `check_bolt_y_restraint.py`, `check_detachable_guide.py`, `check_support_free_prints.py`, `check_frame_printed_clearance.py`, `check_frame_native_contacts.py` and `check_restored_bearings.py`. Published reports accompany the model. No Bambu Studio slicer was used.
