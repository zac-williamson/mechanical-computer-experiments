# Rebuilding and validating this module

Run from the repository root on macOS/Linux (the throttler uses POSIX signals). The captured environment uses Python 3.14.

```sh
python3.14 -m venv .venv-modular
.venv-modular/bin/pip install -r modular-actuator-carriage/Source/requirements.txt
.venv-modular/bin/python modular-actuator-carriage/Source/work/run_queued_cool_job.py modular-actuator-carriage/Source/rebuild.py
```

The serial runner limits geometry work to one worker at 10% duty and guards memory. It fails closed if the installed Manifold library lacks the expected TBB control API. Output goes to `Source/build/`; committed print files are never overwritten by this command.

## Inputs and editing

`Inputs/` contains the complete assembly and print meshes, viewer animation, and coupling hardware immediately before the final rod-hole-spacing revision. It is intentional source geometry, not a download cache. `rod-pin-spacing/build.py` applies the last hole relocation and rebuilds Model.json, the viewer, individual rod STLs and the full plate. Change the explicit 0.5 mm shifts consistently in rod cuts and coupling-pin placement if editing that parameter. The other components are preserved from their bundled mesh inputs. This design was developed by successive mesh booleans; it is not a fully parameterized CAD model.

`History/` preserves ALL 307 original project Python scripts, including builders, analyses, packaging and validation. These are historical reference, not the supported entry point: many retain original local paths or depend on obsolete candidate snapshots. Use them to understand the geometric construction, then port the desired operation to the current pipeline with the current Inputs. Do not run an old publisher against the current design. No historical private workspace paths are needed for the supported rebuild.

`Switching trace.json`, `native_envelopes.py`, the single-band geometry and the seam-cleaning helper are bundled dependencies. Native hardware meshes are already embedded in Model.json; LEGO Studio is not needed to rebuild or validate the current revision.

## Tests and limits

Rebuild runs 26 mechanism poses, 2x2 tiling and connector insertion, carriage installation around the preassembled actuator, watertightness, and print-oriented layer-growth comparison. Existing rod hole-roof overhang flags are retained and printed; the comparison is not a support-free certificate. Contact exclusions and conservative native-part envelopes are documented in validation code. Test reports establish geometry only, not physical load capacity or reliable operation.

Never infer physical validation from `passed`. Read module README.md, AGENTS.md and Compact module notes.md before editing.
