# Rebuilding the prototype

Install requirements.txt in a Python environment, set LDRAW_PATH to an installed LDraw library (default: Studio 2.0 on macOS), then run `python Sources/rebuild.py` from the package folder. This regenerates geometry, uses the included Gear checks.json phase solution, lays out the print plates and rebuilds the assembly viewer data/image.

The input folder contains the specific multiplexer meshes and old adder gear records used by the generator. No other checkout is needed. LDraw geometry is not bundled.

After geometry changes, run gear_check.py to recompute tooth phases, then apply_phases.py. The individual fixed_check.py, moving_check.py, native_check.py, mount_check.py, print_layout.py and logic_check.py scripts write validation reports beside the assembly. Checks are sampled and have the limits documented in the main README. Serve the package with a local HTTP server to view Viewer.html.
