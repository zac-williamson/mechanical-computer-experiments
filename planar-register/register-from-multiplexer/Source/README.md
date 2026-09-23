# Current integrated-cam revision

`build_integrated_cam.py` is the current integrated-cam build/finite-check/publish entry point. Run it with `work/register-print-env/bin/python` from the workspace root. It builds in `work/integrated-cam-development`, then publishes `Planar register`. The earlier direct-cam generators describe superseded geometry and should not be run to update the current design.

Inputs are the frozen `work/register-before-left-cam` assembly and viewer, the mux reference at `work/register-mux-reference/multiplexer`, and the preserved baseboards in `Superseded planar parts`. These local inputs are required. No unrelated register design is used. Existing part identities/relative gearing are inherited; the revision is not a physical torque qualification.

The current Python source generates assembly STLs, metadata, a self-contained viewer, oriented print files, fit-test coupons and finite clearance/timing/loading reports. The archive `Superseded cam revision before integrated carriage` preserves the prior published design. The build deliberately asserts against detected unintended contacts but cannot certify a physical 0.1 Nm requirement or solve the single-latch closing-data race.


The viewer now uses `coupled_register.py` and `coupled_actuator.py`. The actuator advances shaft angle and solves reaction-gear/lever/roof contact; carriage travel follows the worm-lead constraint. `coupled_pose.py` defines the same joint transforms as the viewer. `check_coupled_model.py` checks the motion invariant and phased lever contact; `check_coupled_assembly.py` scans all displayed part pairs across all exported frames. Legacy independent-pose reports are not certificates for the current animation. Native/native open-mesh contacts and real load/friction remain unresolved.

## Internal axle stops corrected

The continuous axles through both 26287 clutch joiners were real material intersections. They are replaced with separate 5L LEGO axles. The actual 59443 bridge connector replaces the filled placeholder. See `Axle stop checks.json` for axial clearance and engagement, and `Coupled assembly contacts.json` for the current all-part sweep. Older prescribed-pose reports are superseded and do not certify this revision.
