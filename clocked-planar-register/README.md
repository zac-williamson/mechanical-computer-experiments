# Clock-triggered 1-bit register — separate development

This directory is reserved for a new clock-triggered register based on the preserved planar latch in `../planar-register/`. No clocked CAD design has been validated or exported here yet. The existing `../register/` workstream is separate and untouched by this snapshot.

## Design requirements

- Target Ben Eater 8-bit SAP register behavior; verify clock edge, write-enable and bus timing against the reference circuit before committing to an architecture.
- Capture on a rising clock edge; retain Q between writes. Include write enable appropriate to the target register.
- Evaluate a master/slave arrangement with mechanically enforced non-overlap: disconnect each storage drive before locking it, and unlock before reconnecting.
- Keep rotational logic: 0 clockwise, 1 anticlockwise, using a consistently defined viewing direction.
- Retain LEGO gears, axles, clutch parts, axle connectors, bushings and friction-pin assembly. Printed parts use standard Bambu PLA, 0.4 mm nozzle; target 0.1 Nm on all input/output axles.
- Compact, approximately coplanar, open XZ view. Preserve bearing-friendly print orientations.
- Check all input combinations and transitions, early clutch engagement, backlash, setup/hold requirements, free-body restraint, swept collisions and plausible deflection. Separate geometric evidence from physical qualification.

All new CAD, source, viewers and print files belong here. Use independent copies of any starting geometry; never make the clocked build publish into the planar-register directory. Do not use Bambu Studio CLI.
