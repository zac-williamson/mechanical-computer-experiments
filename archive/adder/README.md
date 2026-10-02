# 1-bit adder

Single-level shared-actuator layout: 292 × 111 mm base, 39 printed parts, 30 gears, three actuators and four clutches.

B′ = B XOR SUB; P = B′ XOR Cin; SUM = P XOR A; Cout = P ? A : B′.

Cin drives only its worm actuator. P drives the shared carry/sum actuator; the pinned carriage tie operates both clutches. The carry data drive comes from local A or B′, so Cin does not power a chain of downstream clutch gears.

`Viewer.html` includes axle labels, part selection, pan and axes. The common carriage, lever and bridge meshes come from ../multiplexer. Carry and sum have local shaft-clearance and tie attachment adaptations. The sum fork has no independent lever or upper actuator frame.

This is an engineering prototype, not a print release. The three actuator carriages have a rounded inside-roof relief for the fixed pivot axle, sized for the full 8.675 mm stroke and 0.5 mm nominal radial clearance. Bearing positions, travel, gear count and external dimensions are unchanged.

Current checks pass: all 16 arithmetic cases, 12 gear-pitch checks, fixed supports, mounting and coupling pins, conservative native envelopes, and carriage travel. `Pivot clearance checks.json` checks a continuous swept axle envelope and verifies that the relief does not materially remove the lever's working contact surfaces over the prescribed switching trace (0.001 mm³ mesh tolerance).

A complete print-orientation audit and loaded shared-carriage test remain outstanding. No print layout is supplied. CAD checks do not establish printed friction, wear or loaded reliability.

Rebuild with Sources/build.py, Sources/gear_check.py, Sources/apply_phases.py, Sources/publish.py, then the audit scripts, including Sources/pivot_clearance_check.py. Python dependencies are in Sources/requirements.txt; LDraw is loaded from the local installation. Do not change stage spacing independently of stock axle lengths and connector positions.
