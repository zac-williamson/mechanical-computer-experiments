# 1-bit adder

Single-level shared-actuator layout: 292 × 111 mm base, 39 printed parts, 30 gears, three actuators and four clutches.

B′ = B XOR SUB; P = B′ XOR Cin; SUM = P XOR A; Cout = P ? A : B′.

Cin drives only its worm actuator. P drives the shared carry/sum actuator; the pinned carriage tie operates both clutches. The carry data drive comes from local A or B′, so Cin does not power a chain of downstream clutch gears.

`Viewer.html` includes axle labels, part selection, pan and axes. The common carriage, lever and bridge meshes come from ../multiplexer. Carry and sum have local shaft-clearance and tie attachment adaptations. The sum fork has no independent lever or upper actuator frame.

This is an engineering prototype, not a print release. Logic, gear pitch, tooth-phase, mounting and coupling checks are supplied. `Travel checks.json` includes conservative axle-envelope findings; an actual-mesh check also identifies a small pivot-axle/carriage contact near the positive mechanical stop. That must be resolved before printing this assembly. A complete print-orientation audit and loaded shared-carriage test are also outstanding. No print layout is supplied.

Rebuild with Sources/build.py, Sources/gear_check.py, Sources/apply_phases.py, Sources/publish.py, then the audit scripts. Python dependencies are in Sources/requirements.txt; LDraw is loaded from the local installation. Do not change stage spacing independently of stock axle lengths and connector positions.
