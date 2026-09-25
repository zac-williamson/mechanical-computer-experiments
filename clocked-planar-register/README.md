Current connections: [LEGO friction pins; no screw connections](Wall%20register/Friction-pin%20connections.md).

Current axle-hole manufacturing review: [bed-face correction and remaining exceptions](Wall%20register/Axle-hole%20printing.md).

Current frame manufacture: [flat-bed frame revision](Wall%20register/Flat%20frame%20printing.md).

Latest bearing revision: [removable two-pin bearing walls and relocated POWER train](Wall%20register/Bearing%20bracing%20revision.md).

# Clocked planar register

[Open the current compact model](Compact%20layout.html). It retains the planar register's actuator cores and uses four worm actuators, five clutch rings and eight elastic bands. The earlier full-size assembly remains in `Viewer.html`; the unclocked planar register and multiplexer are separate, preserved designs.

The compact model has direction-coded D, WRITE and CLK inputs, continuous POWER, and Q on the opposite X side. Its two storage stages form a master/slave register. WRITE selects D or feedback Q before the master, avoiding a gated-clock edge when WRITE changes.

- [Current arrangement and sequencing](Compact%20layout/Design.md)
- [Validation evidence and unresolved requirements](Compact%20layout/Validation%20status.md)

The animation uses the same contact-corrected poses as the printed-part collision checks. Native clutch angular windows determine engagement and reversal backlash; a fixed time delay is no longer used. The animation is a quasi-static model, not a physical load test.

Every geometry report must match the current geometry hash. Historical full-size reports, old fixed-delay traces and static assembly-reference poses do not qualify the current compact mechanism. The qualification summary fails closed when required evidence is absent or stale.

## Modular wall-register candidate

[Open the separate shared-control model](Wall%20register.html) for one, two, four or eight vertically arranged rows. It uses rotated shared CLOCK/WRITE actuators with direct rod outputs, a 16T CLOCK header, coplanar storage stages, roller followers and a coordinated chassis. [Design notes](Wall%20register/Design.md) and [current validation status](Wall%20register/Validation%20status.md) distinguish the passing CAD screens from the remaining mechanical work. The repeated bit is thinner with about 4.6% less face area than the original compact model; row pitch is 112 mm. Controller moving printed volume is about 29% lower than the previous wall candidate. Loaded force and printed strength remain unmeasured; development STLs are not a print release.

The wall candidate now uses a shared reversing shaft and a rebuilt shaft-based bearing schedule. See [transmission redesign](Wall%20register/Transmission%20redesign.md). Its eight elastic loops follow their moving anchors in the viewer.

## Reproduction order

Use the existing `work/register-print-env/bin/python` environment from the workspace root. Sources are under `current-designs/clocked-planar-register/Source/`.

1. `compact_layout.py` generates parts and the static viewer.
2. `check_compact_gear_phases.py` and `compact_clutch_profiles.py` derive native gear and clutch phases.
3. `compact_phase_operation.py` simulates all 112 transition/initial-Q cases; `compile_compact_contacts.py` resolves actuator contact poses.
4. Run the geometry, motion, retention, timing, load and print screens listed in the validation status.
5. `publish_compact_motion.py` publishes the animation; `compact_qualification_status.py` reports remaining qualification requirements.

Printed structure, materials, band preload, motor acceleration, joint grip and loaded friction are not established by the nominal motion trace. Do not treat a successful script exit as a working-machine certificate.
