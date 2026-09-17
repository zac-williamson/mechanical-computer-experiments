# Adder handoff

Current model: Viewer.html. Four accessible stages: B XOR SU, A XOR B′, Sum = P XOR Cin, and Cout = P ? Cin : A. The sum shaft line is raised 16 mm; no stages are stacked.

Implemented: shared A shaft, coaxial B′ connection, P/Cin routing, current two-piece multiplexer carriages, two-or-more-pin fixed supports, through-base pin holes, labelled ports, individual print orientations and three draft print plates.

Current base: 176 × 178 × 8.2 mm. Static hardware envelope: 192.6 × 182 mm in plan, 76.2 mm high. Base area is 8.9% smaller; overall plan area remains about 1.5% larger than the old adder. This is not a completed minimum-footprint design.

Passed: all 16 Boolean cases; 23 gear-pair tooth-profile checks over 144 angular samples; print mesh/orientation checks; latest base mounting checks. Earlier printed-only travel checks passed.

Unresolved: sampled travel found X R-P-shaft intersecting P Right carriage half, and S O-shaft intersecting C Right carriage half, each about 1.025 mm. These findings are retained in Native clearance checks.json. That run completed 36 sampled states and 1,296 candidate part-pair checks before the latest trims. Latest axle/base trims have not had a complete repeat check. Physical fit, friction and torque are untested. Do not treat the draft STLs as print-ready.

Sources/ contains the generator and input snapshots. Rebuild with its documented dependencies. Saved separately in adder-draft; the existing register, multiplexer and earlier adder are retained.
