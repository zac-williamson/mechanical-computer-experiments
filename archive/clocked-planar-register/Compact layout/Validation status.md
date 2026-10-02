# Current compact-register evidence

Geometry: `b9269d5256fb8a977353c3f6b3a8afbf76b6aba4205329548a5e14fee67b4e86`.

The current model is **not fully mechanically qualified**. The nominal geometry and sequencing checks below have been run; the unresolved native worm geometry and unmeasured loaded behavior prevent a full-machine sign-off. Neither successful animation nor a zero sampled-intersection count establishes force, tolerance or endurance performance.

## Corrections incorporated

- Corrected the master/slave power-path rotation signs and checked actual Q shaft velocity. Both settled Q directions have unit speed magnitude relative to POWER.
- Replaced fixed backlash delays with native dog angular windows. Disconnected input forks can wait for alignment through 4 mm of one-sided float; withdrawal remains positive.
- Returned gate gears alongside their LEGO connectors and offset only the clutch fingers. Their rear guides retain their compact locations. Cleared the gear sweeps from the fork shoulders.
- Added opposing nominal thrust restraints to all 16 routing shaft stacks, corrected connector insertion lengths and eliminated native pin-collar clashes.
- Turned the D input header mesh around its receiving shaft toward +Y. This clears the full master ring sweep without an extra gear or greater overall depth. Minimum radial envelope gap for the checked D parts is 3.82 mm, or 3.32 mm after the assumed 0.25 mm error on each part.
- Replaced the WRITE-side data shaft with a stock LEGO 4L axle. It has 6 mm nominal overlap in the 8T header gear and 8 mm in the receiving gear, leaving 4 mm between independent shaft ends. No shaft pairs have a nominal end gap below 0.5 mm in the current screen.
- Split the rear clock-pivot bearing from the base and attached it with two non-collinear friction pins. This removes the roughly 31 mm unsupported base ledge. Reinforced the bridge around the rear bush within the existing Y envelope.
- Selected print orientations using layer connectivity before face area, retaining bearing-flat carriage orientations and the pivot bridge's Y-positive flat orientation.
- Updated the animation, port labels and axes. The viewer uses the same corrected poses as the printed-motion checks. Repeated publication no longer duplicates the animation code.

The complete reference-pose envelope remains **278.4 × 127.2 mm in XZ**, **50.42% smaller** than the earlier assembly; Y depth is **76.23 mm**. This is not a swept packaging envelope.

## Current checks

| Check | Result | Scope |
|---|---|---|
| Defined transition behavior | 112 nominal cases; no wrong defined Q, modelled stalls or driven inserted bolts | All 56 distinct D/WRITE/CLK transitions, both initial Q values; quasi-static assumptions |
| Timing sensitivity | 1,120 cases; zero wrong defined Q, modelled stalls or driven inserted bolts | Eight running-phase offsets and two additional pickup limits; no inertia or measured friction |
| Printed moving solids | 56,112 frames, 10,352 unique poses, 53,841 narrow checks; zero intersections | All printed pairs, including levers; finite sampling |
| Native/native surfaces | 36,916 unique assembly poses from all 56,112 frames; 105 contact pairs retained for interface review | No pair whitelist; includes intended keyed/coaxial hardware interfaces and unresolved worm contacts |
| Native clutch dogs | 41,253 distinct dense relative poses; zero surface crossings | Every 2-degree integration step; actual shallow-entry surfaces, not a full-load test |
| Native/printed rotation envelopes | Four potential contacts, all reaction-gear/lever working interfaces | Whole revolutions at sampled relative poses; working interfaces resolved separately by the actuator contact solver |
| External gears and animation | 15 external meshes, unit-magnitude ratios; 105 direction checks pass | Mesh profiles and rotation signs |
| Shaft supports | No unsupported shaft in the 23-shaft radial screen | Presence of supports, not their load capacity |
| Axial restraint | All 16 routing stacks have opposing nominal thrust faces | Does not establish bush/connector grip |
| Printed grounding | No fixed printed component lacks a pin path to the frame | Connection geometry, not pin strength |
| Fork float | 780 states and 8,148 checks; zero printed intersections | Includes full 4 mm lost motion and both bar ends |
| Cam/disconnect order | Positive-withdrawal implication holds continuously; minimum screened bolt clearance about 0.384 mm | Assumed ±1 mm pickup, ±0.25 mm cam phase and 0.2 mm vertical error; excludes loaded deformation |

Native/native results were retained after the pivot reinforcement only after exact comparison established that every native triangle, native metadata other than storage offsets, and every recorded operation case was unchanged. That scope proof is recorded inside `Native motion surface screening.json`. Printed/printed and native/printed checks were rerun for the reinforced bridge.

Twenty-four simultaneous input/rising-clock cases violate setup/hold and do not receive a guaranteed Q value. Four nominal cases leave the master outside a bolt pocket. These are not qualified asynchronous captures. The model also assumes instantaneous spring insertion after angular alignment and stationary disconnected shafts; rapid repeated clocks, coasting and loaded release remain unqualified.

The static construction-reference pose puts both carriages in mid-travel with bolts lowered and is not an operating pose. Its two bolt/carriage intersections are reported separately; operating checks use the actual bolt/carriage relationship.

## Load and print findings

The lossless 0.1 Nm blocked-clock bound gives 200 N at the actuator and 80 N at the common bar. The two printed amplifier plates have estimated peak nominal bending stresses of 8.38 and 14.10 MPa, with approximately 0.146 mm ideal relative output deflection. These are variable-section beam calculations, not contact FEA or measured performance.

The rear pivot bridge was reinforced after its initial 70 N screen gave approximately 34 MPa. The final bridge is screened at **120 N**, assigning the entire pivot reaction to it: **16.37 MPa** peak nominal bending and **0.154 mm** ideal central deflection. The calculation omits joint compliance, offset-pin torsion, local bearing/notch effects, creep and fatigue. The assumed modulus is 1,890 MPa; supplier specimen properties are not part allowables. LEGO shaft/pin material allowables and loaded grip are not established.

All generated printed parts are connected, watertight solids. At 0.2 mm layers, only the common fork/cam bar retains disconnected-layer features (three in the selected screen); it requires support or a further print-specific redesign. The base halves and flat pivot bridge have no disconnected layers. The former large base ledge is removed, but pin-hole bridges, collar lips, bearing finish and fit still need print-process confirmation. No Bambu Studio operation was used.

## Blockers to full mechanical qualification

1. **Inherited worm/reaction geometry:** the 32,400-combination assembly-phase screen found 23 sampled wheel poses with no nonintersecting sampled worm phase. This means the supplied meshes cannot certify those poses. It does not establish that real LEGO gears jam. Trimming the meshes or silently exempting them would not resolve this. Confirmation with the already-printed multiplexer, or more faithful physical tooth geometry, is needed to distinguish a mesh approximation from an actuator-spacing problem.
2. **Loaded dynamics and strength:** no measured clutch withdrawal force at 0.1 Nm, band force/preload, joint grip, motor acceleration/coasting or part-level material strength is available. The quasi-static model cannot validate these by assigning arbitrary favorable values.
3. **Continuous tolerance and manufacturing qualification:** finite motion sampling and selected analytic bounds do not cover every intermediate all-part tolerance combination. Common-bar supports, hole bridging and actual printed fits remain to be qualified.

`Qualification summary.json` therefore remains fail-closed. A full working/print-release claim is not supported by the present evidence.
