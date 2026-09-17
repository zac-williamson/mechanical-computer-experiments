# Register and ALU assembly

Use Ben Eater-style AI, AO, BI, EO and SU requests, plus an external clock and continuous mechanical power. Logic values are rotational directions, not stationary shaft positions. This is a CAD candidate, not a print release.

AI receives the bus into A. AO enables A onto the bus. BI receives the bus into B. EO enables the isolated held arithmetic result onto the bus. SU complements B in the adder. For the least significant bit, subtraction also requires Cin = SU. The controller must not enable AO and EO together. EO and AI may be asserted together: AI is a receiver.

The clock-qualified sequence is: allow operands and arithmetic to settle; capture into H with A and B locked; disconnect H's input and lock H; enable EO if requested; qualify AI and BI; lock the receiving registers; release EO. Capturing H does not depend on EO or the bus.

The four physical cam selectors are ordered AI, capture, EO, BI to avoid crossings between their control routes. Their outputs operate the existing worm actuators. AO and SU retain direct request inputs. The clock is an external input; no hand-cranked instruction cycle is assumed.

Cam angles, in degrees: capture falls 50–80 and rises 330–360; EO rises 110–140 and falls 285–315; AI and BI rise 165–195 and fall 235–265. These are proposed dwell intervals, not load-validated timings. Start bench commissioning slowly; clock dwell must be established from loaded physical tests.

The core has straight X→P and P→sum/carry shaft connections. B faces inward with its unused read mechanism removed. Carry-in feeds the sum and carry data shafts rather than a worm selector. Every new core axle is at most 12 studs long. The utility-routing model remains separate from the validated core audit: structural support for its bearing cheeks, common base tiling, complete travel collision checks, and loaded timing validation remain outstanding.

## Viewing and build status

Open [Viewer.html](Viewer.html) for A, B, the adder, isolated result holder, sequencer and routed shafts. The envelope is approximately 693.5 × 384 × 82.5 mm in wall width, height and depth. Carry-out is live rather than captured.

This is **not a print release**. The utility bearing cheeks still need a connected rigid support frame, additional power-transfer bearings, attachments, common base panels, printable subdivision and collision checks incorporating those structures. Assembly-position STLs are retained as design data, not an all-parts print layout. Printed routing couplers remain present and have not been replaced by native LEGO connectors.

The standalone multiplexer, register and arithmetic board in this repository are not yet integrated into this assembly. Their meshes are not drop-in replacements.

Digital checks cover all 16 arithmetic input combinations and sampled shaft phase, carriage, utility route and cam geometry. They do not validate load-dependent timing, printed fit or physical operation. Viewer motion is an inspection aid rather than a contact simulation.
