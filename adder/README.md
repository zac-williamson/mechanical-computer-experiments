# One-bit adder/subtractor

The 200 × 172 mm board implements X = B XOR SU, P = A XOR X, Sum = P XOR Cin, and Cout = (A AND X) OR (P AND Cin). For least-significant-bit subtraction, drive Cin = SU.

The supports use 60 pins, minimum centre spacing 8 mm, with at least two pins per connected support and the original two-pin pair at every bridge pillar.

## What to print

The three Print plate STL files contain all 30 printed parts exactly once: one base, 21 frame pieces, four carriages and four levers. LEGO gears, axles, pins, hubs, bushings and connectors are excluded. Individual oriented STLs are also in Print parts. The assembly viewer uses assembly-position STLs, which are not print layouts.

## Required printing setup

These are unsliced STL layouts. **Supports are NOT embedded in these files.** Enable external supports for the frame and carriage parts and use a brim. They must not be printed with supports disabled.

Frame pieces are oriented diagonally so both perpendicular axle-hole directions are 45 degrees from vertical, avoiding a horizontal round-hole roof. Mounting-pin holes have a teardrop roof in this print orientation. Round axle bores have not been replaced with printed cross-axle connectors. Carriage guide holes print vertically. Lever bodies print on their flat faces.

Keep support out of the bearing bores; supports are for the external islands, feet, stops and forks. Remove external supports before assembling. The plate spacing is 6 mm; check generated brim/support footprints for overlap before printing. These layouts do not replace a layer-by-layer slicer preview.

The layouts lie within a 256 mm cube, the stated P2S build volume in the [Bambu Lab specification sheet](https://csm.bblcdn.com/hub/2ddc44d3a72443ba914efe0f963fbbc5.pdf). Geometry and bed fit have been checked; material-dependent fit, stiffness and successful physical operation have not been tested.

## Assembly

Use the viewer’s individual-part selector to locate Frame 01–Frame 21. Each frame piece attaches to the base using at least two vertical LEGO pins. Preserve the supplied LEGO hubs and 2L connectors; no printed drivetrain couplers are included.

### Plate 1

Single arithmetic base

### Plate 2

Frame 03, Frame 11, Frame 01, Frame 09, Frame 15, Frame 05, Frame 13, Frame 14, P Guided actuator carriage, S Guided actuator carriage, X Guided actuator carriage, C Guided actuator carriage, Frame 04, Frame 12

### Plate 3

Frame 07, Frame 16, Frame 17, Frame 08, Frame 02, Frame 10, Frame 18, Frame 19, Frame 20, Frame 21, P Direct lever and band cleat, X Direct lever and band cleat, S Direct lever and band cleat, C Direct lever and band cleat, Frame 06

## Status

The supplied mount and print geometry checks cover nominal spacing, connected parts and plate bounds. Fit, stiffness, friction and switching require physical testing. This board is a standalone model; it is not installed in the complete assembly viewer.
