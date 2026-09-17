# Compact one-level adder — review draft

**Not ready to print.** Work is parked at the current design. The travel check found two axle/carriage interferences: the X data shaft against P’s right carriage half, and the Sum output shaft against C’s right carriage half (about 1.0 mm sampled penetration). The most recent shaft and base trims have not received a complete repeat clearance check. See `Handoff.md`.

Four multiplexer-based stages form one adder/subtractor bit:

| Stage | Function | Mechanical signal path |
|---|---|---|
| X | B′ = B XOR SU | B drives both data branches; SU drives the selector. |
| P | P = A XOR B′ | Shares A's shaft with the carry stage; B′ is connected coaxially to its selector. |
| S | Sum = P XOR Cin | Reversed actuator, fed P and an inverted Cin selector. |
| C | Cout = P ? Cin : A | Reversed actuator; inverted P selects between A and inverted Cin, with branch gearing restoring the required polarity. |

For subtraction, SU = 1; the least-significant Cin must also equal SU. Other bits take Cin from the preceding bit's Cout. Cout is the ordinary binary carry (no-borrow convention during subtraction). Rotation values must be read from the same positive axis end, not from whichever axle end faces the observer.

## Layout

The base is **176 × 178 × 8.2 mm**, compared with 200 × 172 mm for the old adder: 8.93% less base area and 12% shorter along X. Including projecting hardware, the static envelope is 192.6 × 182 mm, versus 200.8 × 172 mm previously: about 1.5% MORE overall footprint. The smaller base has therefore not yet achieved the requested overall footprint reduction. This is a compact first prototype, not a proof of the smallest possible footprint.

The stages sit beside each other. None is above another. S has its shaft line raised 16 mm on extended feet so that the carry selector can pass underneath without meeting its clutch. This keeps all four actuators accessible from above.

The main simplifications are the shared A shaft and coaxial B′ connection. The two outer gear trains distribute P and Cin. Their inversion parity compensates for the reversed S and C actuators. The register's lock, cam and bolt are not needed here.

The assembly contains **40 printed components, 299 LEGO components and four elastic bands**. Seventy-three LEGO friction pins mount fixed components to the base; eight more join the two-piece carriages. All fixed supports use at least two base pins. Counts and exact part numbers are in `LEGO parts.csv` and `Assembly manifest.json`.

## Printing

Draft print layouts only; resolve the clearance issues before printing. Use **Print parts/** or the three **Print plate N.stl** files. The loose STL files in this folder are in assembly coordinates and are not print orientations.

- Plate 1: base, 176 × 178 mm.
- Plate 2: approximately 242 × 246 mm occupied area.
- Plate 3: approximately 235 × 100 mm occupied area.
- The two component plates include 6 mm spacing. Check your machine's usable area and any brim before printing; the individual files allow repacking.
- Each carriage remains two parts, joined with LEGO friction pins.
- Axle bore entrances are on the bed, with their axes vertical. Retained carriage running faces remain away from the bed.
- Mounting pin holes are through the base. The base is 8.2 mm deep, leaving the nominal 8 mm pin end 0.2 mm recessed and accessible for pushing out.
- The separate bearing coupon has 5.2, 5.3 and 5.4 mm bores, marked with one, two and three notches. Check your printer's fit before committing to the full set.

No slicer settings or machine-specific project are included. No Bambu Studio was used.

## Assembly

1. Match printed components by stage prefix: X, P, S or C. The viewer isolates each stage; shared bearings and routing appear only in All parts.
2. Assemble each carriage around its worm and guide axles using its two joining pins. Retain the multiplexer lever, reaction gear and elastic-band arrangement.
3. Fit shafts, gears, collars and bearings together before pinning their supports to the base. The common A shaft passes through P and the near carry data gear; it must be installed as one shared shaft, not two independently driven inputs.
4. Fit the coaxial B′ connection, then the P distribution train at the left edge and Cin distribution train at the right edge. The compound 8-tooth gears at each stage are keyed to a common axle.
5. Pin all fixed supports to the base. Follow the manifest/assembled viewer for collar positions rather than adding collars in the narrow common-A gear gap.
6. Fit one elastic band per actuator. Check each selector's complete travel by hand before driving the gear trains together. The printed bores, guide friction and band preload need physical verification.
7. Connect A, B, SU and Cin at the labelled ports. Sum and Cout are the output ports. Only the least significant Cin receives SU directly in an adder/subtractor chain.

## Verification and limits

`Logic checks.json` checks all 16 input combinations, including mesh parity and mirrored selector conventions, against integer addition. `Printed clearance checks.json` checks carriage travel and endpoint combinations. `Native clearance checks.json` samples LEGO vertices and face centres against printed surfaces; `Mount checks.json` checks the mounting pins and through-holes. `Print checks.json` checks connected, watertight meshes, bed contact and axle bore entrance heights.

`Gear checks.json` studies projected native tooth outlines and compatible relative gear phases over one revolution. These are geometric checks, not loaded contact simulation, friction measurements, stiffness analysis or a continuous proof that every LEGO part clears every other part. The viewer uses prescribed settled carriage poses; changing a checkbox does not simulate the dynamics of an input reversal. The mechanism still needs a physical fit and load test, particularly the shared A shaft and selector fan-out.

The existing multiplexer, register and old adder in the Git repository are unchanged. This prototype is saved separately in adder-draft for review.
