# Multiplexer

A worm-driven carriage selects one of two rotating clutch inputs. The reaction gear drives a two-tooth lever against rounded carriage stops. At each end of travel, the lever releases so the input can continue rotating. The assembled design has been reported working in a physical test.

## Files and printing

- `Complete print layout.stl`: all 13 printed components.
- `* - print.stl`: individual components oriented for printing.
- Other component STLs: assembly coordinates.
- `Viewer.html`: full assembly, individual parts and switching animation.
- `Assembly manifest.json` and `LEGO parts.csv`: assembly and hardware inventory.
- `Source/`: geometric validation scripts and their required contact tables.

Use the supplied print orientations to keep axle bores vertical. The clutch fork requires local support. Keep support off bearing bores and sliding faces. Join printed parts with LEGO friction pins.

## Lever pivot assembly

Use a LEGO 3706 6L axle through both bridge bearings. Between their inner faces, assemble **front bearing → 12 mm lever journal → full bush → full bush → rear bearing**. Two external half bushes retain the axle. The nominal total running clearance is 0.4 mm. Seat the internal bushes against the rear bearing without clamping the lever.

## Inspection and checks

The viewer includes manual travel and lever-angle controls, reversal animation and a pivot view that hides the lever to expose both bearings and the bushings. The animation represents rigid contact geometry, not friction or elastic deflection.

See [Source/README.md](Source/README.md) for validation commands, assumptions and coverage. Geometric checks complement physical testing; they do not establish service life or load capacity.
