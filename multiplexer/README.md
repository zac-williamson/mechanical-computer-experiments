# Multiplexer

A worm-actuated sliding carriage operates a LEGO clutch. A toothed reaction lever controls carriage movement and end release. The carriage stop crossbar is supported at both ends; the reaction and lever axles are supported by front and rear bridges. A return band connects behind the rear bridge.

## Print files

- `Complete print layout.stl`: all 15 printed components, oriented and separated, approximately 246.9 × 246.3 × 37 mm.
- Individual `- print.stl` files: the same components in their printing orientations.
- Other STLs are in assembly coordinates and are not print layouts.

Use millimetres and 100% scale. Preserve the supplied orientations. Each carriage half rests on an outer bearing-wall face, with its axle and joining-pin bores vertical. The shoe sliding faces remain off the print bed. Bridge axle bores are vertical in their supplied orientations.

Local removable supports are required under projecting carriage features, the right half's inner wall and the lever's rear hub/link. Keep support out of vertical bores and off shoe sliding surfaces. Support an elevated hub's underside without filling its bore. Inspect the sliced layers; no sacrificial supports are embedded in these meshes.

Bridge mounting sockets have a nominal 4.8 mm diameter; base sockets are 4.92 mm. Check the actual fit without scaling the whole model.

## Hardware

See [LEGO parts.csv](LEGO%20parts.csv) and `Assembly manifest.json` for quantities and placements. Two 8L axles carry the reaction gear and lever. Three 12L axles span the carriage: selector/worm and two guides. Two 2L friction pins join the carriage halves, at the crossbar and base. The clutch actuator belongs entirely to the left half.

The lever's inner half-bushes occupy Z22–26 and Z44–48 mm. Its hubs occupy Z26–30 and Z39.6–44 mm; the rear bearing occupies Z48.2–54 mm. Preserve the approximately 0.2 mm bearing-face clearance. Do not clamp rotating parts with the outside bushes. Guide axles span X−40 to X56 mm, with right retainers at X52–56 mm.

## Assembly

1. Fit the gearbox walls, bottom guides, left guide support and bearing wall X48 to the base.
2. Join the carriage halves with both friction pins and seat their broad mating faces fully.
3. Thread the selector and guide axles through the carriage, retainers and fixed walls. Engage the shoe with the bottom guides and check unloaded travel.
4. Install the front bridge, reaction gear/spacers and forked lever with its inner half-bushes on their respective 8L axles.
5. Fit the rear bridge over both axles, seat its four base pins and install the outside retainers.
6. Fit the return band between its moving cleat and fixed anchor behind the rear bridge. Hand-turn in both directions, checking clutch seating, end release and resetting. Use only enough band tension to reset the lever.

## Inspection

[Viewer.html](Viewer.html) supports part isolation, carriage/worm isolation, zoom, pan and rotation. X is carriage travel, Y points toward the base and Z follows the reaction/lever axles. The three coloured arrows show these axes.

The geometric sequence reaches approximately −4.35 to +4.325 mm carriage displacement over 1,800 samples. Printed parts were checked as single watertight solids and the full plate fits within 250 × 250 mm. Clearance checks sample geometry rather than all loads and manufacturing variation. This assembly has not been physically built or sliced; fit, flex, friction and reliable release remain to be tested.
