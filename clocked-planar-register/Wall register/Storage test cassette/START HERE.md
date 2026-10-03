# Print and test the storage mechanism in small steps

The working design has changed: the master and slave storage supports each now use four separately replaceable parts. These production parts also fit the small storage test base. They are not simplified mock-ups.

| Replaceable part | What a failed test lets you change | Attachment |
|---|---|---|
| Rail and backing cartridge | Rail fit, sliding finish, anti-rocking clearance and backing stiffness | Two LEGO 2780 friction pins |
| Left lock-guide cheek | One side of the bolt guide, clearance and print fit | Two LEGO 2780 friction pins |
| Right lock-guide cheek | Other side of the bolt guide | Two LEGO 2780 friction pins |
| Band anchor | Band seating, retention and anchor strength | Two LEGO 2780 friction pins |

The carriage halves, locking bolt, front/rear actuator bearing cheeks and transmission bearing walls remain separate production parts. A worn or poorly fitting rail no longer entails reprinting the bolt guide or the complete combined storage fixture. The guide cheeks have separate print orientations; their working faces and the rail faces do not rest on support material.

The separate interfaces add stationary material and friction pins; the moving parts are unchanged. Each bank now uses eight support-mounting pins across these four parts, instead of two for the previous combined fixture.

The revised large frame has matching seats for these modules. This is a one-time mounting-interface change: existing old frame halves do not acquire the new sockets without reprinting. **Start with the small test base instead.** The large frame remains two pinned pieces, with its existing seam and overall Z height. Gear centres, clutch polarity, axle supports, carriage travel and row pitch are unchanged.

## First prints

Use `Storage test cassette.zip`. Its individual STLs and three incremental layouts are already oriented. Each later stage contains only the additional printed parts; keep the parts from earlier stages. The complete test cassette is also selectable under **Show → Storage test cassette** in the main viewer.

1. **Rail fit:** print the rail/backing cartridge and the two master carriage halves. Join the carriage halves using their two production 2780 pins. Assemble the carriage onto the detached cartridge before mounting it. Check sliding and rocking over the intended ±3.756 mm along X, by hand. Hold the cartridge's mounting pads; do not use the thin rail as a handle or force the carriage beyond its designed stroke. No controller, large frame, bearing walls or gears are needed for this fit test.
2. **Lock fit:** add the small test base, both guide cheeks, lock bolt and band anchor. Install the rubber band over the detached anchor's rear lip, approaching from +Y toward −Y, before pinning the anchor onto the base. Fit the other end to the bolt. Assemble the two guide cheeks around the bolt, then pin them to the base. Check free bolt travel and locking/release at each carriage endpoint. The lower band seat is Y36.8, −0.5 mm along Y from the previous Y37.3; the upper seat is still Y37.3. Band preload and suitable band size require a physical test.
3. **Actuator and clutch:** add the production bearing walls, actuator cheek carrier, bearing cheeks, lever, worm, native axle/clutch hardware and the second band from the manifest. Assemble the bearing cheeks and native pivot hardware before pinning the carrier into the base; slide the shafts through their bearing walls before fitting the opposed axial retainers. Release the locking bolt by hand before turning the worm input. Test both carriage endpoints and clutch engagement by turning each clutch gear manually. Confirm which gear drives the output in each position; observe drag in the disengaged gear. Do not drive the worm against a locked carriage.

The base occupies **86 mm along X × 105 mm along Z**, with **10.3 mm depth along Y**. Its complete rear face is at Y48.5. All production interfaces stay at their original coordinates. It is a reusable test fixture, not another piece required in the wall assembly.

There is no powered ±POWER supply or clock sequencer in the bench cassette. It tests component fit, travel, locking and manual clutch action; it does not prove clock timing, register logic, output speed under load or eight-bit control force. The viewer's inherited motion illustrates travel and is not a simulation of hand operation or test success.

## Replacement and assembly access

Access the friction pins from the rear of the detached base/frame. Remove pins before withdrawing the affected module. For a fully populated rail/carriage replacement, release the lock and band tension, remove the relevant axial retainers, and withdraw the worm/output shafts far enough to free the carriage before removing the cartridge pins. Separate the carriage on the bench. The surrounding frame and shared controller can remain assembled. For lock-guide changes, release band tension and remove the bolt/guide assembly before separating the two cheeks. The anchor is removable so installing a closed band does not require stretching it through an inaccessible frame pocket.

Preserve the two-pin centres and seating planes when changing a component. The print manifest and storage module schedule record the actual interfaces. Avoid changing rail height, working clearances or attachment locations unintentionally when revising a local part. The master and slave cartridges have the same mount spacing but different clearance reliefs: use their named production variants in the complete register.

## Mounting coordinates for local revisions

All socket axes below run along Y and use LEGO 2780 pins. Coordinates are millimetres for the master/test base; add +106 along X for the slave.

| Part | Pin centres (X, Z) | Pin centre Y | Seating plane Y |
|---|---|---|---|
| Rail/backing | (−6.8, 0), (+6.8, 0) | 38.0 | 38.2 |
| Left guide | (−20.5, 49), (−20.5, 59) | 40.5 | 40.7 |
| Right guide | (+10.5, 49), (+10.5, 59) | 40.5 | 40.7 |
| Band anchor | (−13.5, 39.8), (+3.5, 39.8) | 40.5 | 40.7 |

The rail mounts sit inward of the reversing idlers' axial envelopes, with a nominal 0.4 mm X gap between the mounting pad edge and the nearest idler face. Their centres are 13.6 mm apart. This clearance and mounting spacing must be preserved when widening the pads.

## Print orientation and evidence

- Rail/backing: rear +Y face on the bed; build toward −Y.
- Left guide: outer −X face on the bed; build toward +X.
- Right guide: outer +X face on the bed; build toward −X.
- Band anchor: front −Y face on the bed; build toward +Y.
- Test base and revised frame halves: rear +Y face on the bed; build toward −Y.
- Reused axle-bearing parts: use their supplied production print orientations, with axle bores vertical and bearing rings at the bed. Do not print assembly-coordinate STLs as arranged in the viewer.

The new guide cheeks have horizontal **friction-pin** sockets with 45° roofs in their print orientations. They contain no horizontal axle bearings. Each guide still has two spaced pins and broad seating pads. The new parts use neither screws nor support-formed sliding faces.

For the new modules, changed frame halves and test base, the reports check watertightness, connected solids, every downward face against 45°, and layer-to-layer outward growth at 0.2 mm increments. The base also has socket-material and sampled clearance checks against its actual production components. These are geometric checks. Bambu Studio was not launched, no new slicer preview was generated, and these parts have not been physically printed or strength-tested. Existing unchanged production pieces retain their separately documented print limitations. In particular, the stage-3 actuator-cheek carrier (`bit frame 0 removable fixture 4`) has static overhangs in its supplied orientation; its separate print report records them. It contains no axle bearing or sliding surface. It is excluded from the new-part overhang pass and is not needed for stages 1–2. Print times are intentionally not estimated without slicing.

Coordinate convention: X follows the data/power axles; Z indexes mounting rows and shared control rods; +Y points toward the rear/base. The viewer's fixed-corner coloured +X/+Y/+Z indicator follows camera rotation in every scene, independently of labels and pan/zoom.
