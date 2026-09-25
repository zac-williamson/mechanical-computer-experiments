# Friction-pin-only mechanism connections

No screws, bolts or nuts are specified for connecting the current mechanism's parts.

- The seven bit fixtures use two LEGO 2780 friction pins each. The controller has three broader carriers with four pins each and one smaller fixture with two pins: 28 fixture pins for one bit plus controller. The sockets have wider roots than the former screw pilots, and their positions are chosen against operating envelopes. Existing bearing walls and front cheeks retain their paired pins.
- Each CLOCK/WRITE rod splice uses one front-mounted bridge and two 2780 pins, one engaging each rod-end shoe. The pins carry axial force in shear. This is not a friction clamp. It introduces no intentional clearance along the pin's shear direction; actual backlash depends on printed pin-hole fit.
- The rod shoes are 10.4 mm wide and 7.6 mm thick at their sockets. The splice bridge is 13.2 mm wide. Pins are 14 mm apart along the rod. Side lips help locate the rectangular shoes.
- The two horizontal pins connecting the printable frame halves are retained.

For a one-bit assembly plus controller this replaces the 30 fixture screws with 28 pins and adds four rod-splice pins. An eight-bit bank uses 126 fixture pins and 32 rod-splice pins, in addition to the existing bearing-wall, carriage, cheek and frame-joint pins.

The two bridge STLs in **Print oriented rod splices/** already lie flat on the bed with vertical pin holes. Both pass the geometric 45-degree overhang screen without bridge exemptions.

Assemble each fixture's pins into one mating part, then press the fixture and frame together. Do not attempt to force a pin's centre collar through a nominal 5 mm hole. For each rod splice, insert the pins into the rod shoes from their front mating faces first, then press the bridge onto the exposed pin halves. The centre collars sit between the bridge and shoes; do not try pushing complete pins through the bridge from its outer face. The printed sockets are nominal 5 mm with collar reliefs; friction fit and durability need a small print trial.

This change removes screw connections. It does not qualify the previously unresolved closed rod-guide insertion sequence, all part print orientations, or loaded eight-bit actuation force. The five integrated parts identified in Axle-hole printing.md remain a separate manufacturing issue.
