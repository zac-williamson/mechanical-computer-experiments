This is the pre-redesign audit. Its geometry hash and measured clearances refer to the earlier revision. See Transmission redesign.md for the current shaft/bearing changes. No new strength qualification is implied.

# Printed strength and carriage guidance review

This review applies to geometry ccea559a04bdf3912b48388003c8dbed982160c1f88de5093a4af2297d09ee38. No geometry was changed during this audit.

The original multiplexer base is a functional guide, not just a mounting surface. compact_structure.py extracts its guide profile at X -19.8..19.8, Y 15.5..32.8, Z 9..12.2 and carries that profile into the storage and control modules. Rotation of an entire actuator preserves relative guide geometry, but cropping, connection stiffness and changes in gravity/load direction require separate checks.

## Exploratory free-roll screen

screen_wall_carriage_roll.py perturbs the two printed carriage halves as a rigid assembly around the nominal worm/rail axis. For WRITE the carriage half and merged rod/pickup are included. Each is tested against all fixed printed structure at five poses of one capture. The contact threshold is 0.001 mm³ intersection, so the results are numerical approximations, not manufactured clearances. This does not model axle play, compliance, gears, friction or all operating cases; gate forks and passive selector are excluded.

| Assembly | First contact, one roll direction | Opposite direction |
|---|---:|---:|
| Master/slave | 1.90° | 0.84° |
| CLOCK actuator | 1.90° | 1.61° |
| WRITE actuator including pickup | 0.68° | 0.68° |

Storage contact in the first direction occurs at the retained track near Z 12.1. In the other direction the first contact is near the roof at Z 45.5, not at that track. WRITE's first contact is at the long output rod guides. Therefore these numbers do not establish a suitable, local, opposed carriage bearing. At a 30 mm lever radius, 1.9° corresponds to approximately 1.0 mm transverse movement.

## Stiffness concerns found in the source

- One bit chassis brace is only 1.2 mm through its thin direction (Y 16.4..17.6), with a 37 mm vertical run. Its connections and load sharing need assessment before it can be credited as a stiff support.
- The CLOCK pickup has a roughly 28 mm reach in depth through a 4 mm wide upright. This offsets the rod reaction from the actuator and can bend/twist its connections.
- Guide supports include narrow rear-reaching legs. Their flex adds to nominal guide clearance.
- Bellcrank pivot bosses have 2 mm radial wall around their bores. Their bearing pressure, wear and layer orientation have not been qualified.

No allowable loads or deflections are assigned without material, print orientation, process and measured operating force. A single connected STL and a prescribed-pose collision pass establish neither stiffness nor constraint completeness.

## Design priorities

Preserve a deliberate local anti-roll bearing close to each carriage, with opposed capture for reverse loading and assembly clearance. Carry its reaction into nearby chassis ribs. Avoid using the clutch teeth, roof collision or a distant flexible output rod as the intended anti-roll stop. Short replaceable guide pads and adjustable clearance are candidates to test; smaller contact area alone is not evidence of lower friction. Keep force application close to the guide to limit cocking. Prefer ribs/gussets on fixed supports before adding moving mass. Check the full stroke and return action after any geometry change.

A printed single-bit/controller test should measure carriage twist and guide deflection under the actual rod load in both directions. Test the controller against the measured combined load of eight bits before claiming eight-bit capability.
