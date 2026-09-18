# Multiplexer

One worm-driven carriage selects between two clutch inputs. The reaction gear drives a two-tooth, round-pivot lever against the carriage lozenges. End-of-travel release allows continued input rotation.

## Files

- `Viewer.html`: complete assembly, individual parts and switching animation.
- `Complete print layout.stl`: all 13 printed components.
- `Replacement print layout.stl`: the four repair components: both carriage halves, lever and rear bridge.
- `*-print.stl` / `* - print.stl`: individual bed-oriented components; other component STLs use assembly coordinates.
- `LEGO parts.csv`: hardware inventory.

The carriage uses the original clutch-engaging fork. It needs local support beneath the fork: do not substitute a sloped fork. The small carriage half has a flat bed face. Bearing-hole axes are vertical in the supplied print orientations. Keep support off bores, sliding faces and working contact faces wherever possible. Use LEGO friction pins for the carriage joint and frame mounting.

The lever has deeper two-tooth engagement and rounded working contacts. The rear bridge has a centred tapered band anchor. Bearing walls and lower guide locations remain unchanged. ABS print fit, switching torque and durability still require a physical test; the animation prescribes motion and does not simulate forces.

The assembly-coordinate meshes here are the common actuator source for the other three projects. `Actuator poses.json` supplies their viewer/check poses.
