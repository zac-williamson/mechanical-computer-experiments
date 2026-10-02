# Mechanical computer modules

Build the next 1-bit register from a small collection of reusable mechanical modules. Only these two designs are active:

| Active design | Entry points |
|---|---|
| Modular actuator, carriage and locking bolt | [Viewer](modular-actuator-carriage/Viewer.html) · [Design and print files](modular-actuator-carriage/README.md) · [Build and validation](modular-actuator-carriage/Source/README.md) |
| Latched 1-bit register | [Viewer](planar-register/register-from-multiplexer/Planar%20register/Viewer.html) · [Design, sources and checks](planar-register/README.md) |

Start with the modular actuator/carriage module when developing reusable components. Keep the latched 1-bit register as the assembly reference. It is a level-sensitive write-enabled latch, not the archived clocked master–slave register.

Read [AGENTS.md](AGENTS.md) before making changes. CAD checks do not establish physical strength, friction, wear or reliable switching; consult each design's reported limitations.

All other designs are in [archive/](archive/README.md). They are historical references, not current candidates or starting points. Do not revive them unless explicitly requested.
