# Mechanical LEGO logic

Current CAD models, parts lists, printable components and interactive viewers for a mechanical multiplexer, one-bit register, adder/subtractor and register–ALU assembly.

| Design | Viewer | Build information | Status |
|---|---|---|---|
| Multiplexer | [Open model](multiplexer/Viewer.html) | [Build guide](multiplexer/README.md) | 13-part printable prototype; two-part carriage and through-hole base; physical testing required |
| One-bit register | [Open model](register/Viewer.html) | [Build guide](register/README.md) | Compact 37-part prototype; integrated bridge lock; physical testing required |
| Adder/subtractor | [Open model](adder/Viewer.html) | [Build guide](adder/README.md) | Four functions on a 200 × 172 mm board; three print plates |
| Register + ALU assembly | [Open model](assembly/Viewer.html) | [Interface and status](assembly/README.md) | Inspection model; incomplete routing supports, not a print release |

Download or clone this repository and open a Viewer.html locally in a modern browser. GitHub's file view does not run these viewers. Drag to rotate, scroll to zoom; the multiplexer, adder and assembly also support Shift/right-drag panning and part isolation.

## Files and units

Each directory contains its current assembly manifest, native LEGO inventory and assembly-position STL meshes. Use only explicitly marked print files or print plates for printing, at 100% scale in millimetres. The individual build guides identify them. No sliced machine instructions or embedded supports are supplied.

Native LEGO positions in manifests use LDraw units (20 units per stud; 0.4 mm per unit); printed STL meshes use millimetres. Keep each manifest with its own meshes. Viewers embed their display geometry and work independently of the STL files.

These are four separate current models, not one interchangeable component library. The complete assembly does not yet integrate the standalone direct-axle register, compact arithmetic board or compact multiplexer actuator. Do not substitute those parts into it without redesigning and checking the interfaces.

The legacy assembly includes printed routing couplers. These are unresolved departures from the desired ≤12-stud axle and native LEGO connector constraints. The multiplexer uses axles no longer than 12 studs; the arithmetic board uses native LEGO clutch hubs and connectors.

## Model status

Animations illustrate prescribed geometry, not forces, friction or elastic deformation. Digital clearance checks do not establish printed fit, reliable switching or load capacity. Build and test one mechanism before scaling it up.

This repository contains the current exported design assets. The register also includes a portable generator in register/Source; its inherited multiplexer geometry is supplied as mesh inputs. Development logs, superseded alternatives and local-machine generation scripts are excluded.
