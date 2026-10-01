# Mechanical computer project rules

## All model viewers must render axes

Every new or modified model viewer must display an always-visible, clearly labelled X/Y/Z orientation indicator. This is a standing user requirement, not an optional feature.

- Use a fixed screen corner so the indicator cannot be obscured by geometry or lost while zooming or panning.
- Rotate the indicator with the camera so it reflects the current view. Label the positive directions +X, +Y and +Z, using distinct colours.
- Keep the indicator visible when dimensions, labels, frames or other overlays are disabled, and in every scene/view mode.
- Include a readable coordinate legend: X follows axles/rods left–right; Z indexes mounting rows in the module plane; +Y points toward the rear/base. Preserve the actual model convention if a different design uses different axes, and state it explicitly.
- Describe offsets with their axis and sign plus coordinates where useful (for example, “+8 mm along Y, from Y = 40 to 48”), rather than ambiguous “back”, “up” or “down”.
- Check the axis indicator in front, end and oblique views, including small-screen layouts.

## Printability is a mandatory design constraint

For EVERY new or modified printed part, evaluate printability before publishing geometry, viewer updates, or STLs. The target printer is a Bambu Lab P2S with PLA Basic and a 0.4 mm nozzle.

- Choose and record the actual print orientation first. Evaluate overhangs in that orientation, not in the assembled view.
- Design for printing without supports. Use continuous bed-connected geometry and gradual ramps/gussets; default to at most 45 degrees from the build vertical for unsupported outward growth. Steeper overhangs or bridges need specific slicer/print evidence, not an assumption that PLA can handle them.
- Axle bores must print vertically. Bearing, thrust, sliding, and gear-contact surfaces must NEVER be formed on support material.
- Inspect every changed region and the entire resulting part for abrupt shelves, square projections, unsupported starts, thin flaps, narrow necks, and disconnected or merely touching solids. Reinforcement must itself be printable.
- Check layer-to-layer outward growth against the chosen overhang limit. Checking only for disconnected layer islands is NOT an overhang check and is insufficient. Inspect the print-oriented model and use slicer previews when available.
- Maintain assembly access, full-travel clearance, mating interfaces, and adequate structural thickness when removing or ramping projections. Do not solve printability by introducing interference or fragile webs.
- Regenerate matching print-oriented STLs and layouts after geometry changes. State exactly which checks passed; do not call a part support-free based solely on watertightness, connectedness, or island checks. Clearly distinguish geometric checks from slicer and physical print validation.
- Keep geometry jobs serial and CPU-throttled using the existing project runner.
