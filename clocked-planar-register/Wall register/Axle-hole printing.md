# Axle-hole printing

The current revision separates four rear bearing cheeks and the controller WRITE bearing shoe, correcting the five previously identified bed-contact failures. A relieved sleeve also brings the CLOCK amplifier pivot ring to its fixture's common bed plane.

Use the exported print orientations:

- **Print oriented bearing walls/** for transmission walls.
- **Print oriented axle parts/** for other detected bearing parts.
- **Print oriented assembly parts/** for new detachable cheeks, the WRITE bearing shoe/rod and guide caps.

The [axle print-face audit](Axle%20print-face%20audit.json) records each detected bearing, print axis and bed-ring result against the current geometry hash. A complete annular section is required to classify a bearing; nearby guide material does not count as a bearing hole.

All detected axle-bearing parts must pass this screen before packaging. This does not imply every part is entirely free of overhangs: static surfaces that do not contact moving parts may retain overhangs, as requested. Frames have their separate strict support-free check.

See [assembly and print revision](Assembly%20and%20print%20revision.md) for installation order and limitations. Printed fit remains to be tested physically.
