# Rod and guide sliding surfaces

Use **Print oriented sliding parts/** for the four orange control rods, their nine guide caps, five guide carriers, and the separate CLOCK pickup. These files already have their intended bed faces at Z=0. Do not auto-orient them onto another face.

The CLOCK rod previously included a deep offset pickup whose underside needed support. It is now two pieces. The long rod prints front-face down, with its arm built from bed-supported material and sloped braces. The pickup has a continuous flat rear bed face; its working slot prints upward with vertical walls. Two LEGO 2780 friction pins, spaced 12 mm apart, join the pickup to the rod. The original working-slot position, rod stroke and actuator connection are retained.

The bit CLOCK rod, bit WRITE rod and controller WRITE rod print on their long, flat front faces. Their end shoes and local projections grow above that plane. Their four guide-contact faces are either on the bed, upward-facing, or vertical. Their different outlines do not require support under those running faces.

Guide caps print front-face down; guide carriers print rear-face down. Their inner guide floors face upward and the side walls are vertical. The axle-bearing carriers retain their existing axle-bed orientation. Static features elsewhere on these carriers may still need local support; do not permit the slicer to put support interfaces on the running faces. The small pin-collar shoulders are not sliding surfaces and are reported separately.

## Assembly

1. Print each part in its supplied orientation. Inspect the slicer preview for support contact on the long rod flats, guide-channel faces, or CLOCK pickup slot; none should be required there.
2. Seat the two CLOCK pickup pins into the rod from its rear (+Y) side, then press the separately printed pickup onto their exposed halves. Assemble this joint on the bench before installing the rod and actuator. The central collars have relieved seats; do not force them through the bores.
3. Continue with [the rod/guide installation sequence](Assembly%20and%20print%20revision.md). The existing removable guide caps allow the rods to be installed without passing their enlarged shoes through closed guides.

The [sliding-surface check](Sliding%20surface%20print%20checks.json) examines actual final-mesh faces along all guide contact regions over the nominal stroke, checks the entire detached pickup for unsupported faces, and records the print transforms. This establishes geometric print orientation, not calibrated surface finish or loaded friction. Use the matching updated CLOCK rod and pickup together.

[Actual CLOCK rod and pickup print orientations](CLOCK%20rod%20print%20orientation.png).
