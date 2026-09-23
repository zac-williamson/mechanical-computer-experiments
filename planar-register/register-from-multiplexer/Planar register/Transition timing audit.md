# Timing audit — revised cam

The updated model contains 108 examples across all 12 D/WRITE transitions, including both possible held bits and six assumed speed/phase profiles. It uses hysteretic ±0.4 mm clutch-ring play and first-contact coupling. Time is normalized, not calibrated to a particular motor.

There are no commanded motions through an inserted bolt in these nominal examples. However, the two simultaneous D-reversal / WRITE-closing transitions still have three invalid-HOLD examples each: the memory can stop between its pockets. Closing does not guarantee capture of the old bit. This is a latch timing problem, not cured by the stronger guide.

The animation is an event witness, not a force-driven simulation. It does not establish physical minimum/maximum motor response times, native tooth backlash, loaded release, coasting or freedom from every possible jam. See the JSON for individual cases and the main README for the mechanical checks.
