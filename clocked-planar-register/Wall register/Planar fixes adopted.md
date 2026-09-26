# Adopting the tested planar-register lessons

Source conversation: **Strengthen planar 1-bit register**. The accepted late revision restarted from f973a1c and used small local reinforcements; the earlier broad bracket redesign was rejected. This adaptation uses the final local reduced-rail approach rather than those superseded changes.

The highlighted fault was the thin plastic at the two re-entrant corners of the carriage's rail slot, on both carriage halves. Reducing rail thickness creates space to reinforce those corners without raising the worm or clutch axles. The corresponding wall-register actuator cores retain their axle datums, clutch polarity and operating travel.

The wall model has different rear track supports, so its carriage and track must be updated together. The new nominal rail is 2 mm thick, with 0.25 mm sliding gaps, and carriage pads extend toward a backing face with a nominal 0.3 mm gap. The CLOCK pads are shorter to clear the WRITE rod, and their lower face receives the backing because the WRITE rod occupies space behind the upper face. The slave backing leaves the clock-bar corridor open while retaining support beneath both carriage halves. The final-output section checks verify both 2 mm slot-corner ligaments through each half’s width and the retained backing pads. The slave fork’s upper backing pad is 4 mm wide so it clears the moving clock bar; its full-width corner reinforcement remains. Backing probes require at least 99% material within their interior contact strips, allowing small edge chamfers.

The planar lock-band anchor was a closed bridge with no installation route for a closed elastic loop. The wall version also had a root at its front lip. Its replacement moves the root behind the band groove and leaves a free front end. The complete hook belongs to the removable bolt-guide fixture; a recess in the frame clears its rear root. The original band-seat centre and elastic route are retained. Install the band over the free end before fitting the lock bolt. Use the measured clearance and band-size assumptions in Band installation checks.json; CAD fit does not establish band stretch or spring force.

The planar revision’s small Right side frame haunch belongs to an inherited frame branch that this wall design has replaced with separate pinned bearing walls; its coordinates are not copied onto unrelated geometry. Their paired-pin, widened mounting sections provide the corresponding fixed support.

The other relevant lessons—separate bearing walls, at least two spaced pins, robust mounting ligaments, and axle-bearing faces on the bed—are retained from the preceding wall revision. Rollers and shaft alignment are outside this revision's scope.

This note describes the adaptation; consult current hash-matched validation reports before printing. Old print files must not be mixed with the new carriage/track set.

## Print and assembly

Use the current oriented exports and print the matching carriage/guide fixtures together. The shared frames also change around the band-access recesses; do not mix the previous frame with these lock-anchor fixtures. The frame remains split and joined with friction pins.

The lock-anchor fixture has a projecting free lip. Review it in the slicer and remove local support, if needed, completely from the band contact surfaces. The main frame parts retain their strict support-free screen. Axle-bearing faces use the exported bed orientations.

Fit each lock-return band over the free front lip before fitting the bolt, then stretch its upper end onto the bolt anchor. The insertion check uses a 0.8 mm-thick closed loop expanded to a 2.6 mm inner radius around the 2.4 mm retaining lip; groove width is 1.2 mm. It checks geometric access, not achievable stretch for a particular rubber band.

The lower WRITE guide now uses two side-by-side friction pins beneath its bridge. Its front relief clears the thicker carriage. Fit rod guide carriers on the bench before attaching them to the frame, as described in the assembly revision.

[Actual carriage sections](Carriage%20sections.png) show the guide and backing geometry. The full motion screen includes both adjacent bit rows; its finite poses do not establish physical stiffness, friction or eight-bit actuation force.
