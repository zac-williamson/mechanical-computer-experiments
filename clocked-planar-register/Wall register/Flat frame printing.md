# Flat-bed frame revision

The two bit-frame halves and the controller frame are separate frame parts.
Their rear faces lie on the print bed. Use the STLs in **Print oriented frames/**;
these are already transformed to bed Z = 0 and do not require rotation in the
slicer. Development parts remain in assembly coordinates.

The revision changes the manufactured structure, not the gear or clutch layout:

- Rear rails, attachment pads and joining bosses have a common flat bed face.
- Bed-level ribs join socket feet to the main lattice.
- The two horizontal frame-joining pins are retained. Their holes have circular
  seating sides and 45-degree roofs instead of round, unsupported ceilings.
- Bearing-wall locating pockets open towards the front of the frame.
- Mounting holes are normal to the bed; collar recesses open towards the mating face.
- Actuator and guide fixtures are separate from the frame. Each has at least two LEGO
  2780 friction pins, nominal 5 mm sockets and full-depth mounting pads.
  Seating lands establish the working-face position. There are no fixture screws.

There are seven fixtures per bit and four in the shared controller: 28 fixture
pins for one bit plus controller. Install pins into one mating part, then press
the two parts together; the pin's central collar remains at the joint, rather
than being forced through a socket. The printed fit needs a calibration trial.
The three broad controller carriers use four pins each; the other fixtures use two.
Frame fixture schedule.json identifies every fixture and its pin centres.

The working guide faces retain their positions. The upper controller rod guide
has an additional attachment root on its right side, below the nearby bearing
wall. Frame parts remain separate across the existing split.

**Frame print checks.json** examines the actual exported meshes: bed contact,
watertightness, connected-solid count and downward-facing surfaces exceeding
45 degrees from vertical. It permits no bridge exemption. This is a geometric
support-free criterion; printer calibration and first-layer adhesion still
matter. It applies to the three frame parts, not to all other mechanism parts.
The detached working fixtures require their own orientation and slicer review;
they are not certified support-free by the frame report.

In the viewer, choose **Show → Frames only** to inspect the three frame parts
without the mechanism obscuring them. The model envelope and 112 mm row pitch
are unchanged.
