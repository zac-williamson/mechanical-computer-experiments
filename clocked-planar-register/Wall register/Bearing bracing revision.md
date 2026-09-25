Current frame manufacture: [flat-bed frames and separate working fixtures](Flat%20frame%20printing.md). This supersedes the integrated working fixtures described below.

# Removable bearing walls and relocated POWER train

This revision supersedes the integral transmission-bearing design. Those walls
were fused to the chassis; they were not independently pinned printable parts.

## Gear placement

The complete three-gear POWER input/reversing train moves from X = 32 to X = 70 mm.
All three gears remain 16T, with the same axes, centre distances and two meshes.
The local input axle moves with it. Storage clutch gears and their polarity are
unchanged. The two new POWER-input bearing routes are straight, rather than
looping around the former gear location. Their bearing stations are X = 60 and
80 mm. Other bearing stations and retaining collars are adjusted to suit.

## Separate bearing walls

There are 13 removable transmission-bearing walls per bit and 5 in the shared
controller. Each wall has two matching frame sockets and two native 2780 pins,
with at least 12 mm between pin axes. These are separate connected solids, not
labels applied to regions of a fused chassis. Broad plates and rear heels carry
loads to the paired mounting pads. Closely spaced supports share a wall where
the geometry connects them. Existing working guide faces are reserved during
bearing-route generation; rear interfaces have explicit locating clearance.

The wall-to-frame gap is 0.4 mm. Bit mounting-pin centres lie at Y = 40.5 mm;
controller mounting-pin centres lie at Y = 48.5 mm. Rear socket pads extend to
Y = 48.3 and 56.3 mm respectively. This adds rear depth for real attachment
material and clearance behind the moving control parts. Row pitch remains
112 mm and the moving linkages retain their geometry.

Two obsolete joining pins from the old controller rear pivot bridge are removed;
that bridge is integral with the controller frame.

The four removable actuator front cheeks retain their existing two joining
pins, on axes 7.2 mm apart. Their axle-hole direction already differs from the
transmission walls and must be respected when arranging prints.

## Restored frame joint

The bit frame is two separate printable solids, joined by two horizontal 2780
pins. The bearing-socket consolidation had inadvertently bridged the former
seam. The split is now applied after all sockets and ribs are added.

The 0.4 mm seam is centred at X = 28.5 mm, shifted 4.5 mm from its former
position to clear the bearing mounting pads. Joining-pin axes run along X at
Y = 44.5 mm and Z = -28 / 56 mm, behind the removable bearing walls. The split
passes through the rear frame, leaving the front actuator reaction arm intact.
Neither the external envelope nor the 112 mm row pitch increases. The left
frame spans 169.4 × 104.2 mm in the wall plane; the right spans 136.3 × 102.2 mm.
These are frame-part bounds, not a slicer bed-fit certification.

## Printing and assembly

Use **Print oriented bearing walls/** for the 18 wall STLs in their intended
axis orientation. The bit-wall X bearing axes and controller-wall Z bearing
axes are transformed to vertical printer Z, with the lowest face on Z = 0.
Bearing print orientations.json records these transforms. These exports avoid
printing the transmission axle holes horizontally as part of the base frame.
Review stepped faces, any overhang support and horizontal mounting-pin holes
in the slicer; this is not a support-free print certification.

The complete model's **Development parts/** STLs remain in assembly coordinates.
Do not assume those are already oriented for printing. The frame and actuator
parts still need their own print-orientation review.

Fit gears, couplers and inner retaining collars to each shaft before closing
access with the relevant wall. Seat each wall on both frame pins, then install
outside retainers. Verify free shaft rotation and nominal end clearance by hand.
The CAD checks establish bore, pin-socket and clearance geometry, not physical
fit, assembly access, layer strength or loaded deflection.

## Evidence

Bearing and retention checks.json checks 32 complete bearing lands and axle
engagement. Bearing attachment checks.json verifies two pin sockets at both
ends of every wall attachment, plus the four actuator cheeks. Bearing print
orientations.json checks that every exported wall's axle-hole axis is normal
to the bed. Validation status.md reports current motion and rotating-envelope
checks and the unresolved inherited actuator contacts.
