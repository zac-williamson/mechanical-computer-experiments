This is a separate clocked-register design. Do not overwrite ../planar-register or ../register. Use independent output and working directories. Read README.md for requirements.

## Bearing attachments

Transmission bearing walls must be separate printable parts, with bearing-hole
axes perpendicular to the print bed. Each wall must have at least two spaced,
engaged mounting pins to the frame; never rely on one friction pin. Design gear
and shaft locations before wall routing so supports can be short and broad.
Retain the current storage-clutch polarity; the proposed double inversion was
explicitly cancelled by the user.

The bit frame must remain two separate printable solids joined by two horizontal pins. Apply the frame seam after all support sockets and ribs are added; never fuse the frame halves during consolidation.

Frame parts must have a flat rear bed face and pass the frame overhang screen with no bridge exemption. Export their actual print orientations. Keep working actuator/guide fixtures separately mounted; do not fuse their undercuts back into the frames. Every detached fixture requires two spaced fasteners. Horizontal frame-joint pin holes have 45-degree roofs; bearing walls retain their paired pins.

All mechanism connections must use LEGO friction pins, not screws, bolts or nuts. Detached fixtures require at least two spaced engaged pins. Rod splices must transmit axial force through engaged pins in shear, not screw-clamp preload.

Manufacturing revision scope: keep axle-bearing rings on a common print-bed
face. Other overhangs are acceptable on static surfaces that do not touch moving
parts; avoid unnecessary redesign to eliminate every overhang. Shared rods must
have an explicit installation/removal path through their guides. Strengthen pin
socket edges and fixture load paths without moving gear centres or increasing
frame height. The user has physically tested the axle-based follower approach;
do not redesign rollers or investigate shaft alignment as part of this revision.

Run geometry generation and checks sequentially through Source/run_wall_job.py.
It limits TBB/BLAS to one worker, lowers process priority, enforces a single-job
lock, and stops at 1.2 GiB resident memory. Do not spawn agents or simultaneous
geometry jobs: the user explicitly revoked parallelisation after a machine freeze.

Rod/guide running faces must not print on support material. Maintain the explicit
sliding-part orientations and the full-stroke sliding-face audit. The CLOCK
pickup is a separate flat-backed part joined to its rod by two LEGO friction
pins; do not fuse it back into a support-dependent rod.
