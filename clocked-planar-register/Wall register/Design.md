Current manufacturing revision: [separate two-pin bearing walls and relocated POWER train](Bearing%20bracing%20revision.md). This supersedes descriptions of integral transmission bearings below.

# Current transmission and bearing revision

The shaft-and-bearing redesign supersedes the transmission support details below. See [the rebuilt layout and assembly notes](Transmission%20redesign.md) and [bearing checks](Bearing%20and%20retention%20checks.json). The viewer and development STLs use this revision.

# Wall register — direct control revision

[Open the labelled model](../Wall%20register.html). Choose one, two, four or eight vertical rows and use the label filter to inspect the controller or a bit. One controller serves the whole bank. All rows occupy the same wall plane; the controller is below the lowest row.

This revision rotates the complete CLOCK and WRITE control actuators, including their input axles. It lowers the slave to the master's height, removes the stepped transmissions, and replaces the per-bit bellcrank slot contacts with plain-bearing roller followers. It is development CAD, not a qualified print release. [Validation status](Validation%20status.md) records the scope of the checks.

## Motion and force paths

**WRITE: E → A → G → H.** The rotated input E drives the WRITE actuator A. Its moving carriage and the vertical rod G share a rigid pickup, with no controller direction-changing bellcrank. Each bit has a 24/24 mm bellcrank, so rod travel and selector travel are both ±3.75 mm. WRITE=1 selects that bit's D input; WRITE=0 selects feedback from its own Q. The former controller used a 24/36 mm arrangement followed by its inverse at each bit. Removing this pair preserves the total ideal mechanical advantage and reduces WRITE rod travel from 11.25 to 7.5 mm end to end.

**CLOCK: D → B → C → F → K.** The rotated CLOCK input retains the 16T/16T header. Actuator B drives the existing 12/30 mm stroke amplifier C; approximately ±3.75 mm becomes ±9.375 mm. The shortened output post connects directly to CLOCK rod F. The controller's second direction-changing bellcrank and long horizontal link have been removed. A 20/20 mm roller-ended bellcrank in each bit moves its local sequencing bar K.

**Storage: I → J → L.** Master I and slave J are coplanar again. The original straight interstage, POWER and feedback arrangements replace the three stepped routes. Six extra gear meshes per bit are gone. On a rising clock the bar withdraws master drive and permits its lock to seat, then releases and drives the slave from the master. Falling clock reverses the sequence. Q emerges at L. The original cam working outline, input-fork float, clutch travel and storage cores are retained. The slave's cropped track has been replaced by its original full track.

## Reducing losses without assuming strength

Four added half-bush rollers per bit run in enlarged slots at the input and output of the two bellcranks. Each bush is keyed to its axle; the axle is free to turn in the crank's circular bore. This replaces bare axle-on-slot sliding contact. These are plain-bearing rollers, not ball bearings. The mechanism may still skid if axle friction is excessive; the animation does not solve free roller spin.

Bellcrank webs are 6 mm wide, with 4.5 mm radius bosses around nominal 2.5 mm radius pivot/follower holes. The radial boss wall is 2 mm. The crank thicknesses remain 3.6 mm for CLOCK and 3.2 mm for WRITE. The rod-slot heads have material around the enlarged slots; guide clearances are not intended to preload the rods. These are geometric provisions, not a strength rating.

The controller rods are merged with their moving pickup pieces. This removes separate controller lever pivots, slotted direction conversions and the long clock transfer bar. The original CLOCK amplifier's two sliding follower interfaces remain. Clutch fork/ring contact, locking cam loads, bearings and elastic preload also remain; this is not a frictionless design.

Measured closed-solid printed volume, assuming the same material and infill:

| Moving printed material | Previous wall model | Direct control revision | Reduction |
|---|---:|---:|---:|
| Shared controller | 52.60 cm³ | 37.21 cm³ | 29.3% |
| Each bit | 68.94 cm³ | 66.02 cm³ | 4.2% |

Native hardware and elastic parts are excluded. The four added rollers contribute mass, so these percentages must not be presented as measured total moving-mass or actuation-force reductions. Details are in [Force and mass review](Force%20and%20mass%20review.json).

For N identical rows, ignoring friction and inertia:

- WRITE actuator force remains `N × selector force`.
- WRITE rod force is now `N × selector force`, versus `(2/3) × N × selector force` previously. The shorter rod travel increases rod load; the removed controller amplification cancels this at the actuator.
- CLOCK actuator force remains approximately `2.5 × N × local clock-bar force` because its stroke amplifier is retained.

Real loads add guide friction, pivot friction, roller resistance, clutch/bolt forces and acceleration. Rod buckling, timing scatter and actuator torque reserve still require measurement. Lower mass reduces inertia; it does not by itself reduce static clutch engagement force. Reduced bearing contact area must not be treated as a proportional reduction in dry friction.

## Packaging

Dimensions are nominal static CAD envelopes. They exclude mounting hardware, hand access and swept-motion allowances.

| Item | Previous wall model W × H × D | Direct revision W × H × D |
|---|---|---|
| Repeated bit, excluding coupler overhang | 317.2 × 144.2 × 58.3 mm | 301.2 × 112.2 × 58.3 mm |
| Shared controller | 230.0 × 171.8 × 64.5 mm | 83.0 × 175.8 × 70.8 mm |
| Row pitch | 144 mm | 112 mm |
| Eight rows plus controller, overall bounding box | 317.2 × 1324.4 × 64.5 mm | 333.2 × 1072.4 × 70.8 mm |

Each bit's face envelope is about 26% smaller than the previous wall version. The controller is much narrower but is mounted slightly left of the bits and has a deeper rear frame to clear its amplifier. The whole bank's bounding face area falls about 15%, while overall width and depth increase slightly. Compared with the original compact single-bit model (278.4 × 127.2 × 76.2 mm), the repeated bit is wider, shorter and thinner, with approximately 4.6% less face area. This is not a claim that the complete bank is optimally packed.

The lowered stages remove the reason for the previous 32 mm raised slave and the tall stepped bar. The chassis now supports both storage tracks directly. The controller's rear frame clears the moving amplifier, with additional ties around the rod corridor. Removable front bearing cheeks remain separate for assembly.

## Rotation speed

All 15 discovered external gear meshes have equal tooth counts. The CLOCK header remains 16T/16T; the data header remains 8T/8T. Restoring the original straight routes preserves the rotational parity and engaged speed ratios. Rotating a complete actuator changes its physical axis, not its transmission ratio.

DATA and POWER must use the same reference speed for their intended roles; Q is regenerated from POWER. Engaged logical 0/1 selects direction without changing the nominal speed magnitude. Real reversal necessarily passes through zero and clutch changeover can disconnect the drive. Constant instantaneous RPM through reversal is not guaranteed.

## One-bit prototype

Print one set of `bit` parts, one `control` set and one boundary's four `coupler` halves, then add the native hardware and elastics. [Model inventory](Model%20inventory.csv) gives the modelled one-bit/eight-bit quantities. Inherited U/L native identifiers refer to the original project sources; this is not a complete shopping list. The [development STLs](Development%20parts/) are in assembly coordinates and do not have qualified print orientations or supports.

The 6×6 mm rod segments meet with a nominal 0.4 mm end gap. Each joint uses a front-mounted splice plate and two LEGO 2780 friction pins, one through each widened rod-end shoe. The pins transmit axial load in shear; the connection does not require clamp preload. Side lips locate the shoes. The two pin centres are 14 mm apart. Printed pin fit, loaded stiffness and assembly access still need physical validation. Row pitch is 112 mm.

First test the roller axle fit, rod guides and clamp grip by hand. Verify that rollers rotate freely without pinching their crank arms. Then measure the force-versus-travel curve for one selector and one clock bar through a full cycle, including both directions and loaded clutch transitions. Measure each shared actuator's available force at the rod before adding rows. Scale to two rows before eight, recording peak input torque, stroke loss, deflection, backlash, missed engagement and temperature/wear.

Preserve the original setup/hold requirements around the clock edge. Test both D values, both stored Q values, WRITE enabled/disabled and both clock transitions. Do not infer an eight-row force rating from a one-row animation or from a successful collision screen.

## Reproduction and provenance

The original compact model is unchanged. Base commit: `f973a1cd144a77b852dfcbf43cae51444b4898e0`. The previous wall candidate is archived separately in the workspace's `analysis/wall-register-before-direct-drive` directory.

Using Python with numpy, trimesh, manifold3d and shapely, run in order:

1. `Source/wall_register.py` (calls `wall_direct_control.py`; uses the installed Studio LDraw library).
2. `Source/check_wall_ratios.py`, then `Source/check_wall_gear_phases.py`.
3. `Source/check_wall_register.py all`, `Source/screen_wall_native.py`, `Source/screen_wall_shafts.py`.
4. `Source/wall_force_review.py`, `Source/publish_wall_register.py`, `Source/wall_status.py`.

The original compact geometry, poses and phase report are required. `Previous moving material.json` preserves the volume comparison baseline for portable reproduction. A geometry change invalidates reports with an older SHA-256. Motion-related source changes require re-running their checks even when the static geometry is unchanged.
