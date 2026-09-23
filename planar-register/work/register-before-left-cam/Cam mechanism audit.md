# Cam / roller / bolt: loaded mechanism review

**Decision: do not print the present complete mechanism as a confidence prototype. The short, offset bolt guide needs revision first.** This review isolates the cam lock; the master–slave architecture and simultaneous-input race are outside its scope.

## Findings from the actual geometry

| Item | Finding | Implication |
|---|---|---|
| Guide engagement | Guide is 7.6 mm high, but the rising stem leaves only 4.6 mm overlap at full lift. | The available bearing span shrinks exactly when the roller is highest. |
| Roller overhang | Near the end of the straight ramp, its centre is 9.94 mm above the guide top; at full lift, 11.8 mm. | Horizontal cam force creates a large overturning moment. |
| Rear offset | Roller centre Y27.6; guide centre Y19.6: 8 mm offset. | Even the upward lifting force twists the bolt into the guide walls. |
| Band offset | Its attachment is approximately 7.4 mm to one side and 1 mm forward of guide centre. | Increasing band strength also increases the tendency to cock the bolt; it is not a cure for a bad guide. |
| Guide play | ±3° tilt with ±0.15 mm translation causes **zero solid intersection** between the complete bolt and guide in 12 checked raised poses. Roller X shifts 0.83–0.89 mm. | The previous total 0.6 mm timing-error assumption is unsupported by the guide alone, before print error, pin play or flex. Other parts/cam were absent in this test; it is not a proof of an assembled tilted trajectory. |

The previously published friction formula accounted only for direct lateral force. It omitted the overhang moment, rear offset and return-band eccentricity. **Its apparent friction margin must not be used.**

## Force and flex screen

A rectangular-guide Coulomb-friction calculation balances the cam force and the wall reactions at both ends of the actual remaining stem overlap. It treats the cam roller as ideal, assumes a 2 N band, and varies friction rather than claiming a measured PLA coefficient.

Near WRITE displacement −0.45 mm (bolt lift 5.94 mm), the screen gives:

| Assumed guide friction coefficient | Required upward roller force | Horizontal cam force | Existing gross-beam cam deflection |
|---|---:|---:|---:|
| 0 | 2.0 N | 3.6 N | 0.019 mm |
| 0.05 | 3.54 N | 6.37 N | 0.033 mm |
| 0.10 | 87.2 N | 157 N | 0.81 mm |
| 0.15 or greater | No positive sliding equilibrium in this screen | — | — |

These are **conditional sensitivity results**, not predicted measured forces or a certified critical coefficient. Face-friction moments, compliance, local corner contact, bush drag and material variation are not solved. They show why the earlier “a 2 N band implies a lightly loaded cam” argument was inadequate. The calculated critical region is sensitive enough that this guide is a poor first-print candidate.

The gross-beam deflection uses the earlier unperforated cam approximation, assumed E=1800 MPa and rigid attachment. Pin-hole stress concentration, shear-key fit, local bearing and frame movement remain excluded; 0.81 mm is not a verified assembly deflection. It demonstrates that modest band tension can produce large structural demand when guide reactions amplify friction. A detailed FEA of the cam alone would not resolve the more fundamental guide/contact uncertainty.

A separate bolt-stem beam screen under residual keeper side load predicts approximately 23.8 MPa bending stress at 50 N and 47.7 MPa at 100 N, using a 5.8 mm tip lever arm. These are screening forces, **not** a demonstrated mapping from 0.1 Nm input torque. The actual distributed keeper contact, guide support, printed orientation and load release need measurement. The bolt must not routinely be asked to release against a driven memory actuator.

With the cam removed from contact, the off-centre band can also cock the raised bolt. The simple guide model's return friction limit falls from about 0.45 at low lift to 0.27 at full lift. A stronger band scales both useful pull and friction-generating reaction together.

## What must change in the mechanism

1. **Guide the roller carrier near where the force enters.** Use two separated guide lands or a guided crosshead, keeping the roller within their support span rather than above a short stem sleeve.
2. **Maintain guide overlap through the entire lift.** Do not lose almost half the bearing length when opening.
3. **Centre the cam and return-band loads on the supported carrier as closely as possible.** Keep the narrow locking tip as an extension of that supported carrier; do not use it as the sole anti-tilt guide.
4. **Recalculate the cam sequencing with actual maximum play plus flex.** Do not recover timing margin by assuming a tighter print than the 0.4 mm-nozzle process supplies.
5. **Check attachment and frame deflection together.** Thickening the orange plate alone would transfer increased force into the guide and LEGO-pin joint without curing wedging.

These are design requirements, not an integrated CAD revision. No assembly STL has been altered in this review. Once the carrier layout is chosen, its complete travel and print orientation need new checks before it becomes a print candidate.

## Small-test scope after that revision

The existing `Latch coupon` tests an earlier bolt/keeper arrangement moving along Y. It does **not** reproduce this Z-moving cam, roller offsets, guide overlap or orange pin joint, and must not be used to qualify them.

The useful replacement test assembly must reproduce the actual cam, roller/axle/retainer, carrier guide, band anchors, keeper, and cam mounting joint. Drive the cam manually first, measuring force through the full stroke in both directions and at both keeper positions. Then use the actual WRITE actuator. Preserve representative support spans so the fixture does not hide frame flex.

Record: printed dimensions and free play; band force versus extension; peak cam force in each direction; complete bolt withdrawal and insertion; roller/guide displacement under load; pin-joint slip; performance after repeated cycling. Test deliberate mid-pocket stops using the compliant band return. These establish whether the revised cam lock operates, not a complete register torque rating.

Only after measuring the working mechanism's demand should we test the intended 0.1 Nm on each relevant axle with the actual driveline and output load. A 100 mm torque arm with 1 N tangential force represents 0.1 Nm, but does not establish the corresponding bolt load through a worm train. No motor RPM or filament datasheet substitutes for these measurements.

## Evidence and reproducibility

- `Loaded guide screen.json`: 751 travel positions, seven friction values; actual geometry source hashes and explicit statics assumptions.
- `Bolt guide play.json`: exact solid intersection checks for the 12 tilted guide-only poses.
- Reproduce using `Source/audit_cam_loading.py` and `Source/audit_bolt_play.py` in the supplied CAD Python environment.
- Prior nominal collision checks remain nominal geometry checks; they omit these free-play poses and elastic deformation.

The general eccentric-load binding concern is also explained by [igus's linear-guide design guidance](https://www.igus.com/company/linear-guides-the-2-1-rule-ca). Its product-specific 2:1 rule is **not** used as a PLA design limit here; our figures come from the explicit geometry and stated model.
