# Transition timing audit — do not print this revision yet

**Scope update:** The user has revised the timing question to match Ben Eater’s SAP. See [SAP register timing](SAP%20register%20timing.md): clock capture is separate from load-enable changes. The following is the bare-latch simultaneous-transition stress audit under the earlier old-bit assumption, not a requirement that Ben’s chips resolve setup/hold violations.

Earlier stress-test requirement: when D reverses at the same time as WRITE changes from 1 to 0, retain the **old** bit. The current mechanism does not establish this. It has a race between D moving the memory actuator and WRITE disconnecting its drive. The cam sequences isolation and bolt insertion, but it does not prevent the memory moving during the isolation stroke.

## What was wrong with the previous evidence

The previous transition audit moved WRITE first and memory second. Enumerating all input pairs did not make those paths simultaneous. The viewer used `qe > 0.4` and bolt lift greater than 5.8 mm to permit memory movement; that condition enforced the desired behaviour instead of deriving it from the clutch. Clutch ring displacement was a function of carriage position only and omitted reversal history. Thus the previous collision results remain sampled geometry checks, not functional qualification of these transitions.

## Revised model and evidence

The viewer now includes all 12 directed changes, both possible stored bits whenever starting in HOLD (18 initial-state cases), and six timing/contact examples (108 runs). It allows the actuators to move concurrently. Each clutch ring has direction-dependent fork play: its previous position stays unchanged while inside the interval carriage position ±0.4 mm, then follows the pushing face. This is the existing nominal CAD allowance, not measured LEGO tolerance.

Torque transfer is considered possible as soon as dogs contact, without waiting for full axial engagement. Angular take-up can be zero when a driving flank is already aligned. A delayed-reversal example illustrates the opposite ordering; its delay is an assumption, not a measured upper bound. The model stops the memory immediately when the D clutch separates: even this optimistic no-coasting assumption produces failures.

The ring extends to ±11.2 mm and the facing gear begins at ±12 mm. Consequently 0.8 mm ring displacement is the earliest outer-envelope overlap. A native triangle-surface sweep (360 relative phases, 0.025 mm axial increments) first detects contact at 1.95 mm. This is a sampled geometric observation, not a calibrated engagement threshold or a guarantee about chamfers, deflection or loaded contact. Both the conservative envelope and this mesh-contact example are included. The mesh scan is saved separately.

Time units are normalized: WRITE carriage speed is 1 mm/unit. Equal carriage speeds are an illustrative favourable assumption, not an assertion that equal motor RPM produces identical travel histories. The other ratios are sensitivities, not measured speed tolerances.

## Concrete counterexample

Start with D=1, WRITE=1, Q=1. Reverse both inputs. With the mesh-contact example, no reversal take-up delay and equal carriage speeds:

- WRITE retreats from +3.75 mm; its ring follows the withdrawing fork face after the nominal lost travel.
- D stays coupled until WRITE reaches approximately +1.55 mm.
- During that interval the memory carriage moves from −3.75 mm to approximately −1.55 mm.
- D then disconnects. The memory is outside either full-insertion pocket position (which requires |q| ≥3.3 mm within the operating stroke).
- The return band cannot push the bolt through the solid keeper. The assembly finishes HOLD **not locked**, with the output at the edge of its clutch-contact region, not a reliably retained state.

For the 1.5× memory-speed example, the old output clutch releases around normalized t=1.47; D isolation occurs around t=2.21. This violates the requested ordering directly. Memory stops around −0.45 mm, leaving Q undriven. At 4×, memory reaches the other pocket and locks the **new, wrong** bit. With enough reversal delay the old bit survives. This range of outcomes is the race; backlash cannot be treated as a safety feature.

## All twelve transitions

Pairs below are (D, WRITE). “Conditional” means the prescribed contact-event examples reach the intended endpoint, **not** that the axle loads, tooth phases, reaction times or physical operation pass.

| Start | End | Result |
|---|---|---|
| 00 | 01 | Conditional: acquire 0; checked starting with either stored bit |
| 00 | 10 | Conditional: remain locked; D isolated |
| 00 | 11 | Conditional: acquire 1; simultaneous reversal and unlock |
| 01 | 00 | Conditional: retain 0 if initially settled |
| 01 | 10 | **Fails old-bit retention guarantee; can stop between pockets or capture 1** |
| 01 | 11 | Conditional: switch to 1 while writing; output transfers through a neutral interval |
| 10 | 00 | Conditional: remain locked; D isolated |
| 10 | 01 | Conditional: acquire 0; simultaneous reversal and unlock |
| 10 | 11 | Conditional: acquire 1; checked starting with either stored bit |
| 11 | 00 | **Fails old-bit retention guarantee; can stop between pockets or capture 0** |
| 11 | 01 | Conditional: switch to 0 while writing; output transfers through a neutral interval |
| 11 | 10 | Conditional: retain 1 if initially settled |

Starting from an unsettled previous write adds further interruption cases; twelve input pairs alone do not cover the continuous internal state or input chatter.

## Ordering and axle binding

There are three distinct requirements:

1. Isolate D before the memory loses the old output clutch engagement.
2. Isolate D before the lock bolt can obstruct memory travel.
3. Actually seat the bolt in the **old** pocket, rather than stop over the solid keeper.

The current mechanism does not enforce 1 or 3. Nominal cam geometry favours 2: first bolt-tip intrusion is at WRITE q≈−0.75 mm, whereas the conservative earliest-contact clutch envelope is separated below q≈+0.4 mm on withdrawal. That is a nominal 1.15 mm sequencing interval, not a time margin or a force proof. The previous 0.6 mm cam-error assumption was not measured and is not a worst-case tolerance stack.

| Axle / load path | What remains unresolved |
|---|---|
| D → WRITE dog clutch → memory worm | Loaded dog withdrawal can resist WRITE motion. If the bolt obstructs a still-driven actuator through lag, flex or sticking, the worm drive can stall or overload. Disconnection stops commanded drive, not necessarily stored torsional energy or inertia. |
| WRITE → actuator → cam | The steep cam, guide side load, band force and loaded clutch release can stall WRITE; no finite maximum closing time has been established. |
| POWER → opposing-direction memory clutch gears | Nominal rigid axial envelopes do not allow the ring to reach both gears at once (one needs r≥+0.8 mm, the other r≤−0.8 mm). This excludes that particular rigid double-engagement path, not all power binding. Tooth-on-tooth entry, shallow engagement and output reversal under load remain unqualified. |
| Q | Its inertia and connected load affect clutch impact and release. A neutral interval means Q can coast or stop; it is not a commanded 0 or 1. |

Locking the memory carriage fixes its **axial** position; it does not inherently brake Q. A power jam needs an actual rotational reaction path such as obstructed clutch engagement or a blocked output. The current kinematic model cannot prove every axle free from binding, and it should not label a possible jam as an observed one.

## Backlash and minimum/maximum reaction time

For a measured angular take-up β degrees and relative speed n RPM at that interface, the ideal take-up time is β/(6|n|) seconds. Gear ratios must reflect each interface's backlash to the relevant shaft. Fork axial play is a separate hysteresis, not the same quantity as angular dog clearance. Motor reversal, worm/reaction-lever phase, output load, guide friction, elastic deflection and coast add further delays.

The conservative earliest angular reaction is **zero take-up delay**. A finite latest reaction cannot presently be justified: relative speed can fall to zero or a tooth/guide can stall. The available CAD contains neither measured backlash bounds nor a motor torque-speed/load model. Therefore a finite numerical timing certificate would be invented. Equal nominal D, WRITE and POWER RPM does not remove those uncertainties. Inertia and available torque also make absolute RPM relevant.

To guarantee old-bit retention, the maximum memory movement before isolation and residual motion must fit inside the old pocket's remaining travel margin: nominally **0.45 mm** toward the other bit from either ±3.75 mm endpoint. This is much stricter than merely retaining partial output clutch engagement. In timing form, the design needs a justified bound on the integral of memory travel from simultaneous reversal through complete D isolation, including recoil/coast; or a positive mechanism preventing that movement. Incidental backlash cannot supply a guaranteed minimum delay across all phases.

## Required before a print recommendation

The closing race needs a mechanism or an input timing contract that holds the old memory position throughout D isolation. A timed contract would require D remain stable until closing completes, which does **not** meet the requested arbitrary simultaneous-change behaviour. A mechanical revision must provide deliberate isolation priority or bounded data delay and then be checked against worst-case reaction times; a faster nominal WRITE actuator alone is insufficient.

Before qualification, measure actual clutch first/last torque-carrying axial positions in both directions, fork play, angular lost motion at all relevant phases, loaded release force, actuator reversal travel versus angle, cam/guide friction and clearance, and coast under the intended Q load and 0.1 Nm torque. These can be investigated on LEGO subassemblies and a small guide/cam coupon rather than committing to the full six-hour print. They still would not cure the demonstrated sequencing race by themselves.

**Decision: do not print the complete current revision expecting a reliable register.** The geometry clearance reports are retained, but the functional transition audit fails the stated old-bit retention requirement. The viewer is now an explicit contact-event investigation, not a demonstration that the mechanism works.
