# Multiplexer validation

Install the packages in `requirements.txt` in a Python environment. Run these commands from this directory:

```
python check_prints.py
python check_structure.py
python check_pivot.py
python check_trace.py
python check_clearance.py
python check_contacts.py
python check_switching.py
python check_print_layers.py
python check_native.py
```

- `check_prints.py`: mesh solidity, print-bed alignment and complete plate count.
- `check_structure.py`: solid roof-pillar connection and hub/carriage clearance.
- `check_pivot.py`: both bridge supports, axle/bush envelopes and lever-angle sweep.
- `check_trace.py`: full lever body along the switching trace, carriage/frame and pivot clearance. Allows only very small sampled stop-contact overlap (below 0.005 mm³).
- `check_clearance.py`: independent carriage/lever positions and full worm envelope. Separately identifies positions already prohibited by intentional stop-pad contact.
- `check_contacts.py`: 486 lock/release cases using stop offsets of ±0.25 mm and the gear-offset cases in the contact tables.
- `check_switching.py`: recomputes coupled switching in both directions.
- `check_print_layers.py`: reports unsupported layer growth at 0.2 mm layers across all printed parts; support flags require interpretation, particularly at the clutch fork.
- `check_native.py`: assembly-wide sampled mesh contact inventory. It reports candidate contacts rather than asserting that intentional pin fits, gear meshing, shaft fits and clutch contacts are faults. The band remains at its neutral geometry in this audit.

`common.py` and `contact.py` provide shared geometry operations. `Data/` contains required gear/lever contact matrices for the current tooth profile; run `python rebuild_contact_tables.py` to regenerate them. `Data/tooth_profile.npz` is the contact profile input and must be updated if the teeth change. The motion JSON files in the component directory are also used by the register and cam-test checks.

These are sampled geometric checks, not stress, friction, wear or fatigue simulations. The native contact inventory requires interpretation. A passing run does not establish a load rating.
