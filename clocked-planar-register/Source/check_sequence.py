"""Bounded sequencing checks. Does not certify a mechanism or motor timing."""
from pathlib import Path
from itertools import product
import json
import numpy as np
from sequence import pose, selected_data, RAIL_LIMITS, RAMP_WIDTH, LIFT

OUT = Path(__file__).resolve().parents[1]


def run():
    # Deliberately includes possible pickup much earlier than full engagement.
    assumptions = dict(first_dog_contact_mm=[0.8, 1.95],
                       fork_position_error_mm=1.0,
                       cam_phase_error_mm=0.25,
                       bolt_vertical_error_mm=0.2,
                       required_tip_lift_mm=3.2,
                       extra_tip_clearance_mm=0.3,
                       angular_backlash_deg=[0, 30, 60],
                       status='UNMEASURED engineering bounds; no RPM qualification')
    rail = np.linspace(*RAIL_LIMITS, 20001)
    failures = []
    min_clearance = float('inf')
    overlap_samples = 0
    for s in rail:
        # Worst-case gate advanced in either direction, bolt lift retarded.
        pm = pose(float(s - assumptions['cam_phase_error_mm']))
        pp = pose(float(s + assumptions['cam_phase_error_mm']))
        ring_m = max(pm['master_ring'], pp['master_ring']) + assumptions['fork_position_error_mm']
        ring_s = max(pm['slave_ring'], pp['slave_ring']) + assumptions['fork_position_error_mm']
        active_m, active_s = ring_m >= .8, ring_s >= .8
        overlap_samples += bool(active_m and active_s)
        for stage, active in [('master', active_m), ('slave', active_s)]:
            if active:
                lift = min(pm[stage+'_lift'], pp[stage+'_lift']) - .2
                clearance = lift - 3.2
                min_clearance = min(min_clearance, clearance)
                if clearance < .3 - 1e-9:
                    failures.append(dict(s=float(s), stage=stage, clearance=clearance))

    # Combinational truth table only. Simultaneous clock/data events cannot be
    # assigned deterministic old/new values without setup/hold specifications.
    cases = []
    states = list(product([0, 1], repeat=3))  # D, WRITE, CLK
    for start, end in product(states, repeat=2):
        if start == end:
            continue
        for q in [0, 1]:
            rising = start[2] == 0 and end[2] == 1
            inputs_changed = start[:2] != end[:2]
            timing_fault = rising and inputs_changed
            expected = None if timing_fault else (
                selected_data(start[0], start[1], q) if rising else q)
            cases.append(dict(start=start, end=end, initial_q=q,
                              expected_q=expected,
                              classification='setup/hold undefined' if timing_fault
                              else 'defined protocol behavior'))
    assert len(cases) == 112
    assert not failures and overlap_samples == 0
    assert min_clearance >= .3 - 1e-9
    # Specific failure of naïve CLK AND WRITE is excluded by the feedback mux:
    for d, q in product([0, 1], repeat=2):
        for w0, w1 in [(0, 1), (1, 0)]:
            c = next(c for c in cases if c['start'] == (d, w0, 1)
                     and c['end'] == (d, w1, 1) and c['initial_q'] == q)
            assert c['expected_q'] == q
    result = dict(scope='Analytic rail/fork sequencing and protocol truth table ONLY',
                  assumptions=assumptions, ramp_width_mm=RAMP_WIDTH,
                  rail_limits_mm=RAIL_LIMITS, samples=len(rail),
                  possible_simultaneous_drive_samples=overlap_samples,
                  minimum_tip_clearance_during_possible_drive_mm=min_clearance,
                  sequence_failures=failures,
                  input_transitions=56, initial_state_cases=112,
                  defined_cases=sum(c['expected_q'] is not None for c in cases),
                  undefined_timing_cases=sum(c['expected_q'] is None for c in cases),
                  cases=cases,
                  not_validated=['Linkage stiffness and backlash bounds',
                      'Actual dog clutch disengagement under torque',
                      'Dynamic bolt seating or a mid-stroke storage carriage',
                      'All-part collision sweep and physical gear phases',
                      'Motor-speed-dependent settling and setup/hold',
                      '0.1 Nm strength and fatigue'])
    (OUT/'Sequencing checks.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ['cases','assumptions']}, indent=2))


if __name__ == '__main__':
    run()
