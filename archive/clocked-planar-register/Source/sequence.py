"""Clock sequencer geometry, millimetres. Bounds are assumptions, not measurements.

Positive rail travel closes master then opens slave. A positive-locking cam
groove must drive each input fork; a return spring alone is not sufficient.
This module models required fork positions, not completed transmission CAD.
"""
from math import sqrt

RADIUS = 3.6
LIFT = 3.8
SLOPE = 1.2
AMPLIFICATION = 3.0
ACTUATOR_TRAVEL = (-3.755874, 3.749041)
RAIL_LIMITS = (3 * ACTUATOR_TRAVEL[0], 3 * ACTUATOR_TRAVEL[1])
RAMP_WIDTH = (LIFT + RADIUS * (sqrt(1 + SLOPE**2) - 1)) / SLOPE
RING_TRAVEL = 3.35


def slave_lift(s):
    """Exact circular follower above a straight ramp and flat crest."""
    u = 1 + RAMP_WIDTH - s
    if u <= 0:
        return LIFT
    tangent = RADIUS * SLOPE / sqrt(1 + SLOPE**2)
    if u <= tangent:
        return LIFT - RADIUS + sqrt(RADIUS**2 - u**2)
    return max(0.0, LIFT - RADIUS - SLOPE * u
               + RADIUS * sqrt(1 + SLOPE**2))


def gate(s):
    return -1.5 + (RING_TRAVEL + 1.5) * min(1.0, max(0.0, (s - 7) / 3))


def pose(s):
    return dict(master_lift=slave_lift(-s), slave_lift=slave_lift(s),
                master_ring=gate(-s), slave_ring=gate(s))


def selected_data(d, write, q):
    return d if write else q


def bellcrank_track(s, stage, gate_x):
    """Follower centre in translating rail coordinates; pivot Z=35, input arm 10, output arm 12.

    A second vertical slot at the output fork absorbs the output pin's Z arc.
    This curve is a manufacturing requirement, not a substitute for a linkage.
    """
    q = pose(s)[stage + '_ring']
    sin_theta = -q / 12
    return (gate_x - 10 + 10 * sqrt(1 - sin_theta**2) - s,
            35 + 10 * sin_theta)
