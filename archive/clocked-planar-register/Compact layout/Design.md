# Compact clocked register

The current static complete-assembly envelope is **278.4 × 127.2 mm in XZ**, a **50.42% area reduction** from the earlier full-size clocked assembly. Overall Y depth is **76.23 mm**, reduced from 99.4 mm. These include shafts, retainers and frame, but are reference-pose dimensions rather than a swept-motion packaging claim.

## Components and routing

There are four planar-derived worm actuators: WRITE selector, master storage, output storage and CLOCK sequencer. Five clutch rings comprise the WRITE selector, two storage outputs and two input-disconnect gates. Eight elastic loops serve four actuator returns, two bolts and two passive fork biases.

D, WRITE, CLK and POWER enter the negative-X side; Q exits positive X. CLOCK uses one reversing mesh. Its inversion is absorbed by the sequencing direction. The selected signal uses one mesh to the master gate; master output directly drives the slave gate input. Q feedback uses one axle and one mesh at either end. The D header's single mesh is oriented in Y around its receiving shaft to clear the master ring without an additional idler or greater Y envelope.

Each storage power branch has unit speed magnitude. Master and slave clutch-side assignments differ to absorb the interstage inversion. The model checks the actual Q shaft's angular velocity; its settled ratio to POWER is +1 or -1, not a value inferred merely from carriage position.

## Clock sequence

The original clock actuator drives a 12 mm / 30 mm amplifier. Its approximately ±3.75 mm stroke produces approximately ±9.375 mm common-bar travel. A rising clock withdraws master drive, permits the master bolt to seat, lifts the output bolt and admits master data to the output actuator. Falling clock reverses the sequence.

Each input fork has 4 mm of one-sided float: elastic bias permits its dogs to wait for angular alignment, while the bar positively withdraws it. The rear fork guides retain their compact positions; only each front clutch finger is offset 6 mm. The gate gears sit alongside their LEGO clutch connectors. Nominal first tip contact is at 6.8 mm bar travel, before full insertion. The model tracks the measured CAD angular windows and about 64 degrees of dog backlash.

Changes to D or WRITE at the capture boundary violate the setup/hold assumption. Those cases are explicitly indeterminate rather than assigned a guaranteed bit. Four such nominal cases leave the master outside a locking pocket. The mechanism must not be described as accepting arbitrary asynchronous captures.

## Structure and manufacture

Supports follow the actual shaft and pin datums. All sixteen routing shaft stacks have opposing nominal thrust faces; grip strength remains a separate requirement. Carriage halves retain bearing-flat print orientations.

The rear clock-pivot bearing is a separate bridge, attached with two non-collinear LEGO friction pins. Its bearing axis and amplifier position are unchanged. This removes the former roughly 31 mm unsupported ledge beneath the base's pivot region. The bridge prints flat on its Y face. Pin-bore roofs and counterbore lips still need bridge-quality verification; the common cam/fork bar has support-dependent features and is not designated support-free.

See [validation status](Validation%20status.md) for the current evidence, failures and limitations. The planar register and multiplexer have not been regenerated or modified by this work.
