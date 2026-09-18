# Register cam test

Two actuators isolate the register cam and lock. CAM CONTROL withdraws the bolt before its clutch allows TEST DRIVE to move the other carriage.

`Viewer.html` shows the complete assembly, input labels, individual parts and an illustrative operating sequence. `Assembly manifest.json` identifies all 22 printed parts and hardware; `LEGO parts.csv` is the hardware inventory.

Print `Print plate 1.stl` and `Print plate 2.stl`, or use the individually oriented files in `Print parts`. Inspect support placement around the original clutch fork. Do not assume these plates are support-free. Axle bores face vertically in the supplied orientations. Assemble with LEGO friction pins; each base-mounted bearing uses at least two pins. Use 3 elastic bands.

Slide the bolt into the integral front-bridge guide before installing the rear bridge. Install the follower and cam, then the return band. The cam withdraws the bolt 5.2 mm. Only relock with the storage/test carriage at one of its end pockets. Check free motion by hand before applying drive torque.

Current geometric results are recorded in `Printed clearance checks.json`, `Lock checks.json` and `Print checks.json`. These do not validate loaded operation or ABS tolerances. The actuator's physical reliability remains under test.

## Rebuild

Use Python with `Source/requirements.txt` and an installed LDraw library. Run `Source/build.py`, `Source/print_layout.py`, then `Source/publish.py`; run the check scripts after changes. Rebuild ../register first; this rig takes its K and W components directly from that assembly. No slicer application is required.
