# Ben Eater register timing and implications for this latch

Ben's A and B registers use 74LS173 devices with independent clock and load-enable inputs. Data is captured on the positive clock edge when enabled. Load-enable is not a transparency control. The device requires data/enable setup and hold timing around that edge. See [A register schematic](https://eater.net/schematics/a-register.png), [B register schematic](https://eater.net/schematics/b-register.png) and [TI 74LS173 datasheet](https://eater.net/datasheets/74ls173.pdf).

The microstep counter receives inverted CLK, so its normal step advance occurs on the system's falling edge. The following rising edge captures the selected transfer, after time for control and data to settle. This describes intended synchronous operation, not a claim that EEPROM glitches or all hardware timing hazards are impossible. See [control schematic](https://eater.net/schematics/control.png).

Ben's ADD microcode changes from `RO|BI` (RAM drives bus, B enabled) to `EO|AI|FI` (ALU drives bus, A and flags enabled). Consequently the bus data and B's load-enable can change as part of the same microstep boundary. A's enable also asserts while the bus source changes. They need not arrive at precisely the same physical instant. SUB has the analogous change with SU asserted. See [Ben's original microcode](https://github.com/beneater/eeprom-programmer/blob/master/microcode-eeprom-with-flags/microcode-eeprom-with-flags.ino).

There is no separate ALU result register in this design. The adder/subtractor is combinational; arithmetic results are captured in A. The flags and display output register are separate storage. See [ALU schematic and description](https://eater.net/8bit/alu).

## Consequence for the mechanical design

The correct target is to retain the data sampled at a defined capture phase. Ben's design does not require a deterministic old/new choice if D violates setup/hold at the actual capture edge. Simultaneous microcode-enable and data changes are safe only because capture is a separate event.

The current mechanism is a level-sensitive latch. Connecting AI or BI directly to its WRITE shaft does not reproduce Ben's edge-triggered register. In particular, ADD feeds A through the ALU and bus back into A: while a transparent A latch is open, changing A changes its own input again. For A=3 and B=2, the intended one-time capture is 5; continued transparency exposes the next sum 7, then further changes (actual asynchronous bit behaviour need not follow clean integer steps).

A mechanically sequenced capture system can use this latch as a building block, consistent with the original scope allowing clock sequencing to be separate. However, it must preserve old source values during acquisition, close/isolate the receiving storage before bus/control changes, and prevent ALU feedback while A updates. Two storage stages operated with non-overlapping phases are one candidate; a separate temporary result stage plus additional microsteps is another. Neither has been implemented or qualified here. Simply connecting WRITE=enable AND clock, or slowing all motors, is not a demonstrated solution.

The simultaneous-close race in the transition audit remains a valid limitation of the bare latch. It is not by itself proof that an externally sequenced mechanical SAP cannot use a revised/qualified version of it. Treat arbitrary simultaneous D/WRITE capture as a stress case; derive allowed operational timing from the eventual clock architecture. No full-register print recommendation follows until that timing and mechanical binding behaviour are qualified.
