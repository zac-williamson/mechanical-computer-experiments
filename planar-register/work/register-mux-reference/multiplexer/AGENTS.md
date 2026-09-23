# Current multiplexer

This directory is the only current multiplexer design. Start with README.md. Source/build.py is the supported generator entry point; root-level STLs and Viewer.html are outputs. Source/generate_parts.py and its committed construction-inputs reproduce the accepted revision. Source/construction contains preserved upstream provenance; do not bulk-read or execute it for routine builds. Do not reconstruct or restore older variants unless the user asks. Do not create a second design folder when editing this design.

Preserve worm -> reaction gear -> pivoting lever endpoint-release operation, main X-axis axles, gears -> clutch -> carriage ordering in Z, LEGO rotating hardware and friction-pin joints. Both worm and clutch axles have Y=10.2 mm and are 16 mm apart in Z. Preserve smooth printable bearing surfaces, open visibility and a two-pin minimum for removable structural attachments. Normal-operation target is 0.2 Nm; a deliberately jammed mechanism is excluded. Do not use Bambu Studio.

After edits, update geometry, viewer, print layout and relevant clearance checks together. Do not claim physical qualification from CAD checks. Keep changes confined to the multiplexer unless explicitly authorized.
