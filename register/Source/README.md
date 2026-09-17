# Regenerating this prototype

Use Python 3 with the packages in requirements.txt. Set LDRAW_DIR to an installed LDraw library containing parts/ and p/. The default points to the existing Studio 2.0 LDraw library on macOS. No slicer is invoked.

Run build.py, then print_layout.py and publish.py. For verification run check.py, native_check.py, mount_check.py and gear_check.py. Run scripts from this directory or by absolute path. Outputs go to the parent directory; set REGISTER_OUTPUT to a different existing directory to generate a separate copy.

Inputs contains the final multiplexer mesh components and its reference assembly/pose data used by this design, not intermediate design iterations. New structures are parameterised in build.py; inherited actuator components are mesh inputs. This is not a fully parametric solid-CAD model. Viewer.html in Inputs is reference data, not the new register viewer.

LEGO visualisation geometry is loaded from your local LDraw installation. LDraw part authors retain their respective credits and licence terms; those part source files are not redistributed here. The embedded reference and output viewers include derived visualisation meshes. LEGO is a trademark of the LEGO Group; this is an independent prototype.
