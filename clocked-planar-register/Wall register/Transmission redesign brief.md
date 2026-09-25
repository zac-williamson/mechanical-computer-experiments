# Transmission and bearing architecture redesign

Status: implemented as development CAD; see Transmission redesign.md and current validation reports. Supersedes the assumption that existing axles and support-wall positions must be retained. The viewer and development exports now contain the rebuilt geometry. See Shaft layout inventory.csv for measured shaft-piece extents, not inferred from nominal part names.

## Two storage clutches share one power source

P denotes rotation of incoming POWER. X in the existing simulation denotes selected data (WRITE ? D : Q), so using X for POWER would obscure an important distinction.

| Free clutch gear | Present signed speed |
|---|---|
| master L072 at X=16 | +P |
| master L102 at X=-16 | -P |
| slave L072 at X=122 | -P |
| slave L102 at X=90 | +P |

Signs use the same global viewing axis. The swapped sides are intentional in the current actuator arrangement; preserve these assignments when changing the transmission. The independent clutch outputs M and Q must not be joined. Neither may be keyed to a common POWER shaft. The gears supplying opposite directions remain free-running relative to each clutch output until its ring engages.

Current routes: incoming +P meshes at X=32 into the shared -P power shaft. The -P shaft directly drives the +P free gear at each storage clutch through one external mesh. Two separate idler shafts generate +P from -P, at X=32 and X=138 respectively; each then drives its stage's -P free gear through another external mesh.

Preferred candidate: retain the reversing mesh at X=32 and use one mechanically connected +P distribution shaft to supply both reverse-branch clutch gears at X=-16 and X=122. Delete the two redundant drive gears at X=138 (slave POWER 16T 32 and slave reversing idler 16T 32). Terminate the -P distribution shaft shortly beyond its last remaining load at X=90. This removes one external mesh and two gears while preserving the four signed clutch inputs and unity speed magnitude. It does not remove either storage clutch or either clocked data gate.

The +P connection must be real, with an assembly phase that satisfies all meshes. Simply assigning matching animation speeds to disconnected shafts is not a transmission. Avoid retaining both reversing meshes with a connected shaft: that creates a redundant closed gear loop. Phase and collision checks must be rebuilt for the new graph; the old expected count of 15 external meshes must not be used as proof.

## Shaft decisions before bearing walls

- Incoming POWER: present two 96 mm axle pieces cover X=-112..-16 and -8..88, joined near -12, though the transmitting gear is at X=32. Move the input port beside that gear and use a short locally supported shaft unless the external interface requires the old left-edge port. No such left-edge constraint is retained in this proposal.
- Existing POWER distribution: axle pieces cover -18..46 and 48..144. With the X=138 drive deleted, the last gear is at X=90. Choose lengths and couplers around gear hubs, bearings and retainers, rather than retaining this 162 mm outer span.
- Reverse distribution: current separate 72 mm axles span -28..44 and 78..150. A shared branch needs a connection across the gap; deleting a gear pair does not automatically reduce total axle length. Compare shaft torsion, support count and assembly access before selecting stock lengths.
- Q feedback: the 256 mm axle spans -100..156 and connects gears at -92 and 150. Those gears are 242 mm apart. It cannot simply be shortened with those endpoints fixed. Review relocating the output takeoff toward the slave's inner side; otherwise keep the necessary route but use an explicitly supported and retained shaft assembly. Splitting into shorter pieces alone does not remove required transmission length.
- M, Q, selected data and worm/gate shafts: retain independent angular degrees of freedom. Select each length only after defining its keyed hubs, free-running gears, clutch sleeve engagement, bearing lands, retention and assembly insertion path.
- Rotated controller: analyse in its actual global axes. Metadata inherited from unrotated cores is not sufficient to locate support planes.

## Bearing structure derived from the new shaft schedule

Design a small family of transverse bearing webs spanning related axes, near load clusters. A common wall may support several independent shafts without joining their rotation. Do not force all bearings onto one plane when that produces long overhangs or conflicts with moving clutches.

For each shaft, record its loads, bearing stations, material surrounding the entire bore, shaft engagement through each land, gear overhang, axial stops and assembly access. Long distribution shafts may require intermediate supports, but unnecessary tightly constrained bearings can bind; number and placement are decisions, not a fixed two-bearings-everywhere rule. Couplers require engagement and retention, not an empty hole nearby. Every unoccupied hole must have an explicit purpose such as mounting, a pivot, or assembly access; otherwise remove it.

Rebuild transmission-bearing webs without importing legacy Bearing support solids. Preserve identified actuator working faces and anti-roll tracks separately, then deliberately attach them to the new structure. Do not create support by cutting axle tunnels or gear envelopes through arbitrary old walls. A complete bore and its load-carrying ligament must survive all clearances.

## Elastic elements and verification

The viewer currently displays elastic elements in reference shape while rigid parts animate. An apparently floating loop can therefore be a visualisation defect, a missing anchor, or both. Audit each loop's two named anchors and swept path before deciding. Do not remove a return/bias element based only on appearance. Update displayed elastic vertices when the physical anchor geometry is established.

Acceptance evidence must include: signed transmission graph in all clutch states; actual keyed connections and free-running interfaces; support/retention assignment for every shaft; complete bore sections and finite engagement; full-travel moving clearances; every elastic anchor attached; accessible assembly sequence. Inherited prescribed-motion animation and zero axle-crossing results do not prove any of these on their own.
