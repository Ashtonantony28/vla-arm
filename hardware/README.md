# hardware/ — arm mechanics, budgets and controller board as code

Everything here is generated from code and checked by tests. **Nothing has been printed, fabricated or
measured.** Kinematics, masses and servo ratings come from `../spec/kinematics.yaml` (never edited here).

| dir | what | regenerate |
|---|---|---|
| `cad/` | CadQuery parametric parts (base with 61816 yaw-bearing seat + desk-clamp tabs, shoulder bracket with turntable and counterbalance post, upper arm, forearm, wrist pitch/roll bracket, rack-and-pinion parallel gripper with TPU pads, wrist camera mount + gripper cap), STS3215 servo/horn from the datasheet, 6701/61816 bearings, M2/M3 screws, involute pinion, camera board; assembly + FK, wall/bed/mass checks; `audit.py` pairwise clash audit (zero pose, every joint swept, random poses, support graph); `render.py` offscreen renders | `python -m cad.build` -> `cad/out/*.stl, *.step, cad_report.md`; `python -m cad.audit --random 40` -> `cad/out/clash_report.md`; `PYOPENGL_PLATFORM=egl python -m cad.render` -> `cad/out/renders/` |
| `analysis/` | static holding torque per joint, shoulder spring counterbalance sizing, 12 V current budget + PSU sizing, servo-bus timing | `python -m analysis.report` -> `analysis/budget_report.md` |
| `board/` | STM32G474 controller schematic in SKiDL (explicit-pin parts, no KiCad libs), own ERC, MCU pin map with AF numbers, BOM | `python -m board.generate` -> `board/out/controller.net, bom.csv, mcu_pins.md, erc_report.md`; checklist in `board/design_review.md` |
| `tests/` | pytest for all of the above | `python -m pytest` |

Setup (Python 3.13):

```
cd hardware
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python -m cad.build && .venv/bin/python -m analysis.report && .venv/bin/python -m board.generate
.venv/bin/python -m pytest -q          # ~25 min, CAD booleans dominate
```

STEP files in `cad/out/` are git-ignored (regenerate); STLs are kept (all < 3 MB).

## Mechanical design in one paragraph
Every pitch joint uses the same pattern: the servo case sits in an "inner-layer" cradle of the parent link;
the child straddles it with two clevis plates in an "outer layer" (horn plate bolted to the horn, idler plate
running on a stub the parent cradle carries coaxially with the output shaft). Parent and child therefore
occupy different y-layers near each joint, which is what lets the elbow fold +-155 deg. The wrist-roll servo
case rides in the gripper with its horn bolted to the wrist bracket (the 45 mm pitch-to-roll distance in
the spec is too short for the case to sit behind the horn). The gripper servo sits above the roll servo and
drives a module-1.5, 25-tooth pinion between two racks (exactly linear: 30 mm per jaw at 1.60 rad = spec's
60 mm). The shoulder counterbalance is a cable from a pin on the upper arm over a pulley on a post of the
yaw-fixed bracket to an extension spring: an equivalent zero-free-length spring, torque k*a*b*cos(q).

## Verified by code vs. needs a human
Verified by code (tests): each part is one valid closed solid; joint-axis distances measured from the parts'
own B-rep features equal the spec within 0.5 mm, also after placing them with the spec FK; TCP and gripper
opening match the spec; no interference between any parts at zero pose; adjacent parts clear across every
spec joint range except shoulder-down (see below); min wall >= 2 mm (ray-cast local thickness; gear teeth
excluded); every part fits a 220x220x250 mm bed; mass estimate vs spec; torque/current/bus arithmetic
against hand-checked cases; board ERC clean and proven to catch an open pin, a rail short and a missing
decoupling cap; e-stop topology (firmware cannot override the loop).

Clash audit (`cad/audit.py`, 2026-11-24): every pair of the 45 solids (printed parts, servo cases, horns,
bearings, every screw, pinion teeth, camera board) is checked exactly at zero pose, at 25 poses per joint across
the full spec range and at 40 random poses; overlaps > 0.001 mm3 fail, as do unexplained contacts closer than
0.25 mm. Running contacts (horn on case, bearing races, gear mesh, jaw guides) are declared in the audit with
their reason, and a contact graph proves no part floats. Servos are retained by M2 case screws into their
cradles (roll and gripper servos by the gripper cap); horns by 4 x M3 plus the spline screw.

Must be checked by a human / datasheet before ordering or printing:
- **Servo geometry** (`cad/params.py: Servo`) now follows the STS3215 datasheet drawing (45.23 x 24.73 x 35,
  output 12.5 mm off case centre, 20 mm aluminium horn). Still confirm with calipers on a real servo,
  especially the case-screw pattern and horn PCD.
- Bearing 61816 / 6701 dimensions and masses, heat-set inserts (screws are modelled into plain holes),
  print orientation/supports.
- Spring: pick a real extension spring for the rate/extension in `analysis/budget_report.md` and re-run.
- Servo response latency and register map used in the bus timing (assumed 100 us; addresses 0x38-0x46).
- Board: every `verify`/`TBD` BOM line, the medium-confidence AF mappings and recalled LQFP-64 pad numbers,
  servo bus logic level, servo connector type — see `board/design_review.md`.

## Known spec conflicts (spec/ not edited)
- `shoulder_pitch` max is +0.60 rad since spec v1.2 (`spec/DECISIONS.md`): the real servo case is offset from
  its output, which forced the 61816 bearing and a wider base, and the upper arm now reaches the base at
  ~+0.63 rad. Keeping +0.75 would need the yaw servo mounted inverted.
- The counterbalance spring sized in `analysis/budget_report.md` does not fit the post (about 145 mm at full
  extension vs about 80 mm available). Options: a rear tail with a second pulley, 2:1 reeving, or a stronger
  shoulder actuator. The spring is not modelled until one is chosen.
- `armctl`'s capsule collision model was fitted to the previous CAD and should be refitted to these parts.
- `elbow_pitch` +2.70 rad with the shoulder level drives the wrist into the base past ~+2.36 rad
  (pose-dependent self-collision; needs the host-side collision check).
- 0.25 kg at full reach exceeds the 40 % continuous rating at shoulder (even counterbalanced) and elbow;
  continuous limit at full reach is ~0.20 kg (elbow).
- Mass split: wrist 75 g / gripper 110 g in the spec vs 14 g / 171 g here (roll servo case in the gripper);
  the sum matches (185 g).
