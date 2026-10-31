# D405 protected wrist mount R2

Print **D405_wrist_mount_R2_PRINT.stl**, one copy, millimetres, 100% scale.
It is already oriented with the flat camera rear plate on the bed and the
camera opening pointing upward. Size: **49.6 × 82.05 × 31.0 mm**.

Suggested starting settings for a 0.4 mm nozzle:

- PETG, 0.20 mm layers, 6 walls, 6 top/bottom layers, 40% gyroid infill.
- Supports enabled **everywhere** for the mounting arm and overhanging
  gripper seat. The two continuous guard rails have no window bridges. Use your PETG profile's removable support
  interface; do not limit supports to the build plate.
- A 5 mm brim is useful for adhesion. Keep the supplied orientation.
- Clear all supports, the two camera bores, the original recessed mounting
  holes and the cable-tie slots before assembly. Smooth the cable contact edges.

## Hardware and assembly

1. Reuse the existing **two gripper mounting screws**. Their 12 mm centre
   spacing, 4.5 mm clearance bores, 8 mm recesses and seating geometry are
   retained from the confirmed printed mount. The component is on the
   **screw-head side of the gripper**, facing the latest TPU fingers.
2. Slide the D405 through the open front, with its two lenses horizontally
   across the jaws. The 43.6 mm cavity gives 0.8 mm nominal clearance per side.
   Both side centres are open so the USB housing can pass during insertion.
3. Secure the camera from the rear with **two M3×8 socket-head screws and
   0.5 mm plain washers**. The 5 mm plate leaves about **2.5 mm** in the camera.
   RealSense specifies a maximum M3 insertion of **4 mm**. M3×10 is too long
   for this stack. Tighten gently by hand; no powered driver or printed torque
   value is specified.
4. Plug in the USB cable. Use 2.5 mm cable ties through the paired side-rail
   slots to secure the cable jacket behind the camera, leaving a relaxed bend
   at the plug. The clearance check used a straight 20 × 10 mm plug envelope
   projecting 40 mm from either side. Check a different/right-angle plug on
   the hardware.
5. Check seating, camera retention and cable clearance, then perform a slow,
   unloaded gripper opening/closing check before collecting demonstrations.

## What changed

R2 simplifies the windowed R1 cage into two continuous protective rails with
rounded exterior corners, projecting 3 mm beyond the nominal camera front.
The two large guard windows and four small front corner returns are removed.
The 5 mm camera plate, two 5 mm support ribs, rear ventilation openings and
functional cable-tie slots remain. Both sides remain open for USB access.

The outer dimensions and optical pose are unchanged from R1. This adds about
2.6 g of PETG at solid density, eliminates the guard's window bridges and avoids
the open-ended corner tabs. The guard is intended for incidental broad contact,
not as a sealed enclosure or validated crash protection.

Only the mating seat from the old print is carried over. The frame is 49.6 mm
wide; its maximum outward coordinate is 99.48 mm in the vendor gripper frame,
approximately the old print's outward envelope. Camera front centre is
(0, 70, 64) mm in that frame, with the view directed forward and inward.

## Mechanical assessment

The main arrangement is straightforward: camera screws into a rigid plate,
two ribs connect that plate to the gripper seat, and two solid rails protect
the camera edges. There are no adjustment joints, snap fits or extra guard
fasteners. The assembly has one printed part and four screws in total.

The solid rails remove a flexible span present in the old windowed guard.
This is a structural improvement in layout, not a measured strength rating.
The original screw seat still has 1.9 mm of plastic beneath each recessed
screw head, and several internal transitions remain abrupt. Avoid excessive
screw tightening. Check for rocking at the seat and camera movement under
modest handling and robot motion before relying on it for data collection.
Neither stiffness, impact survival nor long-term creep has been physically
tested. R2 is the current print prototype.

## Camera mode and policy images

The design targets **848×480 depth capture**, with the full wrist image resized
for a **224 or 256 pixel policy input**. The physical view should be retained:
aspect-ratio-preserving resize and padding is one option. A centre square crop
can remove an open jaw, so review any policy's crop/augmentation pipeline.
Resizing a captured image does not change the depth sensor's near limit.

RealSense's published minimum depth is **70 mm at 848×480** and **100 mm at
1280×720**. The recommended working range extends to 500 mm; that is not a
hard far clipping plane. These stereo depth limits do not define RGB visibility.

Both lens origins were checked with an 18 mm baseline and the published depth
origin 3.7 mm behind the front glass. Results for this R2:

| Geometric check | Result |
| --- | --- |
| Entire TPU geometry, closed CAD | 88–160 mm axial depth |
| Exposed TPU, closed CAD | 115–160 mm axial depth |
| Exposed TPU opening/bending envelope | **97–165 mm** axial depth |
| Envelope inside both lens views | Pass, even using a narrower 82° × 56° view |
| Carrier intruding into either expanded 94° × 68° lens cone | Zero solid overlap |

The envelope samples each jaw outward by 0/20/42.5 mm, retracted by 0/15 mm,
and independently displaced ±10 mm in X and Z. This is a geometric stress
envelope, **not solved Robotiq kinematics or simulated TPU deformation**.
The final design deliberately does not require every sampled point to stay
beyond the 720p depth limit.

At 224 pixels across the full image, a 43.5 mm exposed-rail reference segment
projects to roughly 17 pixels. Small deformations can therefore be below one
pixel. The wrist image supplies contact context; this geometry check does not
establish force sensing or policy performance. No marker visibility constraint
was used.

## Checks and remaining physical verification

- Exported STL is watertight, consistently wound and one connected component.
- BRep is valid; the original mounting seat is retained with zero missing volume.
- No overlap with the nominal camera, USB housing/insertion sweep, straight
  plug envelopes or the checked screwdriver access cylinders.
- Fusion reports zero collisions against the 33 visible bodies of the closed
  Robotiq assembly and latest V2.2 adapter / TPU finger reference.
- Actual camera intrinsics, stereo fill rate, motion blur, arbitrary grasp
  occlusion, full articulated stroke, cable bending, stiffness and impact
  resistance have not been physically tested. This is a printable prototype.

The older printed design had angle/framing work and a farther camera position.
The near-range concern raised during this revision arose from the new closer
draft; it did not demonstrate a depth-range fault in the old printed mount.

## Files

- `D405_wrist_mount_R2_PRINT.stl`: print this one part.
- `D405_wrist_mount_R2.step`: editable solid in the gripper assembly frame.
- `D405_R2_mount.f3d`: local Fusion archive of the mount.
- `D405_R2_mount.png`: clear standalone view of the printable part.
- `D405_R2_assembly.png`: placement with the latest fingers. Camera shown as a
  dimensioned envelope, not detailed vendor CAD.
- `D405_R2_optical_views.png`: synthetic lens projections; lower row is the
  translated opening envelope, not an articulated assembly.
- `policy_256_*.png`: the same geometric projections at 256-square input size.
- `mechanical_checks.json`, `optical_checks.json`, `fusion_closed_check.json`:
  numeric check results.
- `build_mount.py` and `reference/preserved_two_screw_seat.step`: reproduce the
  solid and mechanical checks with CadQuery, NumPy, trimesh and Shapely.
- `requirements.txt`: versions used to rebuild this release with Python 3.10.

## Rebuilding the mount

In a Python 3.10 environment, run from this directory:

```sh
python -m pip install -r requirements.txt
python build_mount.py
```

The generator reads the included mounting-seat STEP and writes the mount STEP,
print STL, camera envelope, preview mesh and mechanical check results beside
the script. Rebuild a copy of the directory to preserve the release files and
their checksums. `SHA256.json` identifies the supplied release files; STEP
export timestamps can change when rebuilding.

The optical and Fusion JSON files record checks from the design session.
Rebuilding the solid does not rerun those checks or regenerate the Fusion
archive and rendered images.

## Source dimensions

[RealSense D400 datasheet, July 2023](https://www.realsenseai.com/wp-content/uploads/2023/07/Intel-RealSense-D400-Series-Datasheet-July-2023.pdf),
Tables 4-11, 4-16, 4-20 and Figure 10-13 (pages 89, 94, 97, 150).
[D405 product specifications](https://www.realsenseai.com/products/stereo-depth-camera-d405/)
give the nominal 87° × 58° field of view and recommended 7–50 cm range.

