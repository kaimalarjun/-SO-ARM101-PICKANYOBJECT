# Wrist-camera grasp alignment

Use `scripts/grasp_alignment.py` to check a selected object against the open
fingers before approaching. It reads images only and never commands the arm.

1. Raise clear of the object. Select its complete silhouette in a binary mask:
   white for the object, black elsewhere. Exclude fingers and neighbouring objects.
2. Save a local JSON profile with `jaw_tips_px`: two visible fingertip positions
   in image pixels. Use the current gripper opening; the moving tip changes when
   the gripper opens or closes. Keep profiles and images outside Git.
3. Run `python3 scripts/grasp_alignment.py FRAME MASK PROFILE --overlay OUTPUT`.
   The overlay marks the jaw gap green and the object's middle and long axis magenta.
   Align across the narrow dimension and center the body, rather than its end.
4. To suggest a correction, measure two independent small arm movements while
   clear using `measure_visual_step.py`. Add `local_response` with `joints` and
   `pixels_per_radian` (two rows: horizontal and vertical image shift; one column
   per measured joint). Suggestions are limited to 0.08 radians and still require
   gateway limits and an independent clearance check. Re-measure after substantial
   pose/rotation changes or contact. Do not apply a response from another scene.
5. Image alignment does **not** establish depth. Inspect the external view for
   both tips reaching opposite sides below the top surface while clearing the mat.
   Descend incrementally, adjusting shoulder and wrist together when needed to
   preserve approach angle. Re-localize if contact rolls or pushes the object.
6. Close to contact, then make a small lift. Confirm the object leaves its support
   and remains fixed between the fingers in both views before transporting it.

The checker rejects clipped or multiple silhouettes. Round silhouettes have no
reliable long axis. It always reports `grasp_ready: false`: a two-dimensional
check cannot authorize depth or prove a grip. This is an alignment aid, not
automatic object segmentation or calibrated three-dimensional perception.
