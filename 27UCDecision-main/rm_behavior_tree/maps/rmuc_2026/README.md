# RMUC 2026 Nav2 map

This directory is reserved for the map measured by the team on its actual field.
Do not convert the official rendering or STEP model directly into a Nav2 occupancy
grid: wall thickness, field tolerances, carpet seams and the SLAM origin differ in
practice.

Place the generated occupancy files here, for example:

```text
rmuc_2026.pgm
rmuc_2026.yaml
```

After loading the map in Nav2:

1. Define the official field coordinate origin at the lower-left corner of the
   official top view, with +x from red to blue and +y from bottom to top.
2. Measure at least three non-collinear landmarks in both field and `map` frames.
3. Fit the planar transform and write its `x`, `y` and `yaw` to
   `config/rmuc_2026_field.yaml` under `field_to_map`.
4. Adjust every active waypoint on the occupancy map and validate its footprint
   clearance in RViz.
5. Set `field_map.calibrated` to `true` only after red- and blue-side dry runs.

The repository intentionally does not ship a fabricated `.pgm`: navigation must
use the map built from the team's lidar/camera and the physical practice field.

Official references:

- Rules/resources hub: <https://bbs.robomaster.com/wiki/20204847/809871>
- RMU 2026 official field STEP files: <https://bbs.robomaster.com/article/814728>

The official STEP file is a geometry reference, not a navigation map. If it
conflicts with the current rulebook or the event field, follow the rulebook and
the event field.
