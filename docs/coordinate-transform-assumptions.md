# Coordinate transform assumptions for `geojson-exporter`

`geojson-exporter` maps defect coordinates from a local mission frame to the coordinate reference used by the WebODM reconstruction output.

## Assumptions

1. The local mission frame is right-handed and measured in meters.
2. Defect `z` values are already in reconstruction vertical units and are passed through unchanged.
3. XY mapping is modeled as a deterministic 2D affine transform (scale, rotation, translation).
4. Rotation is counter-clockwise around +Z.
5. Transform values are recorded in a checked-in JSON file so runs are reproducible.

## Reproducible mapping config

Use `services/geojson-exporter/config/mapping.example.json` as the baseline template and commit mission/site specific copies beside artifacts when calibration is updated.

## Formula

Given local `(x_l, y_l)` and mapping config values:

- `x_s = x_l * scale_x`
- `y_s = y_l * scale_y`
- `x_r = x_s*cos(theta) - y_s*sin(theta) + translation_x`
- `y_r = x_s*sin(theta) + y_s*cos(theta) + translation_y`

where `theta = rotation_deg * pi/180`.
