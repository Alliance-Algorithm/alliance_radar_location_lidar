# Field walls for GICP localization

Adds 4 boundary walls to a field map so GICP scan-to-map registration gets
strong x/y constraints.

## Why

Ground-only point clouds are near-symmetric in x/y, so GICP can stall at
half-correct translations. Vertical walls constrain both horizontal axes
strongly. Measured on the jinan field data:

| map | 0.3 m offset residual |
|---|---|
| bare `jinan_field_map_reg.pcd` | ~17 cm |
| `jinan_field_map_reg_walls.pcd` | ~1-2 cm |

## Use

```bash
python3 tools/field_walls/add_field_walls.py \
    --in model/generated/jinan_field_map_reg.pcd \
    --out model/jinan_field_map_reg_walls.pcd
```

`model/jinan_field_map_reg_walls.pcd` is the committed, ready-to-use map
(original 665424 points + 6048 wall points: 0.25 m layers to 1.75 m,
0.1 m horizontal spacing on the four edges at x=±14, y=±7.5).

Tune with `--dz/--zmax/--step`; the convergence result is insensitive to
the wall density once any reasonable wall exists.
