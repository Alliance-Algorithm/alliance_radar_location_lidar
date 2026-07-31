# Global Grid Map Tool Design

## Goal

Provide an offline CLI tool in the `radar_lidar` package that converts a registered
global point cloud map (e.g. `model/generated/jinan_field_map_reg.pcd`, 28.2 m x 15.2 m,
665 k points) into a `nav_msgs`-compatible grid map saved as PGM + YAML
(`map_server` standard format), following the height-table obstacle idea from
`rmcs-local-map` but simplified for dense static maps.

Use cases: navigation / path planning / relocalization input, debugging and
visualization. The tool is offline-only; it does not subscribe to live LiDAR.

## Selected Architecture

Single CLI tool with algorithm split from the entry point, mirroring the existing
`registration_tool.cpp` / `tools/radar_lidar/offline_registration.*` pattern:

```text
radar_lidar/
├── tools/
│   ├── grid_map_tool.cpp            # CLI entry: arg parsing + I/O
│   └── radar_lidar/
│       ├── grid_map.hpp             # GridMapParams / GridMapResult / rasterize()
│       └── grid_map.cpp             # implementation
├── test/
│   └── test_grid_map.cpp            # gtest
└── CMakeLists.txt                   # grid_map.cpp -> offline_lib; grid_map_tool target
```

Algorithm code is ROS-free C++23, compiled into `radar_lidar_offline_lib`, so it is
unit-testable and potentially reusable later for online local grid maps.

### Core API

```cpp
struct GridMapParams {
    double resolution       = 0.05;   // meters per cell
    double height_threshold = 0.3;    // z_max - z_min > threshold -> obstacle
    int    min_points       = 3;      // minimum points per cell to consider
    int    dilate           = 0;      // obstacle dilation radius in cells (0 = off)
};

struct GridMapResult {
    int width, height;
    double origin_x, origin_y;        // world coordinates of the lower-left corner
    std::vector<int8_t> data;         // 0 = obstacle, 100 = free, -1 = unknown
};

auto rasterize(const pcl::PointCloud<pcl::PointXYZ>& cloud,
               const GridMapParams& params,
               std::optional<Bounds> bounds) -> std::expected<GridMapResult, std::string>;
```

## Algorithm

```
load PCD
  -> compute bounds (point cloud bbox rounded to resolution multiples, or --bounds)
  -> flat per-cell accumulators {point_count, z_min, z_max} (array of size w*h)
  -> iterate points: (x, y) -> cell index, update accumulators
  -> classify:
        count >= min_points && z_max - z_min > height_threshold  -> obstacle (0)
        count >= 1                                               -> free (100)
        otherwise                                                -> unknown (-1)
  -> optional circular dilation of obstacles (radius = dilate cells)
  -> write PGM (P5 binary) + YAML
```

The height-range test is the key obstacle criterion: ground cells have small
z-spread, wall/cover cells have large z-spread. `min_points` rejects isolated
outlier points.

## Coordinate Convention (map_server)

- PGM row 0 = largest world y (north up); column 0 = smallest world x.
- YAML `origin: [x_min, y_min, 0]` with `negate: 0`, `occupied_thresh: 0.65`,
  `free_thresh: 0.196`.
- PGM writes iterate world y from max to min and world x from min to max so the
  loaded map aligns exactly with the source PCD.
- PGM grayscale: 0 = occupied, 205 = unknown, 254 = free.

## CLI Interface

```
grid_map_tool <map.pcd> --output <prefix> [options]

  --output <path>           output prefix -> <prefix>.pgm + <prefix>.yaml (default map)
  --resolution <float>      cell size in meters (default 0.05)
  --height-threshold <f>    z-spread obstacle threshold in meters (default 0.3)
  --min-points <int>        minimum points per cell (default 3)
  --dilate <int>            obstacle dilation radius in cells (default 0)
  --bounds xmin ymin xmax ymax   explicit map bounds (default: cloud bbox)
  --verbose                 print stats: bounds, obstacle/free/unknown cell counts, time
```

## Error Handling

- PCD load failure / empty cloud -> `std::expected` error string, exit code 1.
- Invalid args (resolution <= 0, min-points < 1, reversed bounds) -> usage + exit 1.
- Degenerate bounds (zero width or height) -> error.
- Output write failure -> error + exit 1.
- Performance: single O(N) pass over points (66 万 points, ~ms); dilation is
  O(obstacles x radius^2), acceptable.

## Testing

gtest in `test_grid_map.cpp` (aligned with existing `test_offline_registration.cpp` style):

1. Synthetic "ground + wall" cloud -> wall cells obstacle, ground cells free,
   empty corners unknown.
2. Height threshold boundary: wall 0.29 m -> free, wall 0.31 m -> obstacle.
3. `min_points` filtering: isolated outlier does not become an obstacle.
4. Dilation: cells within dilate radius around an obstacle become obstacle.
5. Coordinate mapping: known world point maps to expected pixel (y-flip verified).
6. YAML serialization: origin/resolution fields correct.

## Verification

1. `colcon build --packages-select radar_lidar` + run gtest.
2. Run on `model/generated/jinan_field_map_reg.pcd`, convert PGM to PNG with
   Python (PIL/matplotlib), manually inspect arena outline: walls/covers clear,
   ground clean.
3. Archive generated `jinan_field_map_reg.pgm` + `.yaml` into `model/generated/`.
