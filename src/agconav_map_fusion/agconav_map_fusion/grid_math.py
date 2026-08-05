"""Shared grid math for module E's 3-node pipeline (design.md 6-1~6-4 / 9-1~9-5).

map_merge_collector, elevation_map_merger, and merged_elevation_map_saver are
3 separate ROS2 nodes (processes), wired together only by topics (design.md
"노드 간 연결 방식"). Both map_merge_collector (to check cell-grid alignment
during validation) and elevation_map_merger (to actually build the merged
grid) need the exact same union-output-grid math and the exact same
grid_map_msgs/GridMap wire-format packing/unpacking, so it lives here once
instead of being copy-pasted into both node files and risking the two
copies drifting apart. This is an intra-package shared module, not a
cross-package import -- CONTRIBUTING 5's "모듈끼리는 토픽으로만 결합" rule is
about not reaching into a *different* ROS package's internals, not about
code reuse within one node's own package.

The grid_map_msgs/GridMap wire-format packing (axis flip, column-major
flatten) mirrors agconav_ground_mapping's ground_elevation_mapper --
reimplemented independently here, not imported, per CONTRIBUTING 5. It has
to match exactly or the 3 input maps and the merged output won't line up.
"""

from dataclasses import dataclass

from geometry_msgs.msg import Pose
from grid_map_msgs.msg import GridMap, GridMapInfo
import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

ROBOTS = ('drone', 'wheel', 'leg')
# design.md 6-4 / 9-5: applied in this order, low -> high priority, so the
# later (higher-priority) valid values win. Deliberately a fixed tuple, not a
# dict/set, so the merge order is always deterministic.
MERGE_ORDER = ('drone', 'leg', 'wheel')

# README 3.1 / 3.3: single global `map` frame, fixed 0.10 m/cell resolution.
EXPECTED_FRAME_ID = 'map'
EXPECTED_RESOLUTION = 0.10
ELEVATION_LAYER = 'elevation'


@dataclass(frozen=True)
class OutputGrid:
    """Union output grid spec (design.md 6-3 / 9-4): origin + size, no data.

    origin_x/origin_y are the world (map-frame) coordinates of the grid's
    min-x/min-y corner -- same convention agconav_ground_mapping's
    ground_elevation_mapper uses internally for its own accumulation grid
    (reimplemented independently here, not imported, per CONTRIBUTING 5).
    """

    origin_x: float
    origin_y: float
    resolution: float
    n_rows: int
    n_cols: int


def _map_origin(grid_map):
    """Return the (x, y) map-frame coords of grid_map's min-corner.

    grid_map_msgs/GridMap stores a center pose + length, not corners.
    """
    return (
        grid_map.info.pose.position.x - grid_map.info.length_x / 2.0,
        grid_map.info.pose.position.y - grid_map.info.length_y / 2.0,
    )


def map_bounds(grid_map):
    """Return (min_x, max_x, min_y, max_y) of grid_map in the map frame."""
    origin_x, origin_y = _map_origin(grid_map)
    return (
        origin_x, origin_x + grid_map.info.length_x,
        origin_y, origin_y + grid_map.info.length_y,
    )


def validate_maps(maps, robots=ROBOTS):
    """Check frame/resolution/layer for every robot in `maps` (design.md 6-2 / 9-3).

    `maps` is a {robot: GridMap-or-None} dict. Returns an error message
    string on the first failure found (checked in `robots` order for
    determinism), or None if every map passes. No resampling on mismatch --
    validation only ever accepts or rejects the maps as-is.
    """
    for robot in robots:
        grid_map = maps[robot]
        if grid_map is None:
            return f'{robot}: elevation_map was never received'
        if grid_map.header.frame_id != EXPECTED_FRAME_ID:
            return (
                f'{robot}: frame_id "{grid_map.header.frame_id}" != '
                f'"{EXPECTED_FRAME_ID}"')
        if not np.isclose(grid_map.info.resolution, EXPECTED_RESOLUTION, atol=1e-6):
            return (
                f'{robot}: resolution {grid_map.info.resolution} != '
                f'{EXPECTED_RESOLUTION} m/cell')
        if ELEVATION_LAYER not in grid_map.layers:
            return f'{robot}: missing required layer "{ELEVATION_LAYER}"'
    return None


def build_output_grid(maps, robots=ROBOTS, resolution=EXPECTED_RESOLUTION, max_grid_cells=None):
    """Union bounding box of `maps` -> OutputGrid (design.md 6-3 / 9-4).

    Returns (OutputGrid, None) on success, or (None, error message) if the
    resulting grid would exceed max_grid_cells (skipped when None).
    """
    bounds = [map_bounds(maps[robot]) for robot in robots]
    min_x = min(b[0] for b in bounds)
    max_x = max(b[1] for b in bounds)
    min_y = min(b[2] for b in bounds)
    max_y = max(b[3] for b in bounds)

    n_rows = round((max_x - min_x) / resolution)
    n_cols = round((max_y - min_y) / resolution)
    total_cells = n_rows * n_cols

    if max_grid_cells is not None and total_cells > max_grid_cells:
        return None, (
            f'output grid would need {n_rows}x{n_cols}={total_cells} cells, '
            f'over the max_grid_cells limit ({max_grid_cells})')

    return OutputGrid(
        origin_x=min_x, origin_y=min_y,
        resolution=resolution, n_rows=n_rows, n_cols=n_cols,
    ), None


def check_grid_alignment(maps, output_grid, robots=ROBOTS, tolerance=1e-6):
    """Check every input map's origin lands exactly on the output grid's cells.

    frame_id and resolution matching (validate_maps) is not enough: if an
    input map's origin isn't an integer number of cells away from the
    output grid's origin, that map's cells straddle the output grid's cell
    boundaries instead of coinciding with them, and naively copying its
    values in would silently misalign the merged map. Returns an error
    message string on the first misaligned map found (`robots` order), or
    None if every map's origin aligns on both axes.
    """
    for robot in robots:
        origin_x, origin_y = _map_origin(maps[robot])
        for axis, origin, output_origin in (
            ('x', origin_x, output_grid.origin_x),
            ('y', origin_y, output_grid.origin_y),
        ):
            offset_cells = (origin - output_origin) / output_grid.resolution
            if abs(round(offset_cells) - offset_cells) >= tolerance:
                return (
                    f'{robot}: origin_{axis}={origin:.6f} is not aligned to the output '
                    f'grid (origin_{axis}={output_origin:.6f}, resolution='
                    f'{output_grid.resolution}) -- offset is {offset_cells:.6f} cells, '
                    f'not an integer')
    return None


def extract_elevation(grid_map):
    """Undo ground_elevation_mapper-style wire packing for one layer.

    Recovers (elevation, origin_x, origin_y) in the same "min-corner origin,
    row grows +x, col grows +y" convention the source node built it in, from
    the packed grid_map_msgs wire format (matrix index (0, 0) at the (+x,
    +y) corner, column-major flattened -- see agconav_ground_mapping's
    ground_elevation_mapper for the packing this undoes). Assumes
    outer_start_index/inner_start_index == 0, true for every elevation_map
    this project publishes.
    """
    layer_index = grid_map.layers.index(ELEVATION_LAYER)
    layer = grid_map.data[layer_index]
    n_rows = layer.layout.dim[0].size
    n_cols = layer.layout.dim[1].size
    gm_matrix = np.asarray(layer.data, dtype=np.float32).reshape((n_rows, n_cols), order='F')
    elevation = gm_matrix[::-1, ::-1]
    origin_x, origin_y = _map_origin(grid_map)
    return elevation, origin_x, origin_y


def build_merged_grid_map_message(elevation, output_grid, stamp):
    """Pack the merged elevation array into a grid_map_msgs/GridMap.

    Mirrors ground_elevation_mapper's _build_grid_map_message packing (axis
    flip + column-major flatten), reimplemented independently here per
    CONTRIBUTING 5 -- see that node for the convention this matches.
    """
    n_rows, n_cols = elevation.shape
    length_x = n_rows * output_grid.resolution
    length_y = n_cols * output_grid.resolution

    gm_matrix = elevation[::-1, ::-1]
    elevation_layer = Float32MultiArray()
    elevation_layer.layout.dim = [
        MultiArrayDimension(label='column_index', size=n_rows, stride=n_rows * n_cols),
        MultiArrayDimension(label='row_index', size=n_cols, stride=n_rows),
    ]
    elevation_layer.data = gm_matrix.flatten(order='F').tolist()

    info = GridMapInfo()
    info.resolution = output_grid.resolution
    info.length_x = length_x
    info.length_y = length_y
    info.pose = Pose()
    info.pose.position.x = output_grid.origin_x + length_x / 2.0
    info.pose.position.y = output_grid.origin_y + length_y / 2.0
    info.pose.position.z = 0.0
    info.pose.orientation.w = 1.0

    grid_map = GridMap()
    grid_map.header.stamp = stamp
    grid_map.header.frame_id = EXPECTED_FRAME_ID
    grid_map.info = info
    grid_map.layers = [ELEVATION_LAYER]
    grid_map.basic_layers = [ELEVATION_LAYER]
    grid_map.data = [elevation_layer]
    grid_map.outer_start_index = 0
    grid_map.inner_start_index = 0
    return grid_map
