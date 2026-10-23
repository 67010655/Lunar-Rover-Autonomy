"""Metric grid and swept-circle collision checks in a local Cartesian frame.

Cells are (row, column), with row increasing along +y. Coordinates are cell
centres; obstacle cells are closed squares. Touching an obstacle is a collision.
"""

from dataclasses import dataclass
import math

Cell = tuple[int, int]
Point = tuple[float, float]


def _point_segment_distance(p: Point, a: Point, b: Point) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    norm = dx * dx + dy * dy
    t = 0.0 if norm == 0 else max(0.0, min(1.0, ((p[0]-a[0])*dx + (p[1]-a[1])*dy)/norm))
    return math.hypot(p[0] - a[0] - t*dx, p[1] - a[1] - t*dy)


def _segment_box_distance(a: Point, b: Point, bounds: tuple[float, float, float, float]) -> float:
    xmin, ymin, xmax, ymax = bounds
    # Slab intersection, including contact and zero-length segments.
    lo, hi = 0.0, 1.0
    for start, delta, lower, upper in ((a[0], b[0]-a[0], xmin, xmax), (a[1], b[1]-a[1], ymin, ymax)):
        if abs(delta) < 1e-15:
            if start < lower or start > upper:
                lo, hi = 1.0, 0.0
                break
        else:
            t0, t1 = sorted(((lower-start)/delta, (upper-start)/delta))
            lo, hi = max(lo, t0), min(hi, t1)
    if lo <= hi:
        return 0.0
    endpoint_distance = min(math.hypot(max(xmin-x, 0.0, x-xmax), max(ymin-y, 0.0, y-ymax)) for x, y in (a, b))
    return min(endpoint_distance, *(_point_segment_distance(p, a, b) for p in ((xmin,ymin), (xmin,ymax), (xmax,ymin), (xmax,ymax))))


@dataclass(frozen=True)
class GridMap:
    width: int
    height: int
    resolution_m: float
    blocked: frozenset[Cell] = frozenset()
    origin_xy_m: Point = (0.0, 0.0)

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0 or not math.isfinite(self.resolution_m) or self.resolution_m <= 0:
            raise ValueError("Grid dimensions and resolution must be positive")
        if not all(math.isfinite(v) for v in self.origin_xy_m):
            raise ValueError("Map origin must be finite")
        if any(not self.contains(cell) for cell in self.blocked):
            raise ValueError("Obstacle cell lies outside the grid")

    def contains(self, cell: Cell) -> bool:
        return 0 <= cell[0] < self.height and 0 <= cell[1] < self.width

    def to_world(self, cell: Cell) -> Point:
        if not self.contains(cell):
            raise ValueError(f"Cell outside grid: {cell}")
        row, col = cell
        ox, oy = self.origin_xy_m
        return ox + (col + 0.5)*self.resolution_m, oy + (row + 0.5)*self.resolution_m

    def segment_is_free(self, a: Point, b: Point, radius_m: float = 0.0) -> bool:
        if radius_m < 0 or not all(math.isfinite(v) for v in (*a, *b, radius_m)):
            raise ValueError("Collision-check inputs must be finite, radius non-negative")
        ox, oy = self.origin_xy_m
        xmax, ymax = ox + self.width*self.resolution_m, oy + self.height*self.resolution_m
        if any(x-radius_m < ox or y-radius_m < oy or x+radius_m >= xmax or y+radius_m >= ymax for x, y in (a, b)):
            return False
        # Check only the obstacle cells near the swept segment's bounding box.
        r0 = max(0, math.floor((min(a[1],b[1])-radius_m-oy)/self.resolution_m)-1)
        r1 = min(self.height-1, math.floor((max(a[1],b[1])+radius_m-oy)/self.resolution_m)+1)
        c0 = max(0, math.floor((min(a[0],b[0])-radius_m-ox)/self.resolution_m)-1)
        c1 = min(self.width-1, math.floor((max(a[0],b[0])+radius_m-ox)/self.resolution_m)+1)
        for row in range(r0, r1+1):
            for col in range(c0, c1+1):
                if (row, col) in self.blocked:
                    x, y = ox + col*self.resolution_m, oy + row*self.resolution_m
                    if _segment_box_distance(a, b, (x, y, x+self.resolution_m, y+self.resolution_m)) <= radius_m + 1e-12:
                        return False
        return True

    def free_cells(self, radius_m: float) -> frozenset[Cell]:
        return frozenset((r,c) for r in range(self.height) for c in range(self.width)
                         if self.segment_is_free(self.to_world((r,c)), self.to_world((r,c)), radius_m))
