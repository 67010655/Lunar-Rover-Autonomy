"""P0: deterministic shortest-distance A* on an eight-connected grid."""

from dataclasses import dataclass
import heapq
import itertools
import math

from .grid import Cell, GridMap


class NoPathError(ValueError):
    """Start/goal is blocked or there is no collision-free route."""


@dataclass(frozen=True)
class Plan:
    cells: tuple[Cell, ...]
    length_m: float
    expanded_nodes: int


def plan_p0(grid: GridMap, start: Cell, goal: Cell, clearance_m: float = 0.0) -> Plan:
    free = grid.free_cells(clearance_m)
    if start not in free or goal not in free:
        raise NoPathError("Start or goal is blocked, out of bounds, or lacks clearance")
    counter = itertools.count()
    cost = {start: 0.0}
    previous: dict[Cell, Cell] = {}
    queue = [(math.dist(start, goal)*grid.resolution_m, next(counter), 0.0, start)]
    expanded = 0
    while queue:
        _, _, distance, current = heapq.heappop(queue)
        if distance > cost[current] + 1e-12:
            continue
        expanded += 1
        if current == goal:
            path = [current]
            while path[-1] != start:
                path.append(previous[path[-1]])
            return Plan(tuple(reversed(path)), distance, expanded)
        row, col = current
        for dr, dc in ((-1,0), (0,-1), (0,1), (1,0), (-1,-1), (-1,1), (1,-1), (1,1)):
            neighbor = (row+dr, col+dc)
            if neighbor not in free:
                continue
            if dr and dc and ((row+dr,col) not in free or (row,col+dc) not in free):
                continue
            if not grid.segment_is_free(grid.to_world(current), grid.to_world(neighbor), clearance_m):
                continue
            candidate = distance + math.hypot(dr,dc)*grid.resolution_m
            if candidate < cost.get(neighbor, math.inf) - 1e-12:
                previous[neighbor], cost[neighbor] = current, candidate
                heuristic = math.dist(neighbor, goal)*grid.resolution_m
                heapq.heappush(queue, (candidate+heuristic, next(counter), candidate, neighbor))
    raise NoPathError("No route connects start and goal")
