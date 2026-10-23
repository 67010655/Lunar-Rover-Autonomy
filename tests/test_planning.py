import heapq
import math
import random
import unittest

from lunar_rover.grid import GridMap
from lunar_rover.planning import NoPathError, plan_p0


def reference_dijkstra(grid, start, goal):
    """Independent small-map oracle for a point body, no corner cutting."""
    queue, best = [(0.0,start)], {start:0.0}
    while queue:
        cost,cell = heapq.heappop(queue)
        if cell == goal:
            return cost
        if cost > best[cell]:
            continue
        r,c = cell
        for dr in (-1,0,1):
            for dc in (-1,0,1):
                n = (r+dr,c+dc)
                if (not dr and not dc) or not grid.contains(n) or n in grid.blocked:
                    continue
                if dr and dc and ((r+dr,c) in grid.blocked or (r,c+dc) in grid.blocked):
                    continue
                value = cost+math.hypot(dr,dc)*grid.resolution_m
                if value < best.get(n,math.inf):
                    best[n] = value
                    heapq.heappush(queue,(value,n))
    return math.inf


class PlanningTests(unittest.TestCase):
    def test_metric_diagonal_and_stationary_route(self):
        grid = GridMap(6,6,0.25)
        self.assertAlmostEqual(plan_p0(grid,(0,0),(4,4)).length_m,math.sqrt(2))
        self.assertEqual(plan_p0(grid,(2,2),(2,2)).cells,((2,2),))

    def test_blocked_outside_and_disconnected_endpoints(self):
        grid = GridMap(5,5,1.0,frozenset((r,2) for r in range(5)))
        for start,goal in (((1,2),(4,4)),((0,0),(5,5)),((0,0),(4,4))):
            with self.subTest(start=start,goal=goal), self.assertRaises(NoPathError):
                plan_p0(grid,start,goal)

    def test_cannot_cut_diagonal_corner(self):
        grid = GridMap(2,2,1.0,frozenset({(0,1),(1,0)}))
        with self.assertRaises(NoPathError):
            plan_p0(grid,(0,0),(1,1))

    def test_body_clearance_blocks_narrow_passage(self):
        blocked = frozenset((r,3) for r in range(7) if r != 3)
        grid = GridMap(7,7,1.0,blocked)
        self.assertGreater(plan_p0(grid,(3,1),(3,5),0.4).length_m,0)
        with self.assertRaises(NoPathError):
            plan_p0(grid,(3,1),(3,5),0.51)

    def test_astar_matches_independent_dijkstra_on_seeded_maps(self):
        rng = random.Random(2026)
        for _ in range(20):
            blocked = frozenset((r,c) for r in range(7) for c in range(8)
                                if (r,c) not in ((0,0),(6,7)) and rng.random()<0.23)
            grid = GridMap(8,7,0.4,blocked)
            expected = reference_dijkstra(grid,(0,0),(6,7))
            if math.isinf(expected):
                with self.assertRaises(NoPathError):
                    plan_p0(grid,(0,0),(6,7))
            else:
                plan = plan_p0(grid,(0,0),(6,7))
                self.assertAlmostEqual(plan.length_m,expected)
                self.assertEqual(plan,plan_p0(grid,(0,0),(6,7)))

    def test_swept_circle_detects_obstacle_between_endpoints(self):
        grid = GridMap(6,5,1.0,frozenset({(2,2)}))
        self.assertFalse(grid.segment_is_free((1.5,2.5),(4.5,2.5),0.1))
        self.assertFalse(grid.segment_is_free((1.5,1.8),(3.5,1.8),0.21))
        self.assertTrue(grid.segment_is_free((1.5,1.5),(3.5,1.5),0.2))

    def test_nonzero_origin_and_invalid_radius(self):
        grid = GridMap(3,3,0.5,origin_xy_m=(-4.0,8.0))
        self.assertEqual(grid.to_world((1,2)),(-2.75,8.75))
        with self.assertRaises(ValueError):
            grid.segment_is_free((0,0),(1,1),-0.1)
