"""Validated, versioned JSON inputs; all physical quantities use SI units."""

from dataclasses import dataclass
import json
import math
from pathlib import Path

from .grid import Cell, GridMap


@dataclass(frozen=True)
class RobotConfig:
    body_radius_m: float
    planning_margin_m: float
    max_speed_m_s: float
    max_yaw_rate_rad_s: float


@dataclass(frozen=True)
class SimulationConfig:
    dt_s: float
    max_time_s: float
    waypoint_tolerance_m: float
    estimated_goal_tolerance_m: float
    true_goal_tolerance_m: float


@dataclass(frozen=True)
class LocalizationConfig:
    speed_scale_bias: float
    yaw_rate_bias_rad_s: float
    speed_noise_std_m_s: float
    yaw_rate_noise_std_rad_s: float
    absolute_fix_period_s: float
    absolute_fix_noise_std_m: float
    absolute_fix_gain: float


@dataclass(frozen=True)
class Scenario:
    scenario_id: str
    grid: GridMap
    start: Cell
    goal: Cell
    robot: RobotConfig
    simulation: SimulationConfig
    localization: LocalizationConfig
    raw: dict


def _integer(value: object, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{label} must be an integer")
    return value


def scenario_from_dict(raw: dict) -> Scenario:
    try:
        if raw['schema_version'] != 1 or raw['data_type'] != 'synthetic' or raw['coordinate_frame'] != 'map_local_m':
            raise ValueError("This milestone supports schema 1 synthetic map_local_m scenarios only")
        if not isinstance(raw['scenario_id'], str) or not raw['scenario_id'].strip():
            raise ValueError("scenario_id must be a non-empty string")
        spec = raw['grid']
        width, height = _integer(spec['width_cells'], 'width'), _integer(spec['height_cells'], 'height')
        blocked = set()
        for rect in spec['obstacle_rectangles_rc']:
            if len(rect) != 4:
                raise ValueError("Obstacle rectangles require r0,c0,r1,c1 inclusive")
            r0,c0,r1,c1 = (_integer(v, 'obstacle coordinate') for v in rect)
            if not (0 <= r0 <= r1 < height and 0 <= c0 <= c1 < width):
                raise ValueError("Invalid obstacle rectangle")
            blocked.update((r,c) for r in range(r0,r1+1) for c in range(c0,c1+1))
        origin = tuple(spec['origin_xy_m'])
        if len(origin) != 2:
            raise ValueError("origin_xy_m requires two coordinates")
        grid = GridMap(width, height, spec['resolution_m'], frozenset(blocked), origin)
        cells = []
        for label in ('start_rc', 'goal_rc'):
            cell = tuple(_integer(v, label) for v in raw[label])
            if len(cell) != 2 or not grid.contains(cell):
                raise ValueError(f"Invalid {label}")
            cells.append(cell)
        robot = RobotConfig(**raw['robot'])
        sim = SimulationConfig(**raw['simulation'])
        loc = LocalizationConfig(**raw['localization'])
        for group in (robot, sim, loc):
            if not all(isinstance(v, (int,float)) and not isinstance(v, bool) and math.isfinite(v) for v in vars(group).values()):
                raise ValueError("All physical parameters must be finite numbers")
        if min(robot.body_radius_m, robot.max_speed_m_s, robot.max_yaw_rate_rad_s) <= 0 or robot.planning_margin_m < 0:
            raise ValueError("Robot sizes/speeds must be positive; margin non-negative")
        if min(vars(sim).values()) <= 0 or sim.dt_s > sim.max_time_s:
            raise ValueError("Simulation durations/tolerances must be positive, dt <= max_time")
        if min(loc.speed_noise_std_m_s, loc.yaw_rate_noise_std_rad_s, loc.absolute_fix_noise_std_m) < 0:
            raise ValueError("Noise standard deviations must be non-negative")
        if loc.absolute_fix_period_s <= 0 or not 0 < loc.absolute_fix_gain <= 1 or loc.speed_scale_bias <= -1:
            raise ValueError("Invalid fix period/gain or speed scale")
        return Scenario(raw['scenario_id'], grid, cells[0], cells[1], robot, sim, loc, raw)
    except (KeyError, TypeError) as exc:
        raise ValueError(f"Invalid scenario structure: {exc}") from exc


def load_scenario(path: str | Path) -> Scenario:
    return scenario_from_dict(json.loads(Path(path).read_text(encoding='utf-8')))
