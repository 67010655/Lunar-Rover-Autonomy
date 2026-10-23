"""Closed-loop planar prototype. The controller never receives ground truth.

The body model is an abstract (v, yaw-rate) plant, not 6WD/4WS wheel kinematics.
Odometry is generated from commanded motion with configurable noise/bias; fixes
are noisy synthetic absolute positions. Neither estimator mode implements SLAM.
"""

from dataclasses import dataclass, asdict
import math
import random

from .grid import Point
from .planning import Plan
from .scenario import LocalizationConfig, RobotConfig, Scenario, SimulationConfig


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2*math.pi) - math.pi


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float

    @property
    def xy(self) -> Point:
        return self.x, self.y


def integrate(pose: Pose, v: float, w: float, dt: float) -> Pose:
    # Mid-angle/sinc form avoids cancellation when yaw rate is almost zero.
    half_turn = w*dt/2
    sinc = 1-half_turn**2/6 if abs(half_turn)<1e-6 else math.sin(half_turn)/half_turn
    distance = v*dt*sinc
    dx = distance*math.cos(pose.yaw+half_turn)
    dy = distance*math.sin(pose.yaw+half_turn)
    return Pose(pose.x+dx, pose.y+dy, wrap_angle(pose.yaw+w*dt))


class WaypointFollower:
    def __init__(self, points: tuple[Point, ...], robot: RobotConfig, config: SimulationConfig):
        self.points, self.robot, self.config = points, robot, config
        self.index = min(1, len(points)-1)

    def command(self, estimate: Pose) -> tuple[float, float, bool]:
        if math.dist(estimate.xy, self.points[-1]) <= self.config.estimated_goal_tolerance_m:
            return 0.0, 0.0, True
        while self.index < len(self.points)-1 and math.dist(estimate.xy, self.points[self.index]) <= self.config.waypoint_tolerance_m:
            self.index += 1
        x,y = self.points[self.index]
        distance = math.dist(estimate.xy, (x,y))
        error = wrap_angle(math.atan2(y-estimate.y, x-estimate.x)-estimate.yaw)
        w = max(-self.robot.max_yaw_rate_rad_s, min(self.robot.max_yaw_rate_rad_s, 2.5*error))
        v = min(self.robot.max_speed_m_s, 1.5*distance)*max(0.0, math.cos(error))
        if abs(error) > 0.8:
            v = 0.0
        return v, w, False


class OdometryEstimator:
    def __init__(self, initial: Pose, config: LocalizationConfig, mode: str, seed: int):
        if mode not in ('ideal', 'odometry', 'fixes'):
            raise ValueError(f"Unknown localization mode: {mode}")
        self.pose, self.config, self.mode = initial, config, mode
        self.odom_rng = random.Random(seed)
        self.fix_rng = random.Random(seed ^ 0x5EED)
        self.next_fix_s = config.absolute_fix_period_s

    def predict(self, v: float, w: float, dt: float) -> None:
        if self.mode != 'ideal':
            c = self.config
            v = v*(1+c.speed_scale_bias) + self.odom_rng.gauss(0,c.speed_noise_std_m_s)
            w = w + c.yaw_rate_bias_rad_s + self.odom_rng.gauss(0,c.yaw_rate_noise_std_rad_s)
        self.pose = integrate(self.pose, v, w, dt)

    def fix_due(self, time_s: float) -> bool:
        return self.mode == 'fixes' and time_s+1e-10 >= self.next_fix_s

    def correct(self, measurement: Point, time_s: float) -> None:
        gain = self.config.absolute_fix_gain
        self.pose = Pose(self.pose.x+gain*(measurement[0]-self.pose.x),
                         self.pose.y+gain*(measurement[1]-self.pose.y), self.pose.yaw)
        while self.next_fix_s <= time_s+1e-10:
            self.next_fix_s += self.config.absolute_fix_period_s


@dataclass
class Trial:
    samples: list[dict]
    metrics: dict


def run_trial(scenario: Scenario, plan: Plan, seed: int = 42, mode: str = 'odometry') -> Trial:
    points = tuple(scenario.grid.to_world(cell) for cell in plan.cells)
    if not points:
        raise ValueError("Cannot follow an empty plan")
    heading = math.atan2(points[1][1]-points[0][1], points[1][0]-points[0][0]) if len(points)>1 else 0.0
    truth = Pose(*points[0], heading)
    estimator = OdometryEstimator(truth, scenario.localization, mode, seed)
    follower = WaypointFollower(points, scenario.robot, scenario.simulation)
    cfg = scenario.simulation
    samples = []
    elapsed, executed = 0.0, 0.0
    status = 'timeout'

    def sample(v: float = 0.0, w: float = 0.0, fix_applied: bool = False) -> None:
        samples.append({'time_s': elapsed,
                        **{f'true_{key}': value for key,value in zip(('x_m','y_m','yaw_rad'), asdict(truth).values())},
                        **{f'estimated_{key}': value for key,value in zip(('x_m','y_m','yaw_rad'), asdict(estimator.pose).values())},
                        'position_error_m': math.dist(truth.xy, estimator.pose.xy),
                        'command_speed_m_s': v, 'command_yaw_rate_rad_s': w,
                        'absolute_fix_applied': int(fix_applied)})

    sample()
    while True:
        v,w,arrived = follower.command(estimator.pose)
        if arrived:
            status = 'success' if math.dist(truth.xy, points[-1]) <= cfg.true_goal_tolerance_m else 'estimated_goal_only'
            break
        if elapsed >= cfg.max_time_s-1e-10:
            break
        dt = min(cfg.dt_s, cfg.max_time_s-elapsed)
        # Short straight sweeps conservatively cover the curved motion: the
        # extra radius bounds arc-to-chord deviation for each substep.
        segments = max(1, math.ceil(abs(w*dt)/0.02), math.ceil(abs(v*dt)/(scenario.grid.resolution_m/4)))
        candidate = truth
        safe = True
        for _ in range(segments):
            next_pose = integrate(candidate, v, w, dt/segments)
            sagitta = abs(v*w)*(dt/segments)**2/8
            if not scenario.grid.segment_is_free(candidate.xy, next_pose.xy, scenario.robot.body_radius_m+sagitta):
                safe = False
                break
            candidate = next_pose
        if not safe:
            status = 'collision'
            break
        executed += abs(v)*dt
        truth = candidate
        estimator.predict(v,w,dt)
        elapsed += dt
        applied = estimator.fix_due(elapsed)
        if applied:
            noise = scenario.localization.absolute_fix_noise_std_m
            # Ground truth is visible only to the synthetic sensor/evaluator.
            measurement = (truth.x+estimator.fix_rng.gauss(0,noise), truth.y+estimator.fix_rng.gauss(0,noise))
            estimator.correct(measurement, elapsed)
        sample(v,w,applied)
    errors = [row['position_error_m'] for row in samples]
    return Trial(samples, {'status': status, 'success': status == 'success',
                          'localization_model': mode, 'planner': 'P0', 'seed': seed,
                          'localization_rmse_m': math.sqrt(sum(e*e for e in errors)/len(errors)),
                          'final_position_error_m': errors[-1],
                          'true_goal_distance_m': math.dist(truth.xy, points[-1]),
                          'planned_path_length_m': plan.length_m,
                          'executed_path_length_m': executed,
                          'mission_time_s': elapsed, 'sample_count': len(samples),
                          'absolute_fix_count': sum(row['absolute_fix_applied'] for row in samples)})
