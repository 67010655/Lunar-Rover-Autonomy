import copy
import math
from pathlib import Path
import unittest

from lunar_rover.planning import Plan, plan_p0
from lunar_rover.scenario import load_scenario, scenario_from_dict
from lunar_rover.simulation import OdometryEstimator, Pose, WaypointFollower, integrate, run_trial

SCENARIO = Path(__file__).resolve().parents[1]/'configs/scenarios/synthetic_obstacles.json'


class SimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scenario = load_scenario(SCENARIO)
        s = cls.scenario
        cls.plan = plan_p0(s.grid,s.start,s.goal,s.robot.body_radius_m+s.robot.planning_margin_m)

    def test_ideal_closed_loop_reaches_goal_without_truth_feedback(self):
        trial = run_trial(self.scenario,self.plan,mode='ideal')
        self.assertTrue(trial.metrics['success'])
        self.assertLess(trial.metrics['localization_rmse_m'],1e-10)
        self.assertLessEqual(trial.metrics['true_goal_distance_m'],self.scenario.simulation.true_goal_tolerance_m)
        radius = self.scenario.robot.body_radius_m
        for a,b in zip(trial.samples,trial.samples[1:]):
            self.assertTrue(self.scenario.grid.segment_is_free((a['true_x_m'],a['true_y_m']),
                                                             (b['true_x_m'],b['true_y_m']),radius))

    def test_same_seed_reproduces_trajectory_and_new_seed_changes_noise(self):
        first = run_trial(self.scenario,self.plan,seed=42)
        again = run_trial(self.scenario,self.plan,seed=42)
        different = run_trial(self.scenario,self.plan,seed=43)
        self.assertEqual(first,again)
        self.assertNotEqual(first.samples,different.samples)
        self.assertGreater(first.metrics['localization_rmse_m'],0)

    def test_controller_responds_to_estimated_position(self):
        follower = WaypointFollower(((0,0),(3,0)),self.scenario.robot,self.scenario.simulation)
        self.assertGreater(follower.command(Pose(0,0,0))[0],0)
        self.assertTrue(follower.command(Pose(3,0,0))[2])

    def test_odometry_predict_has_no_truth_input(self):
        est = OdometryEstimator(Pose(0,0,0),self.scenario.localization,'ideal',42)
        est.predict(1.0,0.0,2.0)
        self.assertEqual(est.pose,Pose(2.0,0.0,0.0))

    def test_fix_schedule_and_measurement_correction(self):
        est = OdometryEstimator(Pose(0,0,0),self.scenario.localization,'fixes',42)
        self.assertFalse(est.fix_due(0.5))
        self.assertTrue(est.fix_due(1.0))
        est.correct((1,2),1.0)
        self.assertAlmostEqual(est.pose.x,0.35)
        self.assertAlmostEqual(est.pose.y,0.7)
        self.assertFalse(est.fix_due(1.1))
        trial = run_trial(self.scenario,self.plan,mode='fixes')
        self.assertGreater(trial.metrics['absolute_fix_count'],0)

    def test_exact_arc_integration(self):
        pose = integrate(Pose(0,0,0),1,1,math.pi/2)
        self.assertAlmostEqual(pose.x,1)
        self.assertAlmostEqual(pose.y,1)

    def test_timeout_and_invalid_config(self):
        raw = copy.deepcopy(self.scenario.raw)
        raw['simulation']['max_time_s'] = 0.1
        self.assertEqual(run_trial(scenario_from_dict(raw),self.plan,mode='ideal').metrics['status'],'timeout')
        for path,value in ((('simulation','dt_s'),0),(('robot','body_radius_m'),-1),
                           (('localization','absolute_fix_gain'),2),(('localization','speed_noise_std_m_s'),float('nan'))):
            raw = copy.deepcopy(self.scenario.raw)
            raw[path[0]][path[1]] = value
            with self.subTest(path=path), self.assertRaises(ValueError):
                scenario_from_dict(raw)

    def test_success_when_start_equals_goal(self):
        raw = copy.deepcopy(self.scenario.raw)
        raw['goal_rc'] = raw['start_rc']
        scenario = scenario_from_dict(raw)
        trial = run_trial(scenario,Plan((scenario.start,),0,1),mode='ideal')
        self.assertTrue(trial.metrics['success'])
        self.assertEqual(trial.metrics['mission_time_s'],0)

    def test_estimated_goal_is_not_mistaken_for_true_success(self):
        raw = copy.deepcopy(self.scenario.raw)
        raw['grid']['obstacle_rectangles_rc'] = []
        raw['goal_rc'] = [8,30]
        raw['localization'].update(speed_scale_bias=1.0, yaw_rate_bias_rad_s=0,
                                   speed_noise_std_m_s=0, yaw_rate_noise_std_rad_s=0)
        scenario = scenario_from_dict(raw)
        plan = plan_p0(scenario.grid,scenario.start,scenario.goal,0.3)
        trial = run_trial(scenario,plan,mode='odometry')
        self.assertEqual(trial.metrics['status'],'estimated_goal_only')
        self.assertFalse(trial.metrics['success'])
        self.assertGreater(trial.metrics['true_goal_distance_m'],scenario.simulation.true_goal_tolerance_m)

    def test_collision_is_reported_and_vehicle_does_not_enter_obstacle(self):
        # Deliberately unsafe route, independent of the planner, exercises guard.
        trial = run_trial(self.scenario,Plan((self.scenario.start,self.scenario.goal),1,0),mode='ideal')
        self.assertEqual(trial.metrics['status'],'collision')
        last = trial.samples[-1]
        self.assertTrue(self.scenario.grid.segment_is_free((last['true_x_m'],last['true_y_m']),
                                                         (last['true_x_m'],last['true_y_m']),self.scenario.robot.body_radius_m))
