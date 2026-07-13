import math
import unittest

from uav_usv_mission.capture_planner import CapturePlanner
from uav_usv_mission.capture_planner import TYPE_UAV
from uav_usv_mission.capture_planner import TYPE_USV
from uav_usv_mission.capture_planner import VehicleKinematics
from uav_usv_mission.target_predictor import TargetPredictor
from uav_usv_mission.target_predictor import TargetState


def vehicle(vehicle_id, vehicle_type, x, y, yaw=0.0):
    return VehicleKinematics(vehicle_id, vehicle_type, x, y, 0.0, yaw=yaw)


class CaptureAlgorithmsTest(unittest.TestCase):
    def test_zero_turn_rate_matches_constant_heading(self):
        predictor = TargetPredictor(horizon=4.0, step=1.0)
        prediction = predictor.predict(TargetState(
            x=1.0, y=2.0, z=0.5, vx=3.0, vy=0.0, yaw=0.0
        ))
        self.assertAlmostEqual(prediction[-1].x, 13.0)
        self.assertAlmostEqual(prediction[-1].y, 2.0)

    def test_ctrv_predicts_turning_target(self):
        predictor = TargetPredictor(horizon=5.0, step=1.0)
        prediction = predictor.predict(TargetState(
            x=0.0,
            y=0.0,
            z=0.5,
            vx=4.0,
            vy=0.0,
            yaw=0.0,
            yaw_rate=0.2,
        ))
        self.assertGreater(prediction[-1].y, 0.0)
        self.assertLess(prediction[-1].x, 20.0)
        self.assertAlmostEqual(prediction[-1].yaw, 1.0)

    def test_four_uavs_and_two_usvs_receive_unique_typed_roles(self):
        target = TargetState(20.0, 5.0, 0.5, 2.0, 0.5)
        prediction = TargetPredictor().predict(target)
        vehicles = [
            vehicle('alpha', TYPE_UAV, -20.0, -15.0),
            vehicle('bravo', TYPE_UAV, -10.0, -15.0),
            vehicle('charlie', TYPE_UAV, 0.0, -15.0),
            vehicle('delta', TYPE_UAV, 10.0, -15.0),
            vehicle('surface_a', TYPE_USV, -5.0, 2.0),
            vehicle('surface_b', TYPE_USV, -5.0, 12.0),
        ]
        plan = CapturePlanner(capture_radius=24.0).plan(
            target, prediction, vehicles
        )
        self.assertEqual(len(plan.assignments), 6)
        roles = [item.role for item in plan.assignments.values()]
        self.assertEqual(len(roles), len(set(roles)))
        self.assertEqual(
            sum(role.startswith('air_observer_') for role in roles), 4
        )
        self.assertEqual(
            sum(role.startswith('surface_interceptor_') for role in roles), 2
        )

    def test_assignment_uses_vehicle_position_not_identifier_order(self):
        target = TargetState(0.0, 0.0, 0.5, 2.0, 0.0)
        prediction = TargetPredictor().predict(target)
        vehicles = [
            vehicle('zulu', TYPE_UAV, 5.0, 20.0),
            vehicle('alpha', TYPE_UAV, 5.0, -20.0),
        ]
        plan = CapturePlanner(capture_radius=18.0).plan(
            target, prediction, vehicles
        )
        self.assertGreater(plan.assignments['zulu'].y, 0.0)
        self.assertLess(plan.assignments['alpha'].y, 0.0)

    def test_failed_vehicle_is_removed_and_remaining_roles_reallocate(self):
        target = TargetState(10.0, 0.0, 0.5, 1.0, 0.0)
        prediction = TargetPredictor().predict(target)
        planner = CapturePlanner(capture_radius=18.0)
        full = [
            vehicle('u1', TYPE_UAV, -10.0, -10.0),
            vehicle('u2', TYPE_UAV, -10.0, 10.0),
            vehicle('u3', TYPE_UAV, 0.0, 15.0),
            vehicle('s1', TYPE_USV, -5.0, 0.0),
        ]
        first = planner.plan(target, prediction, full)
        remaining = [item for item in full if item.vehicle_id != 'u2']
        second = planner.plan(
            target,
            prediction,
            remaining,
            {key: value.role for key, value in first.assignments.items()},
        )
        self.assertNotIn('u2', second.assignments)
        self.assertEqual(len(second.assignments), 3)
        self.assertEqual(
            len({item.role for item in second.assignments.values()}), 3
        )


if __name__ == '__main__':
    unittest.main()
