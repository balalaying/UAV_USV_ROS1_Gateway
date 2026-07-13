import math
import unittest

from uav_usv_mission.capture_planner import CapturePlanner
from uav_usv_mission.target_predictor import TargetPredictor
from uav_usv_mission.target_predictor import TargetState


class CaptureAlgorithmsTest(unittest.TestCase):
    def test_prediction_uses_target_velocity(self):
        predictor = TargetPredictor(horizon=10.0, step=1.0)
        prediction = predictor.predict(TargetState(2.0, 3.0, 0.5, 1.2, -0.4))
        self.assertAlmostEqual(prediction[-1].x, 14.0)
        self.assertAlmostEqual(prediction[-1].y, -1.0)

    def test_two_uavs_receive_distinct_points_and_usv_intercepts(self):
        state = TargetState(0.0, 0.0, 0.5, 1.0, 0.0)
        prediction = TargetPredictor(horizon=12.0).predict(state)
        plan = CapturePlanner(capture_radius=18.0).plan(
            state, prediction, ['uav_01', 'uav_02'], ['usv_01']
        )
        first = plan.assignments['uav_01']
        second = plan.assignments['uav_02']
        separation = math.hypot(first.x - second.x, first.y - second.y)
        self.assertAlmostEqual(separation, 36.0, places=5)
        self.assertNotEqual(first.z, second.z)
        self.assertGreater(plan.assignments['usv_01'].x, state.x)
        self.assertTrue(
            plan.assignments['usv_01'].role.startswith('surface_interceptor')
        )


if __name__ == '__main__':
    unittest.main()
