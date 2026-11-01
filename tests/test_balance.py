"""倒立振子のバランス制御テスト（シミュレーション検証）。"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sekken_rover.controller import BalanceController
from sekken_rover.imu import ImuSample, SimulatedImuSource
from sekken_rover.plant import TwoWheelPendulumPlant
from sekken_rover.robot import Robot, RobotState

DT = 0.005


class ControllerDirectionTest(unittest.TestCase):
    def test_correction_opposes_forward_tilt(self):
        """前傾したら、立て直す向き（前向き加速度）の補正が出ること。"""
        c = BalanceController()
        sample = ImuSample(t=0.0, pitch=0.10, pitch_rate=0.0)
        accel = c.update(sample, velocity=0.0, target_velocity=0.0)
        self.assertGreater(accel, 0.0)

    def test_correction_opposes_backward_tilt(self):
        c = BalanceController()
        sample = ImuSample(t=0.0, pitch=-0.10, pitch_rate=0.0)
        accel = c.update(sample, velocity=0.0, target_velocity=0.0)
        self.assertLess(accel, 0.0)


class ClosedLoopBalanceTest(unittest.TestCase):
    def _run(self, robot: Robot, seconds: float, disturb_at: float | None = None):
        steps = int(seconds / DT)
        max_late_tilt = 0.0
        for i in range(steps):
            if disturb_at is not None and i == int(disturb_at / DT):
                robot.plant.theta_dot += 0.6  # 外乱: 軽く押された想定
            robot.step(DT)
            if robot.plant.t > seconds - 2.0:
                max_late_tilt = max(max_late_tilt, abs(robot.plant.theta))
        return max_late_tilt

    def test_recovers_from_10deg_tilt(self):
        plant = TwoWheelPendulumPlant(theta=math.radians(10.0))
        robot = Robot(plant, SimulatedImuSource(plant))
        max_late = self._run(robot, 6.0)
        self.assertEqual(robot.state, RobotState.BALANCING)
        self.assertLess(math.degrees(max_late), 3.0)

    def test_recovers_from_push_disturbance(self):
        plant = TwoWheelPendulumPlant(theta=0.0)
        robot = Robot(plant, SimulatedImuSource(plant))
        max_late = self._run(robot, 7.0, disturb_at=2.0)
        self.assertEqual(robot.state, RobotState.BALANCING)
        self.assertLess(math.degrees(max_late), 3.0)

    def test_drives_at_target_velocity(self):
        plant = TwoWheelPendulumPlant(theta=0.0)
        robot = Robot(plant, SimulatedImuSource(plant))
        robot.drive(0.3)
        for _ in range(int(6.0 / DT)):
            robot.step(DT)
        self.assertEqual(robot.state, RobotState.BALANCING)
        self.assertAlmostEqual(plant.v, 0.3, delta=0.08)
        self.assertGreater(plant.x, 1.0)


if __name__ == "__main__":
    unittest.main()
