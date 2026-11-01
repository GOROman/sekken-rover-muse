"""バッテリー低下時の自動帰還テストハーネス。"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sekken_rover.battery import BatteryEvent, BatteryMonitor
from sekken_rover.imu import SimulatedImuSource
from sekken_rover.plant import TwoWheelPendulumPlant
from sekken_rover.robot import Robot, RobotState

DT = 0.005


def make_robot(x: float, battery: float, drain: float) -> Robot:
    plant = TwoWheelPendulumPlant(x=x)
    return Robot(
        plant,
        SimulatedImuSource(plant),
        battery=BatteryMonitor(battery),
        drain_rate_pct_per_s=drain,
    )


class BatteryMonitorTest(unittest.TestCase):
    def test_low_event_fires_once(self):
        b = BatteryMonitor(25.0)
        events = [b.drain(1.0, 3.0) for _ in range(5)]  # 25 -> 10%
        self.assertEqual(events.count(BatteryEvent.LOW), 1)

    def test_no_return_above_threshold(self):
        b = BatteryMonitor(50.0)
        b.drain(1.0, 5.0)
        self.assertFalse(b.should_return)


class AutoReturnTest(unittest.TestCase):
    def test_low_battery_triggers_return_and_docking(self):
        robot = make_robot(x=1.2, battery=24.0, drain=0.25)
        robot.drive(0.2)
        saw = {"low": False, "returning": False, "docked": False, "charged": False}
        for _ in range(int(300.0 / DT)):
            r = robot.step(DT)
            if "battery_low" in r.events:
                saw["low"] = True
            if robot.state == RobotState.RETURNING:
                saw["returning"] = True
            if "docked" in r.events:
                saw["docked"] = True
            if robot.state == RobotState.BALANCING and robot.battery.is_charged:
                saw["charged"] = True
                break
        self.assertTrue(saw["low"], "battery_low イベントが発火すること")
        self.assertTrue(saw["returning"], "自動帰還状態へ遷移すること")
        self.assertTrue(saw["docked"], "ドックへ到達すること")
        self.assertTrue(saw["charged"], "充電完了で通常状態へ戻ること")
        self.assertLessEqual(abs(robot.plant.x - Robot.DOCK_X), Robot.DOCK_TOLERANCE_M + 0.02)

    def test_healthy_battery_keeps_driving(self):
        robot = make_robot(x=0.0, battery=90.0, drain=0.01)
        robot.drive(0.2)
        for _ in range(int(5.0 / DT)):
            robot.step(DT)
        self.assertEqual(robot.state, RobotState.BALANCING)


if __name__ == "__main__":
    unittest.main()
