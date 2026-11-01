"""IMU 接続点のテスト。

- 仮想 IMU がインターフェースを満たし、時刻が単調増加すること
- 実機用スタブが未実装であることを明示すること（誤作動防止）
- 任意の ImuSource 実装に差し替えても制御が回ること
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sekken_rover.controller import BalanceController
from sekken_rover.imu import (
    ImuSample,
    ImuSource,
    M5StackImuSource,
    SimulatedImuSource,
)
from sekken_rover.plant import TwoWheelPendulumPlant

DT = 0.005


class _ConstantSource(ImuSource):
    """差し替え可能性を確認するための最小実装。"""

    def __init__(self) -> None:
        self._t = 0.0

    def read(self) -> ImuSample:
        self._t += DT
        return ImuSample(t=self._t, pitch=0.05, pitch_rate=0.0)


class ImuInterfaceTest(unittest.TestCase):
    def test_simulated_source_timestamps_increase(self):
        plant = TwoWheelPendulumPlant()
        src = SimulatedImuSource(plant)
        last = -1.0
        for _ in range(20):
            s = src.read()
            self.assertGreaterEqual(s.t, last)
            last = s.t
            plant.step(0.0, DT)

    def test_real_imu_stub_is_explicitly_unimplemented(self):
        src = M5StackImuSource()
        with self.assertRaises(NotImplementedError):
            src.read()

    def test_controller_accepts_any_imu_source(self):
        c = BalanceController()
        src: ImuSource = _ConstantSource()
        accel = c.update(src.read(), velocity=0.0, target_velocity=0.0)
        self.assertGreater(accel, 0.0)


if __name__ == "__main__":
    unittest.main()
