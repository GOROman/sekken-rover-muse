"""シミュレーションのエントリポイント。

使い方:
    python3 -m sekken_rover.sim balance   # 傾けて立て直す様子
    python3 -m sekken_rover.sim battery   # 残量低下 -> 自動帰還 -> 充電

どちらも実機ではなく仮想プラント上の検証。実機未検証。
"""

from __future__ import annotations

import math
import sys

from .battery import BatteryMonitor
from .imu import SimulatedImuSource
from .plant import TwoWheelPendulumPlant
from .robot import Robot, RobotState

DT = 0.005  # 制御周期 200Hz（企画書の「高速ループ」の目標値）


def make_robot(theta_deg: float = 0.0, x: float = 0.0, battery: float = 100.0,
               drain: float = 0.05) -> Robot:
    plant = TwoWheelPendulumPlant(theta=math.radians(theta_deg), x=x)
    imu = SimulatedImuSource(plant)
    return Robot(plant, imu, battery=BatteryMonitor(battery), drain_rate_pct_per_s=drain)


def run_balance(seconds: float = 6.0) -> None:
    robot = make_robot(theta_deg=10.0)
    robot.drive(0.0)
    steps = int(seconds / DT)
    for i in range(steps):
        # 2秒時点で外乱（横から軽く押された想定）
        if i == int(2.0 / DT):
            robot.plant.theta_dot += 0.6
        r = robot.step(DT)
        if i % int(0.5 / DT) == 0:
            p = robot.plant
            print(
                f"t={p.t:4.1f}s theta={math.degrees(p.theta):+6.2f}deg "
                f"x={p.x:+5.2f}m state={r.state.value}"
            )
    p = robot.plant
    print(f"結果: 最終傾き {math.degrees(p.theta):+.2f} 度 / 状態 {robot.state.value}")


def run_battery(seconds: float = 240.0) -> None:
    # ドックから 1.2m 離れた場所、残量 24%、速めの放電で帰還を観察する
    robot = make_robot(x=1.2, battery=24.0, drain=0.25)
    robot.drive(0.2)
    last_state = robot.state
    steps = int(seconds / DT)
    for i in range(steps):
        r = robot.step(DT)
        for ev in r.events:
            print(f"t={robot.plant.t:6.1f}s event={ev} battery={robot.battery.level_pct:.1f}%")
        if robot.state != last_state:
            print(
                f"t={robot.plant.t:6.1f}s state -> {robot.state.value} "
                f"(x={robot.plant.x:+.2f}m, battery={robot.battery.level_pct:.1f}%)"
            )
            last_state = robot.state
        if robot.state == RobotState.BALANCING and robot.battery.is_charged:
            break
    print(
        f"結果: 状態 {robot.state.value} / 残量 {robot.battery.level_pct:.1f}% / "
        f"位置 x={robot.plant.x:+.2f}m"
    )


def main(argv: list[str]) -> int:
    scenario = argv[1] if len(argv) > 1 else "balance"
    if scenario == "balance":
        run_balance()
    elif scenario == "battery":
        run_battery()
    else:
        print(f"unknown scenario: {scenario}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
