"""ロボット全体の状態機械。

企画書の役割分担どおり、ここは「身体レイヤー」の高速ループ側。
Muse 側からは速度指令（前進・停止・帰還）だけを受け取り、
姿勢制御とバッテリー判断はこの中で完結させる。

状態遷移:
    BALANCING --(battery_low)--> RETURNING --(ドック到達)--> CHARGING
    CHARGING  --(満充電)--> BALANCING
    任意      --(転倒)--> FALLEN（モーター停止、手動復帰待ち）
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field

from .battery import BatteryEvent, BatteryMonitor
from .controller import BalanceController
from .imu import ImuSource
from .plant import TwoWheelPendulumPlant


class RobotState(enum.Enum):
    BALANCING = "balancing"
    RETURNING = "returning"
    CHARGING = "charging"
    FALLEN = "fallen"


@dataclass
class StepResult:
    state: RobotState
    accel_cmd: float
    events: list[str] = field(default_factory=list)


class Robot:
    DOCK_X = 0.0  # チャージャーの位置 [m]
    DOCK_TOLERANCE_M = 0.05
    RETURN_SPEED = 0.35  # 帰還時の目標速度 [m/s]
    CRITICAL_RETURN_SPEED = 0.18

    def __init__(
        self,
        plant: TwoWheelPendulumPlant,
        imu: ImuSource,
        controller: BalanceController | None = None,
        battery: BatteryMonitor | None = None,
        drain_rate_pct_per_s: float = 0.05,
    ) -> None:
        self.plant = plant
        self.imu = imu
        self.controller = controller or BalanceController()
        self.battery = battery or BatteryMonitor()
        self.drain_rate = drain_rate_pct_per_s
        self.state = RobotState.BALANCING
        self.user_target_velocity = 0.0

    # --- Muse 側からの指令（企画書の連動APIに対応） ---

    def drive(self, velocity: float) -> None:
        """前進/後退の速度指令 [m/s]。帰還中・充電中・転倒時は受け付けない。"""
        if self.state == RobotState.BALANCING:
            self.user_target_velocity = velocity

    def go_home(self) -> None:
        """チャージャーへ帰還する（企画書の go_home コマンド）。"""
        if self.state == RobotState.BALANCING:
            self.state = RobotState.RETURNING

    # --- 制御ループ ---

    def step(self, dt: float) -> StepResult:
        events: list[str] = []
        p = self.plant

        if self.state == RobotState.FALLEN:
            return StepResult(self.state, 0.0, events)

        if p.fallen:
            self.state = RobotState.FALLEN
            events.append("fallen")
            p.step(0.0, dt)
            return StepResult(self.state, 0.0, events)

        if self.state == RobotState.CHARGING:
            self.battery.charge(dt)
            # ドックに機械的に保持されている状態: 倒立制御は止め、
            # 姿勢はドック側で支えられるものとして静定させる
            p.theta = 0.0
            p.theta_dot = 0.0
            p.v = 0.0
            p.t += dt
            if self.battery.is_charged:
                self.state = RobotState.BALANCING
                events.append("charge_complete")
            return StepResult(self.state, 0.0, events)

        # バッテリー監視（走行中のみ放電）
        ev = self.battery.drain(dt, self.drain_rate)
        if ev == BatteryEvent.LOW:
            events.append(BatteryEvent.LOW.value)
        elif ev == BatteryEvent.CRITICAL:
            events.append(BatteryEvent.CRITICAL.value)
        if self.battery.should_return and self.state == RobotState.BALANCING:
            self.state = RobotState.RETURNING
            events.append("auto_return_started")

        # 目標速度の決定
        if self.state == RobotState.RETURNING:
            speed = (
                self.CRITICAL_RETURN_SPEED
                if self.battery.is_critical
                else self.RETURN_SPEED
            )
            dx = self.DOCK_X - p.x
            if abs(dx) <= self.DOCK_TOLERANCE_M and abs(p.v) < 0.08:
                self.state = RobotState.CHARGING
                events.append("docked")
                p.step(0.0, dt)
                return StepResult(self.state, 0.0, events)
            target_velocity = speed if dx > 0 else -speed
        else:
            target_velocity = self.user_target_velocity

        sample = self.imu.read()
        accel = self.controller.update(sample, p.v, target_velocity)
        p.step(accel, dt)
        return StepResult(self.state, accel, events)
