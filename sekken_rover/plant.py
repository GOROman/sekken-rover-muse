"""2輪倒立振子の物理モデル（シミュレーション専用）。

車輪の加速度を入力とし、重心まわりの回転運動を解く簡易モデル。
実機の代替ではない。到着後のゲイン調整は実機で行う前提で、
ここでは「制御の骨格が正しい向きに働くこと」の検証に使う。
"""

from __future__ import annotations

import math


class TwoWheelPendulumPlant:
    """倒立振子プラント。

    状態:
        theta: 傾き角 [rad]（前傾が正）
        theta_dot: 角速度 [rad/s]
        x: 位置 [m]
        v: 速度 [m/s]

    運動方程式（簡易化）:
        theta_ddot = (g / L) * sin(theta) - (a / L) * cos(theta)
        x_ddot = a
    ここで a は車輪加速度 [m/s^2]、L は車軸から重心までの距離 [m]。
    """

    GRAVITY = 9.80665

    def __init__(
        self,
        pendulum_length: float = 0.12,
        max_accel: float = 6.0,
        theta: float = 0.0,
        x: float = 0.0,
    ) -> None:
        self.L = pendulum_length
        self.max_accel = max_accel
        self.theta = theta
        self.theta_dot = 0.0
        self.x = x
        self.v = 0.0
        self.t = 0.0
        self.last_accel = 0.0

    def step(self, accel_cmd: float, dt: float) -> None:
        a = max(-self.max_accel, min(self.max_accel, accel_cmd))
        self.last_accel = a
        theta_ddot = (self.GRAVITY / self.L) * math.sin(self.theta) - (
            a / self.L
        ) * math.cos(self.theta)
        # 半陰的オイラー積分
        self.theta_dot += theta_ddot * dt
        self.theta += self.theta_dot * dt
        self.v += a * dt
        self.x += self.v * dt
        self.t += dt

    @property
    def fallen(self) -> bool:
        """転倒判定（約34度以上）。"""
        return abs(self.theta) > 0.6
