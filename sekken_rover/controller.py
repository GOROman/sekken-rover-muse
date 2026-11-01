"""倒立振子のバランス制御（PID）。

2段構成:
- 外側ループ: 速度誤差から「目標の傾き」を作る（進みたい向きに少し倒す）
- 内側ループ: 傾き誤差に対する PID で車輪加速度を出す

ゲイン初期値はシミュレーション（plant.py の L=0.12m モデル）に合わせた
仮値。実機では docs/IMU_INTEGRATION.md の手順で再調整する。
"""

from __future__ import annotations

from dataclasses import dataclass

from .imu import ImuSample


@dataclass
class PidGains:
    kp: float = 28.0  # 傾き誤差 [rad] -> 加速度 [m/s^2]
    ki: float = 0.6
    kd: float = 3.2  # 角速度 [rad/s] -> 加速度 [m/s^2]


class BalanceController:
    def __init__(
        self,
        gains: PidGains | None = None,
        velocity_gain: float = 0.055,
        max_target_tilt: float = 0.12,
        max_accel: float = 6.0,
    ) -> None:
        self.gains = gains or PidGains()
        self.velocity_gain = velocity_gain
        self.max_target_tilt = max_target_tilt
        self.max_accel = max_accel
        self._integral = 0.0
        self._last_t: float | None = None
        self.target_tilt = 0.0

    def reset(self) -> None:
        self._integral = 0.0
        self._last_t = None
        self.target_tilt = 0.0

    def update(self, sample: ImuSample, velocity: float, target_velocity: float) -> float:
        """車輪加速度指令 [m/s^2] を返す。

        符号の約束: 前傾（pitch > 0）のときは前向きの加速度を出し、
        車輪を重心の下へ戻して倒立を回復させる。
        """
        # 外側ループ: 速度誤差 -> 目標傾き
        tilt_cmd = self.velocity_gain * (target_velocity - velocity)
        self.target_tilt = max(
            -self.max_target_tilt, min(self.max_target_tilt, tilt_cmd)
        )

        # 内側ループ: PID
        error = sample.pitch - self.target_tilt
        if self._last_t is not None:
            dt = max(sample.t - self._last_t, 1e-4)
            self._integral += error * dt
            # 積分の蓄積すぎ防止
            self._integral = max(-0.5, min(0.5, self._integral))
        self._last_t = sample.t

        g = self.gains
        accel = g.kp * error + g.ki * self._integral + g.kd * sample.pitch_rate
        return max(-self.max_accel, min(self.max_accel, accel))
