"""IMU 抽象化レイヤー。

倒立振子の制御ループは、このモジュールの ``ImuSource`` インターフェース
だけに依存する。実機の M5Stack IMU Unit が到着したら ``M5StackImuSource``
の中身を実装するだけで、制御側のコードは一切変更しない。

接続点と到着後の作業は docs/IMU_INTEGRATION.md を参照。
"""

from __future__ import annotations

import abc
import math
import random
from dataclasses import dataclass


@dataclass(frozen=True)
class ImuSample:
    """1回分の姿勢計測。

    Attributes:
        t: サンプル時刻 [s]（単調増加）
        pitch: 前後の傾き角 [rad]。前に倒れる向きが正。
        pitch_rate: 傾き角速度 [rad/s]
    """

    t: float
    pitch: float
    pitch_rate: float


class ImuSource(abc.ABC):
    """姿勢センサーの接続点。制御ループはこの型だけを見る。"""

    @abc.abstractmethod
    def read(self) -> ImuSample:
        """最新の姿勢サンプルを1つ返す。"""
        raise NotImplementedError


class SimulatedImuSource(ImuSource):
    """物理シミュレーション（plant）から姿勢を読む仮想 IMU。

    実センサー相当のノイズと量子化を付ける。到着後の実機データとの
    差し替えポイントを明確にするため、plant への参照だけを持つ。
    """

    def __init__(
        self,
        plant,
        noise_rad: float = 0.0015,
        rate_noise_rad_s: float = 0.01,
        seed: int = 42,
    ) -> None:
        self._plant = plant
        self._noise = noise_rad
        self._rate_noise = rate_noise_rad_s
        self._rng = random.Random(seed)

    def read(self) -> ImuSample:
        p = self._plant
        return ImuSample(
            t=p.t,
            pitch=p.theta + self._rng.gauss(0.0, self._noise),
            pitch_rate=p.theta_dot + self._rng.gauss(0.0, self._rate_noise),
        )


class M5StackImuSource(ImuSource):
    """実機用スタブ：M5Stack IMU Unit（注文 M5-115515、到着見込み 2026-11-10 頃）。

    ★ IMU 到着後にここを実装する ★
    - 接続: I2C（Unit の Grove ポート経由を想定）
    - 実装手順と調整項目: docs/IMU_INTEGRATION.md
    - 未実装の間は NotImplementedError を送出し、実機で誤って
      シミュレーション前提のまま動くことを防ぐ。
    """

    def __init__(self, i2c_bus: int = 0, address: int | None = None) -> None:
        self.i2c_bus = i2c_bus
        self.address = address  # 到着後に実チップを確認して確定する

    def read(self) -> ImuSample:
        raise NotImplementedError(
            "M5Stack IMU Unit は未接続です。到着後に docs/IMU_INTEGRATION.md の"
            "手順でこのクラスを実装してください。"
        )


def tilt_from_accel(ax: float, az: float) -> float:
    """加速度センサーの値から静的な傾き角を求める補助関数（実機実装用）。

    pitch = atan2(ax, az) [rad]
    """
    return math.atan2(ax, az)
